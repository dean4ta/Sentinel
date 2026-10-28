// C++ port of sentinel_drive/cmd_vel_serial.py -- that file stays the
// authoritative behavior spec. Same params (drive_params.yaml), same
// topics, same serial protocol: "fl,fr,bl,br\n" out at publish_rate Hz,
// "FL <count> <us>" encoder lines in (rpm from Pico-clock deltas).
#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentinel_msgs/msg/encoder_counts.hpp"
#include "sentinel_msgs/msg/gripper_sensors.hpp"
#include "sentinel_msgs/msg/wheel_speeds.hpp"
#include "std_msgs/msg/string.hpp"

using geometry_msgs::msg::Twist;
using sentinel_msgs::msg::EncoderCounts;
using sentinel_msgs::msg::GripperSensors;
using sentinel_msgs::msg::WheelSpeeds;
using std_msgs::msg::String;

namespace {

const std::array<std::string, 4> kWheels = {"FL", "FR", "BL", "BR"};

// /cmd_vel carries SI units; these ceilings normalize to the [-1,1] wire
// protocol. First-pass estimates, not trusted exact values (TODO measure).
constexpr double kVMaxLinearMps = 2.29;
constexpr double kWheelMaxRpm = 312.0;

// Encoder ticks per output-shaft rev (measured 2026-09-24).
constexpr double kTicksPerRev = 532.0;

// Per-wheel PWM sign: +1 = forward. FR/BR flipped (observed 2026-09-24).
const std::map<std::string, double> kWheelDirections = {
    {"FL", 1.0}, {"FR", -1.0}, {"BL", 1.0}, {"BR", -1.0}};

// Encoder count sign during forward motion (mirrored mounting).
const std::map<std::string, double> kEncDirections = {
    {"FL", -1.0}, {"FR", 1.0}, {"BL", -1.0}, {"BR", 1.0}};

// Encoder silence longer than this disables PID output (neutral PWM).
constexpr double kEncStaleS = 0.25;

struct WheelPID {
  // Error in rpm -> normalized PWM output [-out_max, out_max].
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double out_max = 0.0;
  double integral = 0.0;
  double prev_error = 0.0;

  double update(double error, double dt)
  {
    if (dt <= 0.0) {
      return 0.0;
    }
    integral += error * dt;
    // Anti-windup: integral term limited to ±out_max of authority.
    integral = std::clamp(integral, -out_max, out_max);
    double deriv = (error - prev_error) / dt;
    prev_error = error;
    double out = kp * error + ki * integral + kd * deriv;
    return std::clamp(out, -out_max, out_max);
  }
};

}  // namespace

class CmdVelSerial : public rclcpp::Node
{
public:
  CmdVelSerial()
  : Node("cmd_vel_serial")
  {
    std::string port = declare_parameter("serial_port", "/dev/sentinel_pico");
    int baud = declare_parameter("baud", 460800);
    double rate = declare_parameter("publish_rate", 100.0);

    pid_enabled_ = declare_parameter("pid_enabled", true);
    ff_enabled_ = declare_parameter("feed_forward_enabled", true);
    ik_wheelbase_m_ = declare_parameter("ik_wheelbase_m", 0.35);
    ik_roller_swap_ = declare_parameter("ik_roller_swap", false);

    for (const auto & w : kWheels) {
      std::string l = w;
      std::transform(l.begin(), l.end(), l.begin(),
                     [](unsigned char c) {return std::tolower(c);});
      pids_[w] = WheelPID{
        declare_parameter(l + "_kp", 0.005),
        declare_parameter(l + "_ki", 0.0),
        declare_parameter(l + "_kd", 0.0),
        declare_parameter(l + "_output_limit", 0.45)};
      kS_[w] = declare_parameter(l + "_kS", 0.05);
      kV_[w] = declare_parameter(l + "_kV", 0.85);
      ema_alpha_[w] = declare_parameter(l + "_ema_alpha", 0.4);
      wheel_targets_[w] = 0.0;
      counts_[w] = 0;
      enc_us_[w] = 0;
      last_counts_[w] = 0;
      last_enc_us_[w] = 0;
      measured_rpm_[w] = 0.0;
      raw_rpm_[w] = 0.0;
    }

    open_serial(port, baud);

    cmd_vel_sub_ = create_subscription<Twist>(
      "/cmd_vel", 10,
      [this](const Twist::SharedPtr m) {latest_ = *m;});
    wheel_cmd_sub_ = create_subscription<String>(
      "/wheel_cmd", 10,
      [this](const String::SharedPtr m) {wheel_cmd_cb(m);});
    encoder_pub_ = create_publisher<EncoderCounts>("/encoder", 10);
    wheel_speeds_pub_ = create_publisher<WheelSpeeds>("/wheel_speeds", 10);
    gripper_pub_ = create_publisher<GripperSensors>("/gripper", 10);

    auto now = get_clock()->now();
    last_wheel_cmd_ = now;
    last_enc_time_ = now;
    last_pid_time_ = now;

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      [this] {tick();});

    RCLCPP_INFO(get_logger(),
      "Gateway on %s @ %d: writing at %.1f Hz, publishing EncoderCounts "
      "on /encoder.", port.c_str(), baud, rate);
  }

private:
  void open_serial(const std::string & port, int baud)
  {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s: %s",
        port.c_str(), std::strerror(errno));
      throw std::runtime_error("serial open failed");
    }
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr %s: %s",
        port.c_str(), std::strerror(errno));
      throw std::runtime_error("tcgetattr failed");
    }
    cfmakeraw(&tty);
    speed_t spd = B460800;
    switch (baud) {
      case 115200: spd = B115200; break;
      case 230400: spd = B230400; break;
      case 921600: spd = B921600; break;
      case 460800: default: spd = B460800; break;
    }
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr %s: %s",
        port.c_str(), std::strerror(errno));
      throw std::runtime_error("tcsetattr failed");
    }
  }

  void wheel_cmd_cb(const String::SharedPtr msg)
  {
    // "FL 30" debug format: one wheel at target RPM, rest zeroed.
    std::istringstream is(msg->data);
    std::string w;
    double rpm;
    if (!(is >> w >> rpm)) {
      return;
    }
    std::string rest;
    if (is >> rest) {
      return;  // exactly two fields, like the Python original
    }
    std::transform(w.begin(), w.end(), w.begin(),
                   [](unsigned char c) {return std::toupper(c);});
    if (std::find(kWheels.begin(), kWheels.end(), w) == kWheels.end()) {
      return;
    }
    for (const auto & name : kWheels) {
      wheel_targets_[name] = 0.0;
    }
    wheel_targets_[w] = rpm;
    last_wheel_cmd_ = get_clock()->now();
    mode_wheels_ = true;
  }

  void tick()
  {
    drain_encoders();
    update_rpm();
    send_latest();
  }

  void drain_encoders()
  {
    char buf[1024];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "Serial read failed: %s", std::strerror(errno));
      }
      return;
    }
    if (n == 0) {
      return;
    }
    rx_buf_.append(buf, static_cast<size_t>(n));

    bool updated = false;
    size_t start = 0;
    while (true) {
      size_t nl = rx_buf_.find('\n', start);
      if (nl == std::string::npos) {
        break;
      }
      std::string line = rx_buf_.substr(start, nl - start);
      start = nl + 1;
      if (line.empty()) {
        continue;
      }
      std::istringstream is(line);
      std::string name;
      if (!(is >> name)) {
        continue;
      }
      if (name == "GR") {
        // Gripper sensor line: "GR <hinge> <anvil> <us>". The Pico
        // drives the 10 Hz rate; publish on receipt.
        int hinge;
        int anvil;
        int64_t us;
        if (!(is >> hinge >> anvil >> us)) {
          continue;
        }
        std::string rest;
        if (is >> rest) {
          continue;
        }
        (void)us;  // Pico-clock timestamp; unused for gripper
        GripperSensors msg;
        msg.hinge = static_cast<uint16_t>(hinge);
        msg.anvil = static_cast<uint16_t>(anvil);
        gripper_pub_->publish(msg);
        continue;
      }
      int value;
      int64_t us;
      if (!(is >> value >> us)) {
        continue;
      }
      std::string rest;
      if (is >> rest) {
        continue;
      }
      if (std::find(kWheels.begin(), kWheels.end(), name) == kWheels.end()) {
        continue;
      }
      counts_[name] = value;
      enc_us_[name] = us;
      enc_updated_.insert(name);
      enc_seen_.insert(name);
      updated = true;
    }
    rx_buf_.erase(0, start);  // keep the partial tail for next tick
    if (updated) {
      last_enc_time_ = get_clock()->now();
      publish_counts();
    }
  }

  void publish_counts()
  {
    EncoderCounts msg;
    msg.fl = counts_["FL"];
    msg.fr = counts_["FR"];
    msg.bl = counts_["BL"];
    msg.br = counts_["BR"];
    encoder_pub_->publish(msg);
  }

  void update_rpm()
  {
    // Per-wheel rpm from Pico-clock deltas (report timestamps), so host
    // timer jitter and burst arrival can't alias into rpm spikes. Wheels
    // without a new report this tick keep their last measurement.
    for (const auto & name : enc_updated_) {
      if (enc_primed_.count(name) == 0) {
        // Prime baseline on first report; enc_seen is marked this same
        // tick, so a seen-check would never prime.
        last_counts_[name] = counts_[name];
        last_enc_us_[name] = enc_us_[name];
        enc_primed_.insert(name);
        continue;
      }
      double dt_us =
        static_cast<double>(enc_us_[name] - last_enc_us_[name]);
      if (dt_us <= 0.0) {
        continue;
      }
      // ponytail: no int32 wrap handling; wraps after ~4M revs.
      double delta =
        static_cast<double>(counts_[name] - last_counts_[name]);
      double raw = delta * kEncDirections.at(name) * 60.0 /
        kTicksPerRev / (dt_us / 1e6);
      raw_rpm_[name] = raw;
      // EMA: raw rpm quantizes ~±11 rpm steps at low speed; filter
      // before the PID sees it.
      measured_rpm_[name] = ema_alpha_[name] * raw +
        (1.0 - ema_alpha_[name]) * measured_rpm_[name];
      last_counts_[name] = counts_[name];
      last_enc_us_[name] = enc_us_[name];
    }
    enc_updated_.clear();
  }

  double feed_forward(const std::string & name, double target)
  {
    // u_ff = kS*sign(v_ref) + kV*v_ref (v_ref normalized).
    double v_ref = target / kWheelMaxRpm;
    if (v_ref == 0.0) {
      return 0.0;
    }
    double sign = v_ref > 0.0 ? 1.0 : -1.0;
    return kS_[name] * sign + kV_[name] * v_ref;
  }

  void send_latest()
  {
    rclcpp::Time now = get_clock()->now();

    // Wheel debug commands win while fresh; fall back to /cmd_vel after
    // 0.5 s of silence so a dead debug node can't hold motion.
    if (mode_wheels_ && (now - last_wheel_cmd_).seconds() > 0.5) {
      mode_wheels_ = false;
    }

    std::map<std::string, double> targets;
    if (mode_wheels_) {
      targets = wheel_targets_;
    } else {
      // Mecanum IK (x forward, y left, CCW positive) -> per-wheel
      // surface speeds, scaled so no wheel exceeds full speed while
      // direction ratios are kept.
      double vx = latest_.linear.x;
      double vy = latest_.linear.y;
      double w = latest_.angular.z * ik_wheelbase_m_;
      if (ik_roller_swap_) {
        vy = -vy;
        w = -w;
      }
      double fl = vx - vy - w;
      double fr = vx + vy + w;
      double bl = vx + vy - w;
      double br = vx - vy + w;
      double m = std::max(
        kVMaxLinearMps,
        std::max({std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br)}));
      double k = kWheelMaxRpm / m;
      targets = {
        {"FL", fl * k}, {"FR", fr * k}, {"BL", bl * k}, {"BR", br * k}};
    }

    double dt = (now - last_pid_time_).seconds();
    last_pid_time_ = now;

    bool enc_stale = (now - last_enc_time_).seconds() > kEncStaleS;
    if (enc_stale) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Encoder data stale, sending neutral PWM");
    }

    std::map<std::string, double> cmds;
    for (const auto & name : kWheels) {
      double out_max = pids_[name].out_max;
      double target = targets[name];
      double v_ref = target / kWheelMaxRpm;
      if (pid_enabled_) {
        // ff + PID, then per-wheel limit.
        double u_ff = ff_enabled_ ? feed_forward(name, target) : 0.0;
        double error = target - measured_rpm_[name];
        double cmd = u_ff + pids_[name].update(error, dt);
        cmd = std::clamp(cmd, -out_max, out_max);
        if (enc_stale) {
          cmd = 0.0;
        }
        cmds[name] = cmd;
      } else {
        // PID off, open loop: ff (or raw v_ref) through the limit.
        double u_ff = ff_enabled_ ? feed_forward(name, target) : v_ref;
        cmds[name] = std::clamp(u_ff, -out_max, out_max);
      }
    }

    publish_wheel_speeds(targets, cmds);

    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.3f\n",
      cmds["FL"] * kWheelDirections.at("FL"),
      cmds["FR"] * kWheelDirections.at("FR"),
      cmds["BL"] * kWheelDirections.at("BL"),
      cmds["BR"] * kWheelDirections.at("BR"));
    ssize_t written = ::write(fd_, buf, std::strlen(buf));
    if (written < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Serial write failed: %s", std::strerror(errno));
    }
  }

  void publish_wheel_speeds(
    const std::map<std::string, double> & targets,
    const std::map<std::string, double> & cmds)
  {
    WheelSpeeds msg;
    msg.fl_target_rpm = static_cast<float>(targets.at("FL"));
    msg.fr_target_rpm = static_cast<float>(targets.at("FR"));
    msg.bl_target_rpm = static_cast<float>(targets.at("BL"));
    msg.br_target_rpm = static_cast<float>(targets.at("BR"));
    msg.fl_measured_rpm_filtered =
      static_cast<float>(measured_rpm_["FL"]);
    msg.fr_measured_rpm_filtered =
      static_cast<float>(measured_rpm_["FR"]);
    msg.bl_measured_rpm_filtered =
      static_cast<float>(measured_rpm_["BL"]);
    msg.br_measured_rpm_filtered =
      static_cast<float>(measured_rpm_["BR"]);
    msg.fl_measured_rpm_raw = static_cast<float>(raw_rpm_["FL"]);
    msg.fr_measured_rpm_raw = static_cast<float>(raw_rpm_["FR"]);
    msg.bl_measured_rpm_raw = static_cast<float>(raw_rpm_["BL"]);
    msg.br_measured_rpm_raw = static_cast<float>(raw_rpm_["BR"]);
    // Normalized PWM -> rpm-equivalent so the bag shows what was
    // actually commanded to the wheel.
    msg.fl_cmd_rpm = static_cast<float>(cmds.at("FL") * kWheelMaxRpm);
    msg.fr_cmd_rpm = static_cast<float>(cmds.at("FR") * kWheelMaxRpm);
    msg.bl_cmd_rpm = static_cast<float>(cmds.at("BL") * kWheelMaxRpm);
    msg.br_cmd_rpm = static_cast<float>(cmds.at("BR") * kWheelMaxRpm);
    wheel_speeds_pub_->publish(msg);
  }

  int fd_ = -1;
  std::string rx_buf_;

  bool pid_enabled_ = true;
  bool ff_enabled_ = true;
  double ik_wheelbase_m_ = 0.35;
  bool ik_roller_swap_ = false;
  bool mode_wheels_ = false;

  std::map<std::string, WheelPID> pids_;
  std::map<std::string, double> kS_;
  std::map<std::string, double> kV_;
  std::map<std::string, double> ema_alpha_;
  std::map<std::string, double> measured_rpm_;
  std::map<std::string, double> raw_rpm_;
  std::map<std::string, int32_t> counts_;
  std::map<std::string, int64_t> enc_us_;
  std::map<std::string, int32_t> last_counts_;
  std::map<std::string, int64_t> last_enc_us_;
  std::map<std::string, double> wheel_targets_;
  std::set<std::string> enc_updated_;
  std::set<std::string> enc_seen_;
  std::set<std::string> enc_primed_;

  Twist latest_;
  rclcpp::Time last_wheel_cmd_;
  rclcpp::Time last_enc_time_;
  rclcpp::Time last_pid_time_;

  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<String>::SharedPtr wheel_cmd_sub_;
  rclcpp::Publisher<EncoderCounts>::SharedPtr encoder_pub_;
  rclcpp::Publisher<WheelSpeeds>::SharedPtr wheel_speeds_pub_;
  rclcpp::Publisher<GripperSensors>::SharedPtr gripper_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CmdVelSerial>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("cmd_vel_serial"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
