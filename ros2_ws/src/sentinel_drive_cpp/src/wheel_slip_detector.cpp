// C++ port of sentinel_drive/wheel_slip_detector.py (+ wheel_consistency.py
// math). Same params, same topics: /wheel_speeds -> /wheel_consistency.
// Leave-one-out mecanum check; asymmetric: overspeed is suspicious, a
// slower wheel stays trusted.
#include <algorithm>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sentinel_msgs/msg/wheel_consistency.hpp"
#include "sentinel_msgs/msg/wheel_speeds.hpp"

using sentinel_msgs::msg::WheelConsistency;
using sentinel_msgs::msg::WheelSpeeds;

namespace {

// Wheel order: FL FR BL BR (matches WHEELS in the Python original).
// v_i = vx + vy_sign_i * vy + w_sign_i * wL
const std::array<int, 4> kVySign = {-1, 1, 1, -1};
const std::array<int, 4> kWSign = {-1, 1, -1, 1};

struct ConfidenceFilter {
  double alpha_suspicious = 0.2;
  double alpha_recover = 0.02;
  double confidence = 1.0;

  double update(double suspicion)
  {
    double target = 1.0 - std::clamp(suspicion, 0.0, 1.0);
    double a = target < confidence ? alpha_suspicious : alpha_recover;
    confidence += a * (target - confidence);
    return confidence;
  }
};

// Forward kinematics from the three wheels excluding index `ex`.
void fk_three(const std::array<double, 4> & v, int ex,
              double & vx, double & vy, double & wL)
{
  double fl = v[0];
  double fr = v[1];
  double bl = v[2];
  double br = v[3];
  if (ex == 0) {        // exclude FL: use FR, BL, BR
    vy = (fr - br) / 2.0;
    wL = (fr - bl) / 2.0;
    vx = fr - vy - wL;
  } else if (ex == 1) { // exclude FR: use FL, BL, BR
    vy = (bl - fl) / 2.0;
    wL = (br - fl) / 2.0;
    vx = fl + vy + wL;
  } else if (ex == 2) { // exclude BL: use FL, FR, BR
    vy = (fr - br) / 2.0;
    wL = (br - fl) / 2.0;
    vx = br + vy - wL;
  } else {              // exclude BR: use FL, FR, BL
    vy = (bl - fl) / 2.0;
    wL = (fr - bl) / 2.0;
    vx = bl - vy + wL;
  }
}

double median3(double a, double b, double c)
{
  return std::max(std::min(a, b), std::min(std::max(a, b), c));
}

// Asymmetric slip suspicion in [0, 1], sign-aware. See the Python
// wheel_consistency.py docstring for the full gate logic.
double suspicion(double residual, double measured, double target,
                 const double peer_errors[3],
                 const double peer_measured[3],
                 const double peer_targets[3],
                 double threshold, double scale, double tracking_scale,
                 double margin)
{
  double excess = std::abs(measured) - std::abs(target);
  double peer_excess = -1e9;
  for (int i = 0; i < 3; i++) {
    peer_excess = std::max(
      peer_excess,
      std::abs(peer_measured[i]) - std::abs(peer_targets[i]));
  }
  if (excess - peer_excess <= margin) {
    return 0.0;
  }
  double pred = measured - residual;
  double s;
  if (pred > 0.0) {
    s = 1.0;
  } else if (pred < 0.0) {
    s = -1.0;
  } else {
    s = target >= 0.0 ? 1.0 : -1.0;
  }
  double over = residual * s - threshold;
  if (over <= 0.0) {
    return 0.0;
  }
  over = std::min(1.0, over / scale);
  double err = std::abs(target - measured);
  double peer_med = median3(peer_errors[0], peer_errors[1], peer_errors[2]);
  double adv = std::clamp((peer_med - err) / tracking_scale, 0.0, 1.0);
  return over * (0.5 + 0.5 * adv);
}

}  // namespace

class WheelSlipDetector : public rclcpp::Node
{
public:
  WheelSlipDetector()
  : Node("wheel_slip_detector")
  {
    double circ = declare_parameter("wheel_circumference_m", 0.44);
    rpm_to_mps_ = circ / 60.0;
    threshold_ = declare_parameter("residual_threshold_mps", 0.05);
    scale_ = declare_parameter("residual_scale_mps", 0.15);
    tracking_scale_ = declare_parameter("tracking_scale_mps", 0.30);
    margin_ = declare_parameter("peer_margin_mps", 0.03);
    double a_susp = declare_parameter("alpha_suspicious", 0.20);
    double a_rec = declare_parameter("alpha_recover", 0.02);
    for (auto & f : filters_) {
      f.alpha_suspicious = a_susp;
      f.alpha_recover = a_rec;
    }

    sub_ = create_subscription<WheelSpeeds>(
      "/wheel_speeds", 10,
      [this](const WheelSpeeds::SharedPtr m) {cb(m);});
    pub_ = create_publisher<WheelConsistency>("/wheel_consistency", 10);

    RCLCPP_INFO(get_logger(),
      "Slip detector: wheel_circumference=%f m, threshold=%f m/s, "
      "scale=%f, tracking_scale=%f, margin=%f, alpha=%f/%f",
      circ, threshold_, scale_, tracking_scale_, margin_, a_susp, a_rec);
  }

private:
  void cb(const WheelSpeeds::SharedPtr m)
  {
    std::array<double, 4> measured = {
      m->fl_measured_rpm_filtered * rpm_to_mps_,
      m->fr_measured_rpm_filtered * rpm_to_mps_,
      m->bl_measured_rpm_filtered * rpm_to_mps_,
      m->br_measured_rpm_filtered * rpm_to_mps_};
    std::array<double, 4> targets = {
      m->fl_target_rpm * rpm_to_mps_,
      m->fr_target_rpm * rpm_to_mps_,
      m->bl_target_rpm * rpm_to_mps_,
      m->br_target_rpm * rpm_to_mps_};

    WheelConsistency out;
    out.header.stamp = get_clock()->now();

    double confidences[4] = {0.0, 0.0, 0.0, 0.0};
    double residuals[4] = {0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 4; i++) {
      double vx, vy, wL;
      fk_three(measured, i, vx, vy, wL);
      double predicted = vx + kVySign[i] * vy + kWSign[i] * wL;
      double residual = measured[i] - predicted;
      residuals[i] = residual;

      double peer_errors[3];
      double peer_measured[3];
      double peer_targets[3];
      int n = 0;
      for (int j = 0; j < 4; j++) {
        if (j == i) {
          continue;
        }
        peer_errors[n] = std::abs(targets[j] - measured[j]);
        peer_measured[n] = measured[j];
        peer_targets[n] = targets[j];
        n++;
      }
      double s = suspicion(residual, measured[i], targets[i],
        peer_errors, peer_measured, peer_targets,
        threshold_, scale_, tracking_scale_, margin_);
      confidences[i] = filters_[i].update(s);
    }

    out.fl_confidence = static_cast<float>(confidences[0]);
    out.fr_confidence = static_cast<float>(confidences[1]);
    out.bl_confidence = static_cast<float>(confidences[2]);
    out.br_confidence = static_cast<float>(confidences[3]);
    out.fl_residual = static_cast<float>(residuals[0]);
    out.fr_residual = static_cast<float>(residuals[1]);
    out.bl_residual = static_cast<float>(residuals[2]);
    out.br_residual = static_cast<float>(residuals[3]);
    pub_->publish(out);
  }

  double rpm_to_mps_ = 0.44 / 60.0;
  double threshold_ = 0.05;
  double scale_ = 0.15;
  double tracking_scale_ = 0.30;
  double margin_ = 0.03;
  std::array<ConfidenceFilter, 4> filters_;

  rclcpp::Subscription<WheelSpeeds>::SharedPtr sub_;
  rclcpp::Publisher<WheelConsistency>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelSlipDetector>());
  rclcpp::shutdown();
  return 0;
}
