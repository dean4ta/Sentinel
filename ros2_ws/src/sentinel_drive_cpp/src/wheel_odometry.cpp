// C++ port of sentinel_drive/wheel_odometry.py (+ odometry_math.py).
// /wheel_speeds + /wheel_consistency -> ridge-regularised weighted LS
// twist -> integrated odom-frame pose -> nav_msgs/Odometry on
// /wheel_odom. No TF here; the fusion node owns odom->base.
#include <array>
#include <cmath>
#include <optional>

#include <Eigen/Dense>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentinel_msgs/msg/wheel_consistency.hpp"
#include "sentinel_msgs/msg/wheel_speeds.hpp"

using nav_msgs::msg::Odometry;
using sentinel_msgs::msg::WheelConsistency;
using sentinel_msgs::msg::WheelSpeeds;

namespace {

constexpr double kBigVar = 1e6;  // unobserved axes (z, roll, pitch)

// Rows follow wheel order FL FR BL BR; columns are (vx, vy, wL).
const Eigen::Matrix<double, 4, 3> kA = (Eigen::Matrix<double, 4, 3>() <<
  1.0, -1.0, -1.0,
  1.0, 1.0, 1.0,
  1.0, 1.0, -1.0,
  1.0, -1.0, 1.0).finished();

}  // namespace

class WheelOdometry : public rclcpp::Node
{
public:
  WheelOdometry()
  : Node("wheel_odometry")
  {
    double circ = declare_parameter("wheel_circumference_m", 0.44);
    k_ = circ / 60.0;
    // half-track + half-wheelbase; must match cmd_vel_serial's
    // ik_wheelbase_m or commanded and measured yaw rates disagree.
    lxy_ = declare_parameter("lxy_m", 0.362);
    weight_floor_ = declare_parameter("weight_floor", 0.05);
    ridge_ = declare_parameter("ridge", 0.05);
    noise_ = declare_parameter("wheel_noise_mps", 0.02);
    degraded_eig_ = declare_parameter("degraded_min_eig", 0.5);
    conf_timeout_ = declare_parameter("confidence_timeout_s", 0.2);
    max_dt_ = declare_parameter("max_dt_s", 0.2);
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");

    conf_sub_ = create_subscription<WheelConsistency>(
      "/wheel_consistency", 10,
      [this](const WheelConsistency::SharedPtr m) {
        conf_ = {m->fl_confidence, m->fr_confidence,
          m->bl_confidence, m->br_confidence};
        conf_rx_ = get_clock()->now();
      });
    speeds_sub_ = create_subscription<WheelSpeeds>(
      "/wheel_speeds", 10,
      [this](const WheelSpeeds::SharedPtr m) {speeds_cb(m);});
    pub_ = create_publisher<Odometry>("/wheel_odom", 10);

    RCLCPP_INFO(get_logger(),
      "Wheel odometry: circ=%f m, lxy=%f m, floor=%f, ridge=%f",
      circ, lxy_, weight_floor_, ridge_);
  }

private:
  // Confidences, or equal weights if the detector is silent.
  Eigen::Vector4d weights()
  {
    Eigen::Vector4d w = Eigen::Vector4d::Ones();
    if (!conf_ || !conf_rx_) {
      return w;
    }
    double age = (get_clock()->now() - *conf_rx_).seconds();
    if (age > conf_timeout_) {
      return w;
    }
    for (int i = 0; i < 4; i++) {
      w[i] = std::clamp(static_cast<double>((*conf_)[i]),
        weight_floor_, 1.0);
    }
    return w;
  }

  void speeds_cb(const WheelSpeeds::SharedPtr m)
  {
    rclcpp::Time now = get_clock()->now();  // WheelSpeeds has no header

    Eigen::Vector4d v;
    v << m->fl_measured_rpm_filtered * k_,
      m->fr_measured_rpm_filtered * k_,
      m->bl_measured_rpm_filtered * k_,
      m->br_measured_rpm_filtered * k_;
    Eigen::Vector4d w = weights();

    Eigen::Vector3d x0(prior_(0), prior_(1), prior_(2));
    Eigen::Matrix3d h_data = kA.transpose() * w.asDiagonal() * kA;
    Eigen::Matrix3d h = h_data + ridge_ * Eigen::Matrix3d::Identity();
    Eigen::Vector3d x = h.ldlt().solve(
      kA.transpose() * (w.asDiagonal() * v) + ridge_ * x0);
    Eigen::Matrix3d cov = (noise_ * noise_) * h.inverse();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(h_data);
    double min_eig = es.eigenvalues()[0];
    bool degraded = min_eig < degraded_eig_;

    prior_ = x;
    double omega = x(2) / lxy_;

    double dt = prev_t_ ? (now - *prev_t_).seconds() : 0.0;
    prev_t_ = now;
    if (dt > 0.0 && dt <= max_dt_) {
      // Midpoint integration of the body-frame twist.
      double yaw_mid = pose_yaw_ + 0.5 * omega * dt;
      double c = std::cos(yaw_mid);
      double s = std::sin(yaw_mid);
      pose_x_ += (x(0) * c - x(1) * s) * dt;
      pose_y_ += (x(0) * s + x(1) * c) * dt;
      pose_yaw_ += omega * dt;
      pvar_xy_ += (cov(0, 0) + cov(1, 1)) * 0.5 * dt * dt;
      pvar_yaw_ += cov(2, 2) / (lxy_ * lxy_) * dt * dt;
    }

    if (degraded != was_degraded_) {
      was_degraded_ = degraded;
      if (degraded) {
        RCLCPP_WARN(get_logger(), "Wheel odometry DEGRADED (min_eig=%.2f)",
          min_eig);
      } else {
        RCLCPP_INFO(get_logger(), "Wheel odometry healthy (min_eig=%.2f)",
          min_eig);
      }
    }

    publish(now, x, omega, cov);
  }

  void publish(const rclcpp::Time & stamp, const Eigen::Vector3d & x,
               double omega, const Eigen::Matrix3d & cov)
  {
    Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;

    out.pose.pose.position.x = pose_x_;
    out.pose.pose.position.y = pose_y_;
    out.pose.pose.orientation.z = std::sin(0.5 * pose_yaw_);
    out.pose.pose.orientation.w = std::cos(0.5 * pose_yaw_);

    out.twist.twist.linear.x = x(0);
    out.twist.twist.linear.y = x(1);
    out.twist.twist.angular.z = omega;

    out.pose.covariance[0] = pvar_xy_;
    out.pose.covariance[7] = pvar_xy_;
    out.pose.covariance[35] = pvar_yaw_;
    out.pose.covariance[14] = out.pose.covariance[21] =
      out.pose.covariance[28] = kBigVar;

    out.twist.covariance[0] = cov(0, 0);
    out.twist.covariance[7] = cov(1, 1);
    out.twist.covariance[35] = cov(2, 2) / (lxy_ * lxy_);
    out.twist.covariance[14] = out.twist.covariance[21] =
      out.twist.covariance[28] = kBigVar;

    pub_->publish(out);
  }

  double k_ = 0.44 / 60.0;
  double lxy_ = 0.362;
  double weight_floor_ = 0.05;
  double ridge_ = 0.05;
  double noise_ = 0.02;
  double degraded_eig_ = 0.5;
  double conf_timeout_ = 0.2;
  double max_dt_ = 0.2;
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";

  std::optional<std::array<float, 4>> conf_;
  std::optional<rclcpp::Time> conf_rx_;
  std::optional<rclcpp::Time> prev_t_;
  Eigen::Vector3d prior_ = Eigen::Vector3d::Zero();
  double pose_x_ = 0.0;
  double pose_y_ = 0.0;
  double pose_yaw_ = 0.0;
  double pvar_xy_ = 0.0;
  double pvar_yaw_ = 0.0;
  bool was_degraded_ = false;

  rclcpp::Subscription<WheelConsistency>::SharedPtr conf_sub_;
  rclcpp::Subscription<WheelSpeeds>::SharedPtr speeds_sub_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometry>());
  rclcpp::shutdown();
  return 0;
}
