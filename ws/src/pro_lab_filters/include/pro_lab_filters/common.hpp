#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <array>

namespace pro_lab_filters {

inline double wrap_angle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

inline geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

inline double quat_to_yaw(const geometry_msgs::msg::Quaternion& q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// Convenience variant: equal x/y variance, no cross-correlation → RViz draws
// a circle. Kept for nodes that don't expose a full 2D covariance.
inline geometry_msgs::msg::PoseWithCovarianceStamped
make_pose(const rclcpp::Time& stamp, const std::string& frame_id,
          double x, double y, double yaw, double cov_xy, double cov_yaw) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = yaw_to_quat(yaw);
  std::array<double, 36> cov{};
  cov[0] = cov_xy;       // x-x
  cov[7] = cov_xy;       // y-y
  cov[35] = cov_yaw;     // yaw-yaw
  for (size_t i = 0; i < 36; ++i) msg.pose.covariance[i] = cov[i];
  return msg;
}

// Full-2D variant: pass the actual P(0,0), P(1,1), P(0,1) so RViz renders an
// ellipse aligned with the filter's real principal axes (instead of a circle).
inline geometry_msgs::msg::PoseWithCovarianceStamped
make_pose_xy(const rclcpp::Time& stamp, const std::string& frame_id,
             double x, double y, double yaw,
             double pxx, double pyy, double pxy, double cov_yaw) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = yaw_to_quat(yaw);
  for (size_t i = 0; i < 36; ++i) msg.pose.covariance[i] = 0.0;
  msg.pose.covariance[0]  = pxx;   // x-x
  msg.pose.covariance[1]  = pxy;   // x-y
  msg.pose.covariance[6]  = pxy;   // y-x
  msg.pose.covariance[7]  = pyy;   // y-y
  msg.pose.covariance[35] = cov_yaw;
  return msg;
}

}  // namespace pro_lab_filters
