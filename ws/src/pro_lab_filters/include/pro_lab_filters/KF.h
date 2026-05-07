#pragma once
// Linear Kalman Filter (pure, no ROS).
// State: [x, y, yaw]. Control: [v, omega]. Measurements: IMU yaw, full pose.
#include <Eigen/Dense>
#include <cmath>

namespace pro_lab_filters {

inline double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

class KF {
public:
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;

  KF() = default;

  void init(const Vec3& x0, const Mat3& P0, const Mat3& Q) {
    x_ = x0; P_ = P0; Q_ = Q;
  }

  // Linear prediction around current yaw (treated constant over dt).
  void predict(double v, double w, double dt) {
    const double theta = x_(2);
    Mat3 F = Mat3::Identity();
    Eigen::Matrix<double, 3, 2> B;
    B << std::cos(theta) * dt, 0.0,
         std::sin(theta) * dt, 0.0,
         0.0,                  dt;
    Eigen::Vector2d u(v, w);
    x_ = F * x_ + B * u;
    x_(2) = wrap_pi(x_(2));
    P_ = F * P_ * F.transpose() + Q_ * dt;
  }

  // IMU yaw measurement.
  void updateImuYaw(double yaw_meas, double r_yaw) {
    Eigen::Matrix<double, 1, 3> H;
    H << 0.0, 0.0, 1.0;
    const double innov = wrap_pi(yaw_meas - x_(2));
    const double S = (H * P_ * H.transpose())(0, 0) + r_yaw;
    Vec3 K = P_ * H.transpose() / S;
    x_ += K * innov;
    x_(2) = wrap_pi(x_(2));
    P_ = (Mat3::Identity() - K * H) * P_;
  }

  // Full pose measurement [x, y, yaw] with diag covariance.
  void updatePose(double mx, double my, double myaw, const Vec3& r_diag) {
    Mat3 H = Mat3::Identity();
    Mat3 R = r_diag.asDiagonal();
    Vec3 z(mx, my, myaw);
    Vec3 innov = z - x_;
    innov(2) = wrap_pi(innov(2));
    Mat3 S = H * P_ * H.transpose() + R;
    Mat3 K = P_ * H.transpose() * S.inverse();
    x_ += K * innov;
    x_(2) = wrap_pi(x_(2));
    P_ = (Mat3::Identity() - K * H) * P_;
  }

  const Vec3& state() const { return x_; }
  const Mat3& covariance() const { return P_; }

private:
  Vec3 x_ = Vec3::Zero();
  Mat3 P_ = Mat3::Identity();
  Mat3 Q_ = Mat3::Identity() * 0.05;
};

}  // namespace pro_lab_filters
