#pragma once
// Extended Kalman Filter (pure, no ROS).
// Motion: unicycle. Measurements: IMU yaw, full pose.
#include <Eigen/Dense>
#include <cmath>

namespace pro_lab_filters {

inline double ekf_wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

class EKF {
public:
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;

  EKF() = default;

  void init(const Vec3& x0, const Mat3& P0, const Mat3& Q) {
    x_ = x0; P_ = P0; Q_ = Q;
  }

  void predict(double v, double w, double dt) {
    const double theta = x_(2);
    x_(0) += v * std::cos(theta) * dt;
    x_(1) += v * std::sin(theta) * dt;
    x_(2) = ekf_wrap_pi(theta + w * dt);
    Mat3 F;
    F << 1.0, 0.0, -v * std::sin(theta) * dt,
         0.0, 1.0,  v * std::cos(theta) * dt,
         0.0, 0.0,  1.0;
    P_ = F * P_ * F.transpose() + Q_ * dt;
  }

  void updateImuYaw(double yaw_meas, double r_yaw) {
    Eigen::Matrix<double, 1, 3> H;
    H << 0.0, 0.0, 1.0;
    const double innov = ekf_wrap_pi(yaw_meas - x_(2));
    const double S = (H * P_ * H.transpose())(0, 0) + r_yaw;
    Vec3 K = P_ * H.transpose() / S;
    x_ += K * innov;
    x_(2) = ekf_wrap_pi(x_(2));
    P_ = (Mat3::Identity() - K * H) * P_;
  }

  // Range-bearing landmark update (Probabilistic Robotics §7.5).
  //   z = (range, bearing) measured from robot to a known landmark at (lx, ly).
  // Linearises h(x) around the current mean to build the Jacobian H, then
  // does the standard EKF correction. Bearing residual is wrapped to (-π, π].
  void updateLandmark(double lx, double ly,
                      double meas_range, double meas_bearing,
                      double r_range, double r_bearing) {
    const double dx = lx - x_(0);
    const double dy = ly - x_(1);
    const double q  = dx * dx + dy * dy;
    if (q < 1e-9) {
      return;
    }
    const double sq = std::sqrt(q);
    const double pred_range   = sq;
    const double pred_bearing = ekf_wrap_pi(std::atan2(dy, dx) - x_(2));

    Eigen::Matrix<double, 2, 3> H;
    H << -dx / sq,        -dy / sq,         0.0,
          dy / q,         -dx / q,         -1.0;

    Eigen::Matrix<double, 2, 2> R;
    R << r_range, 0.0, 0.0, r_bearing;

    Eigen::Vector2d innov(meas_range - pred_range,
                          ekf_wrap_pi(meas_bearing - pred_bearing));

    Eigen::Matrix<double, 2, 2> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();
    x_ += K * innov;
    x_(2) = ekf_wrap_pi(x_(2));
    P_ = (Mat3::Identity() - K * H) * P_;
  }

  void updatePose(double mx, double my, double myaw, const Vec3& r_diag) {
    Mat3 H = Mat3::Identity();
    Mat3 R = r_diag.asDiagonal();
    Vec3 z(mx, my, myaw);
    Vec3 innov = z - x_;
    innov(2) = ekf_wrap_pi(innov(2));
    Mat3 S = H * P_ * H.transpose() + R;
    Mat3 K = P_ * H.transpose() * S.inverse();
    x_ += K * innov;
    x_(2) = ekf_wrap_pi(x_(2));
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
