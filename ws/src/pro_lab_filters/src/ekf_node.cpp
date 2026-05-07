// ROS2 wrapper for EKF. Inputs: /cmd_vel, /imu, /pose, /landmarks/observations.
// Outputs: /ekf/pose, /ekf/runtime_us.
//
// Landmarks (range/bearing) come in as Float32MultiArray with stride 3:
//   [id_0, range_0, bearing_0, id_1, range_1, bearing_1, ...]
// where id matches the landmark_*s arrays declared on this node. Every
// observation triggers one EKF correction step (see EKF::updateLandmark).
#include <chrono>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include "pro_lab_filters/EKF.h"
#include "pro_lab_filters/common.hpp"

using namespace std::chrono_literals;

class EKFNode : public rclcpp::Node {
public:
  EKFNode() : Node("ekf_node") {
    declare_parameter("init_x", 0.0);
    declare_parameter("init_y", 0.0);
    declare_parameter("init_yaw", 0.0);
    declare_parameter("init_cov", 0.1);
    declare_parameter("q_scale", 0.05);
    declare_parameter("r_yaw_imu", 0.02);
    declare_parameter("r_pose_xy", 0.05);
    declare_parameter("r_pose_yaw", 0.05);
    declare_parameter("frame_id", std::string("odom"));
    // Landmark fusion: same xs/ys/ids the landmark_detector_node uses.
    // r_landmark_range / r_landmark_bearing are measurement variances (= σ²).
    declare_parameter<std::vector<int64_t>>("landmark_ids", {1, 2, 3});
    declare_parameter<std::vector<double>>("landmark_xs",  {-5.0,  3.0, -2.0});
    declare_parameter<std::vector<double>>("landmark_ys",  { 2.0, -2.0, -6.0});
    declare_parameter("r_landmark_range",   0.01);   // σ=0.10 m
    declare_parameter("r_landmark_bearing", 0.0025); // σ≈0.05 rad

    Eigen::Vector3d x0(
      get_parameter("init_x").as_double(),
      get_parameter("init_y").as_double(),
      get_parameter("init_yaw").as_double());
    Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity() * get_parameter("init_cov").as_double();
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * get_parameter("q_scale").as_double();
    ekf_.init(x0, P0, Q);

    r_yaw_imu_ = get_parameter("r_yaw_imu").as_double();
    r_pose_diag_ << get_parameter("r_pose_xy").as_double(),
                    get_parameter("r_pose_xy").as_double(),
                    get_parameter("r_pose_yaw").as_double();
    frame_id_ = get_parameter("frame_id").as_string();

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr m) {
        v_ = m->linear.x; w_ = m->angular.z;
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 20,
      [this](sensor_msgs::msg::Imu::SharedPtr m) {
        ekf_.updateImuYaw(pro_lab_filters::quat_to_yaw(m->orientation), r_yaw_imu_);
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        double yaw = pro_lab_filters::quat_to_yaw(m->pose.pose.orientation);
        ekf_.updatePose(m->pose.pose.position.x, m->pose.pose.position.y, yaw, r_pose_diag_);
      });

    // Build the {id → (lx, ly)} lookup for landmark updates.
    {
      const auto ids = get_parameter("landmark_ids").as_integer_array();
      const auto xs  = get_parameter("landmark_xs").as_double_array();
      const auto ys  = get_parameter("landmark_ys").as_double_array();
      const std::size_t n = std::min({ids.size(), xs.size(), ys.size()});
      for (std::size_t i = 0; i < n; ++i) {
        landmarks_[static_cast<int>(ids[i])] = {xs[i], ys[i]};
      }
    }
    r_lm_range_   = get_parameter("r_landmark_range").as_double();
    r_lm_bearing_ = get_parameter("r_landmark_bearing").as_double();

    landmarks_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/landmarks/observations", 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr m) {
        // Stride-3 (id, range, bearing) flat array.
        for (std::size_t i = 0; i + 2 < m->data.size(); i += 3) {
          const int    id      = static_cast<int>(m->data[i]);
          const double range   = m->data[i + 1];
          const double bearing = m->data[i + 2];
          auto it = landmarks_.find(id);
          if (it == landmarks_.end()) continue;
          ekf_.updateLandmark(it->second.first, it->second.second,
                              range, bearing,
                              r_lm_range_, r_lm_bearing_);
        }
      });

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf/pose", 10);
    runtime_pub_ = create_publisher<std_msgs::msg::Float64>("/ekf/runtime_us", 10);
    timer_ = create_wall_timer(50ms, [this]() { tick(); });
  }

private:
  void tick() {
    auto now = this->now();
    double t = now.seconds();
    if (!have_last_) { last_t_ = t; have_last_ = true; return; }
    double dt = t - last_t_;
    last_t_ = t;
    const auto t_start = std::chrono::steady_clock::now();
    if (dt > 0) ekf_.predict(v_, w_, dt);
    const auto& x = ekf_.state();
    const auto& P = ekf_.covariance();
    pub_->publish(pro_lab_filters::make_pose(now, frame_id_, x(0), x(1), x(2), P(0, 0), P(2, 2)));
    const auto t_end = std::chrono::steady_clock::now();
    std_msgs::msg::Float64 rt;
    rt.data = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    runtime_pub_->publish(rt);
  }

  pro_lab_filters::EKF ekf_;
  double v_ = 0.0, w_ = 0.0;
  double last_t_ = 0.0;
  bool have_last_ = false;
  double r_yaw_imu_ = 0.02;
  Eigen::Vector3d r_pose_diag_{0.05, 0.05, 0.05};
  std::string frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr landmarks_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr runtime_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<int, std::pair<double, double>> landmarks_;
  double r_lm_range_   {0.01};
  double r_lm_bearing_ {0.0025};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}
