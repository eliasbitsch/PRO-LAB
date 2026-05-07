// ROS2 wrapper for KF. Inputs: /cmd_vel (Twist), /imu (Imu), /pose (PoseWithCovarianceStamped).
// Output: /kf/pose (PoseWithCovarianceStamped).
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include "pro_lab_filters/KF.h"
#include "pro_lab_filters/common.hpp"

using namespace std::chrono_literals;

class KFNode : public rclcpp::Node {
public:
  KFNode() : Node("kf_node") {
    declare_parameter("init_x", 0.0);
    declare_parameter("init_y", 0.0);
    declare_parameter("init_yaw", 0.0);
    declare_parameter("init_cov", 0.1);
    declare_parameter("q_scale", 0.05);
    declare_parameter("r_yaw_imu", 0.02);
    declare_parameter("r_pose_xy", 0.05);
    declare_parameter("r_pose_yaw", 0.05);
    declare_parameter("frame_id", std::string("odom"));

    Eigen::Vector3d x0(
      get_parameter("init_x").as_double(),
      get_parameter("init_y").as_double(),
      get_parameter("init_yaw").as_double());
    Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity() * get_parameter("init_cov").as_double();
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * get_parameter("q_scale").as_double();
    kf_.init(x0, P0, Q);

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
        kf_.updateImuYaw(pro_lab_filters::quat_to_yaw(m->orientation), r_yaw_imu_);
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        double yaw = pro_lab_filters::quat_to_yaw(m->pose.pose.orientation);
        kf_.updatePose(m->pose.pose.position.x, m->pose.pose.position.y, yaw, r_pose_diag_);
      });

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf/pose", 10);
    runtime_pub_ = create_publisher<std_msgs::msg::Float64>("/kf/runtime_us", 10);
    timer_ = create_wall_timer(50ms, [this]() { tick(); });
  }

private:
  void tick() {
    auto now = this->now();
    double t = now.seconds();
    if (!have_last_) { last_t_ = t; have_last_ = true; return; }
    double dt = t - last_t_;
    last_t_ = t;
    // Wall-clock timing of the predict + publish step. Used in the
    // assignment's "Runtime / Performance" comparison vs EKF and PF.
    const auto t_start = std::chrono::steady_clock::now();
    if (dt > 0) kf_.predict(v_, w_, dt);
    const auto& x = kf_.state();
    const auto& P = kf_.covariance();
    pub_->publish(pro_lab_filters::make_pose(now, frame_id_, x(0), x(1), x(2), P(0, 0), P(2, 2)));
    const auto t_end = std::chrono::steady_clock::now();
    std_msgs::msg::Float64 rt;
    rt.data = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    runtime_pub_->publish(rt);
  }

  pro_lab_filters::KF kf_;
  double v_ = 0.0, w_ = 0.0;
  double last_t_ = 0.0;
  bool have_last_ = false;
  double r_yaw_imu_ = 0.02;
  Eigen::Vector3d r_pose_diag_{0.05, 0.05, 0.05};
  std::string frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr runtime_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KFNode>());
  rclcpp::shutdown();
  return 0;
}
