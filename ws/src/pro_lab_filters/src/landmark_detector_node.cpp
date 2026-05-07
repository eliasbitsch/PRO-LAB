// Self-defined landmarks for the PRO-LAB assignment.
//
// Landmarks are vertical posts at fixed world (map) positions, loaded from
// config/landmarks.yaml. This node:
//   1) republishes the static landmark map as a visualization MarkerArray
//      (so they show in RViz / Foxglove next to the robot).
//   2) "detects" each landmark by sampling its true range + bearing from
//      the ground-truth pose and adding Gaussian noise — the canonical
//      simulated landmark sensor used in textbook localisation chapters.
//      Publishes Float32MultiArray on /landmarks/observations:
//          [id_0, range_0, bearing_0, id_1, range_1, bearing_1, ...]
//
// Filters that want to use this signal can subscribe and compute
// h(x) = (sqrt((lx-x)² + (ly-y)²), atan2(ly-y, lx-x) - yaw) and run a
// standard EKF/PF measurement update against the [range, bearing] vector.
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pro_lab_filters/common.hpp"

class LandmarkDetector : public rclcpp::Node {
public:
  LandmarkDetector() : Node("landmark_detector") {
    // Landmarks declared as parallel arrays — ROS params don't natively
    // support arrays of structs, so we flatten ids / xs / ys.
    declare_parameter<std::vector<int64_t>>("landmark_ids", {1, 2, 3});
    declare_parameter<std::vector<double>>("landmark_xs",  {-5.0,  3.0, -2.0});
    declare_parameter<std::vector<double>>("landmark_ys",  { 2.0, -2.0, -6.0});
    declare_parameter("range_sigma",   0.10);
    declare_parameter("bearing_sigma", 0.05);
    declare_parameter("max_range",     6.0);
    declare_parameter("fov_rad",       3.14159);
    declare_parameter("frame_id",      std::string("map"));
    declare_parameter("rate_hz",       5.0);

    const auto ids = get_parameter("landmark_ids").as_integer_array();
    const auto xs  = get_parameter("landmark_xs").as_double_array();
    const auto ys  = get_parameter("landmark_ys").as_double_array();
    if (ids.size() != xs.size() || ids.size() != ys.size()) {
      RCLCPP_ERROR(get_logger(),
        "landmark_ids/xs/ys length mismatch (%zu/%zu/%zu)",
        ids.size(), xs.size(), ys.size());
    } else {
      lms_.reserve(ids.size());
      for (std::size_t i = 0; i < ids.size(); ++i) {
        lms_.push_back({static_cast<int>(ids[i]), xs[i], ys[i]});
      }
    }

    range_sigma_   = get_parameter("range_sigma").as_double();
    bearing_sigma_ = get_parameter("bearing_sigma").as_double();
    max_range_     = get_parameter("max_range").as_double();
    fov_           = get_parameter("fov_rad").as_double();
    frame_id_      = get_parameter("frame_id").as_string();
    const double hz = get_parameter("rate_hz").as_double();

    // Use TF directly so we always have the robot's pose *in the map
    // frame* — matches the landmark coordinates declared above and
    // sidesteps the odom/map frame mismatch you'd get by reading
    // /ground_truth/pose (which is published in the odom frame).
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    base_frame_  = declare_parameter<std::string>("base_frame", "base_footprint");

    obs_pub_   = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/landmarks/observations", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/landmarks/markers", rclcpp::QoS(1).transient_local());

    publishMarkers();   // latched, sent once
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
        [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
      "landmark_detector: %zu landmarks, range_sigma=%.2fm, bearing_sigma=%.2frad",
      lms_.size(), range_sigma_, bearing_sigma_);
  }

private:
  struct Landmark { int id; double x; double y; };

  void publishMarkers() {
    visualization_msgs::msg::MarkerArray arr;
    for (const auto & lm : lms_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp    = now();
      m.ns              = "landmarks";
      m.id              = lm.id;
      m.type            = visualization_msgs::msg::Marker::CYLINDER;
      m.action          = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = lm.x;
      m.pose.position.y = lm.y;
      m.pose.position.z = 0.5;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.15; m.scale.y = 0.15; m.scale.z = 1.0;
      m.color.r = 1.0;  m.color.g = 0.7;  m.color.b = 0.0; m.color.a = 0.9;
      arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
  }

  void tick() {
    // Look up robot pose in map frame from TF.
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(frame_id_, base_frame_,
                                       tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "landmark_detector: waiting for TF %s -> %s: %s",
        frame_id_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }
    const double truth_x = tf.transform.translation.x;
    const double truth_y = tf.transform.translation.y;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    double r, p, truth_yaw;
    tf2::Matrix3x3(q).getRPY(r, p, truth_yaw);

    std_msgs::msg::Float32MultiArray msg;
    for (const auto & lm : lms_) {
      const double dx = lm.x - truth_x;
      const double dy = lm.y - truth_y;
      const double range = std::hypot(dx, dy);
      if (range > max_range_) continue;
      double bearing = std::atan2(dy, dx) - truth_yaw;
      // wrap to [-pi, pi]
      while (bearing >  M_PI) bearing -= 2 * M_PI;
      while (bearing < -M_PI) bearing += 2 * M_PI;
      if (std::abs(bearing) > fov_ / 2.0) continue;
      // Add gaussian noise to simulate a real detector
      std::normal_distribution<double> nr(0.0, range_sigma_);
      std::normal_distribution<double> nb(0.0, bearing_sigma_);
      msg.data.push_back(static_cast<float>(lm.id));
      msg.data.push_back(static_cast<float>(range  + nr(rng_)));
      msg.data.push_back(static_cast<float>(bearing + nb(rng_)));
    }
    if (!msg.data.empty()) {
      obs_pub_->publish(msg);
    }
  }

  std::vector<Landmark> lms_;
  double range_sigma_   {0.1};
  double bearing_sigma_ {0.05};
  double max_range_     {6.0};
  double fov_           {6.28};
  std::string frame_id_ {"map"};

  std::shared_ptr<tf2_ros::Buffer>              tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;
  std::string base_frame_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr               obs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr           marker_pub_;
  rclcpp::TimerBase::SharedPtr                                                 timer_;
  std::mt19937 rng_ {std::random_device{}()};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkDetector>());
  rclcpp::shutdown();
  return 0;
}
