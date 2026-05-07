// Resolves the robot's true world pose by looking up TF (target_frame ←
// source_frame) and republishes it as PoseStamped on /ground_truth/pose
// at a configurable rate. Used by the metrics_node as the reference signal.
//
// In the nav2_minimal_tb4_sim setup, the chain is:
//   <world> -> odom -> base_link
// Default lookup is "odom" -> "base_footprint" (works for any tb4 spawn).
//
// Parameters:
//   target_frame  default "odom"
//   source_frame  default "base_footprint"
//   publish_hz    default 20.0
//   topic         default "/ground_truth/pose"
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

class TruthRelay : public rclcpp::Node {
public:
  TruthRelay() : Node("truth_relay") {
    declare_parameter("target_frame", std::string("odom"));
    declare_parameter("source_frame", std::string("base_footprint"));
    declare_parameter("publish_hz", 20.0);
    declare_parameter("topic", std::string("/ground_truth/pose"));

    target_ = get_parameter("target_frame").as_string();
    source_ = get_parameter("source_frame").as_string();
    double hz = get_parameter("publish_hz").as_double();
    auto topic = get_parameter("topic").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, hz)));
    timer_ = create_wall_timer(period, [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
        "truth_relay: %s <- %s @ %.1f Hz -> %s",
        target_.c_str(), source_.c_str(), hz, topic.c_str());
  }

private:
  void tick() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(target_, source_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      // TF not yet available (sim still spinning up): silent throttle.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "no TF %s -> %s yet: %s", target_.c_str(), source_.c_str(), ex.what());
      return;
    }
    geometry_msgs::msg::PoseStamped p;
    p.header = tf.header;
    p.pose.position.x = tf.transform.translation.x;
    p.pose.position.y = tf.transform.translation.y;
    p.pose.position.z = tf.transform.translation.z;
    p.pose.orientation = tf.transform.rotation;
    pub_->publish(p);
  }

  std::string target_, source_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TruthRelay>());
  rclcpp::shutdown();
  return 0;
}
