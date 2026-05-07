// Live "kidnap" the robot: subscribes to /kidnap_pose and calls Gazebo's
// set_pose service to teleport the robot model there.
//
// /kidnap_pose is intentionally separate from /initialpose so the two
// experiments stay independent:
//   /kidnap_pose  -> teleport the GZ model only (creates the kidnapped state)
//   /initialpose  -> reinit the PF only        (RViz/AMCL-style recovery hint)
//
// In Foxglove configure a second "Publish" button (PoseWithCovarianceStamped
// on /kidnap_pose) next to the built-in "Pose Estimate" tool.
#include <cstdlib>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

class RobotTeleporter : public rclcpp::Node {
public:
  RobotTeleporter() : Node("robot_teleporter") {
    model_     = declare_parameter<std::string>("model_name", "turtlebot4");
    world_     = declare_parameter<std::string>("world_name", "warehouse");
    spawn_z_   = declare_parameter<double>("spawn_z", 0.05);
    partition_ = declare_parameter<std::string>("gz_partition", "prolab");

    // Match Foxglove bridge's clientPublish QoS (RELIABLE + TRANSIENT_LOCAL)
    // so we don't lose discovery after a browser F5 reconnect.
    rclcpp::QoS qos(10);
    qos.reliable().transient_local();
    sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/kidnap_pose", qos,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          on_pose(*msg);
        });

    RCLCPP_INFO(get_logger(),
                "teleporter: /kidnap_pose -> gz set_pose for model '%s' in "
                "world '%s' (PF reinit lives separately on /initialpose)",
                model_.c_str(), world_.c_str());
  }

private:
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;

    std::ostringstream req;
    req << "name: \"" << model_ << "\", "
        << "position: {x: " << p.x << ", y: " << p.y
        << ", z: " << spawn_z_ << "}, "
        << "orientation: {x: " << q.x << ", y: " << q.y
        << ", z: " << q.z << ", w: " << q.w << "}";

    std::ostringstream cmd;
    cmd << "GZ_PARTITION=" << partition_
        << " gz service -s /world/" << world_ << "/set_pose"
        << " --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean"
        << " --timeout 1000"
        << " --req '" << req.str() << "' >/dev/null 2>&1";

    int rc = std::system(cmd.str().c_str());
    if (rc == 0) {
      RCLCPP_INFO(get_logger(),
                  "teleported %s to (%.2f, %.2f)",
                  model_.c_str(), p.x, p.y);
    } else {
      RCLCPP_WARN(get_logger(),
                  "gz set_pose failed (rc=%d) for (%.2f, %.2f)",
                  rc, p.x, p.y);
    }
  }

  std::string                                                                       model_;
  std::string                                                                       world_;
  std::string                                                                       partition_;
  double                                                                            spawn_z_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr    sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotTeleporter>());
  rclcpp::shutdown();
  return 0;
}
