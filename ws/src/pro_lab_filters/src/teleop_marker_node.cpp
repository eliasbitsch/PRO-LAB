// Drag-to-move interactive marker for RViz2.
//
// Spawns a 6-DOF interactive marker in the "base_footprint" frame so it
// follows the robot. Dragging it in the X/Y plane (forward / sideways
// translate, yaw rotate) maps to a Twist published on /cmd_vel_in.
//
// Why /cmd_vel_in: cmd_vel_watchdog forwards it to /cmd_vel and zeroes
// after a short silence, giving us "release-to-stop" semantics for free.
//
// Idea: marker pose deviation from origin (in base frame) is treated as
// the desired velocity, scaled. So:
//   pull marker forward 1m  -> linear.x = +scale * 1.0
//   yaw marker by 0.5 rad   -> angular.z = +yaw_scale * 0.5
// On mouse-up the marker snaps back to origin and we publish zero.
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class TeleopMarkerNode : public rclcpp::Node {
public:
  TeleopMarkerNode() : Node("teleop_marker") {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    v_scale_    = declare_parameter<double>("linear_scale", 0.6);   // 1 m drag → 0.6 m/s
    w_scale_    = declare_parameter<double>("angular_scale", 1.0);  // 1 rad rot → 1.0 rad/s
    v_max_      = declare_parameter<double>("v_max", 0.3);
    w_max_      = declare_parameter<double>("w_max", 1.0);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_in", 10);

    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "teleop_marker", this);

    makeMarker();
    server_->applyChanges();

    // Republish current twist at 20 Hz so a brief drop doesn't strand cmd_vel
    // at a non-zero value. cmd_vel_watchdog still acts as the safety net.
    timer_ = create_wall_timer(50ms, [this]() {
      pub_->publish(twist_);
    });
  }

private:
  void makeMarker() {
    visualization_msgs::msg::InteractiveMarker im;
    im.header.frame_id = base_frame_;
    im.name            = "teleop";
    im.description     = "Drag to drive";
    im.scale           = 0.6;

    // A small visual sphere so the user sees what they're grabbing.
    visualization_msgs::msg::Marker sphere;
    sphere.type    = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.18;
    sphere.color.r = 0.25f; sphere.color.g = 0.55f; sphere.color.b = 1.0f;
    sphere.color.a = 0.85f;

    visualization_msgs::msg::InteractiveMarkerControl visual;
    visual.always_visible = true;
    visual.markers.push_back(sphere);
    im.controls.push_back(visual);

    // Translation along robot-X (forward/back)
    visualization_msgs::msg::InteractiveMarkerControl move_x;
    move_x.name = "move_x";
    move_x.orientation.w = 1.0;
    move_x.orientation.x = 1.0;
    move_x.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    im.controls.push_back(move_x);

    // Translation along robot-Y (strafe — gets mapped to angular for diff drive)
    visualization_msgs::msg::InteractiveMarkerControl move_y;
    move_y.name = "move_y";
    move_y.orientation.w = 1.0;
    move_y.orientation.z = 1.0;
    move_y.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    im.controls.push_back(move_y);

    // Rotation about Z (yaw)
    visualization_msgs::msg::InteractiveMarkerControl rotate_z;
    rotate_z.name = "rotate_z";
    rotate_z.orientation.w = 1.0;
    rotate_z.orientation.y = 1.0;
    rotate_z.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    im.controls.push_back(rotate_z);

    server_->insert(im,
        [this](const auto & fb) { this->onFeedback(fb); });
  }

  void onFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr fb) {
    using FB = visualization_msgs::msg::InteractiveMarkerFeedback;
    switch (fb->event_type) {
      case FB::POSE_UPDATE: {
        // marker pose is expressed in base_frame_, i.e. relative to the robot.
        const double dx = fb->pose.position.x;
        const double dy = fb->pose.position.y;
        tf2::Quaternion q(fb->pose.orientation.x,
                          fb->pose.orientation.y,
                          fb->pose.orientation.z,
                          fb->pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Diff-drive friendly mapping:
        //   forward drag -> linear.x
        //   yaw rotation -> angular.z (primary)
        //   sideways drag -> additional angular.z (proportional)
        double v = std::clamp(v_scale_ * dx, -v_max_, v_max_);
        double w = std::clamp(w_scale_ * yaw + 0.5 * w_scale_ * dy,
                              -w_max_, w_max_);
        twist_.linear.x  = v;
        twist_.angular.z = w;
        break;
      }
      case FB::MOUSE_UP: {
        // Snap marker back to origin and stop.
        twist_ = geometry_msgs::msg::Twist();
        geometry_msgs::msg::Pose origin;
        origin.orientation.w = 1.0;
        server_->setPose(fb->marker_name, origin);
        server_->applyChanges();
        break;
      }
      default:
        break;
    }
  }

  std::string base_frame_;
  double      v_scale_, w_scale_, v_max_, w_max_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      pub_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  geometry_msgs::msg::Twist                                    twist_;
  rclcpp::TimerBase::SharedPtr                                 timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
