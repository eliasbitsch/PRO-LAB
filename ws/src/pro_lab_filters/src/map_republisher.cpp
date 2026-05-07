// Latched /map relay. Forwards /map to /map_repub on content change only.
//
// nav2_map_server publishes /map exactly once with TRANSIENT_LOCAL QoS.
// Foxglove sometimes loses that latched copy across panel toggles or
// websocket reconnects, leaving the 3D panel without a map background.
//
// We re-publish on /map_repub with TRANSIENT_LOCAL too, so any subscriber
// — at any time — gets the last map immediately. To avoid forcing
// Foxglove to re-upload the texture (which causes a visible flicker),
// we forward only when the actual map data has changed (hashed).
#include <functional>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

class MapRepublisher : public rclcpp::Node {
public:
  MapRepublisher() : Node("map_republisher") {
    auto in_top  = declare_parameter<std::string>("input_topic",  "/map");
    auto out_top = declare_parameter<std::string>("output_topic", "/map_repub");

    rclcpp::QoS qos(1);
    qos.transient_local().reliable();

    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(out_top, qos);
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        in_top, qos,
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { on_map(*msg); });

    // Republish the cached map at a steady rate so Foxglove never evicts
    // the OccupancyGrid from its renderer when sim_time jumps around.
    // Stamp is forced to 0 so the message is always treated as "current".
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
      if (latest_) {
        pub_->publish(*latest_);
      }
    });

    RCLCPP_INFO(get_logger(),
                "republisher: %s -> %s (latched + 2Hz keepalive, stamp=0)",
                in_top.c_str(), out_top.c_str());
  }

private:
  void on_map(const nav_msgs::msg::OccupancyGrid & msg) {
    std::size_t h = std::hash<std::string>{}(
        std::string(reinterpret_cast<const char *>(msg.data.data()),
                    msg.data.size()));
    h ^= std::hash<unsigned>{}(msg.info.width)  + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<unsigned>{}(msg.info.height) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<float>{}(msg.info.resolution) + 0x9e3779b9 + (h << 6) + (h >> 2);

    if (have_hash_ && h == last_hash_) {
      return;  // identical map — already latched downstream, stay quiet.
    }
    last_hash_ = h;
    have_hash_ = true;

    // Cache with stamp=0 so the timer can spam-publish without Foxglove
    // ever rejecting the message as "in the future" when sim_time jumps.
    auto cached = std::make_shared<nav_msgs::msg::OccupancyGrid>(msg);
    cached->header.stamp.sec     = 0;
    cached->header.stamp.nanosec = 0;
    latest_ = cached;
    pub_->publish(*latest_);
    RCLCPP_INFO(get_logger(), "cached map (%ux%u, res=%.3f)",
                msg.info.width, msg.info.height, msg.info.resolution);
  }

  bool                                                            have_hash_{false};
  std::size_t                                                     last_hash_{0};
  nav_msgs::msg::OccupancyGrid::SharedPtr                         latest_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr   sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr      pub_;
  rclcpp::TimerBase::SharedPtr                                    timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapRepublisher>());
  rclcpp::shutdown();
  return 0;
}
