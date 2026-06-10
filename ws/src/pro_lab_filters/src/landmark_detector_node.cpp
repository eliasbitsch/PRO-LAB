// Self-defined landmarks for the PRO-LAB assignment — REAL scan-cluster
// detection (no ground truth used to generate the measurement).
//
// Landmarks are physical vertical posts (cylinders) spawned in the gz world
// (see config/landmark_post.sdf + the experiment launch) at the fixed map
// positions in config/landmarks.yaml. This node:
//   1) republishes the landmark positions as an RViz MarkerArray.
//   2) DETECTS the posts in the live /scan:
//        - convert beams to (x,y) in the base frame (lidar TF applied)
//        - segment into clusters by range/gap discontinuity
//        - keep small, isolated, post-sized clusters
//        - report each as [id, range, bearing] on /landmarks/observations
//      Data association (which post) uses the robot's TF pose, NOT to fake the
//      measurement — only to label the detection (known-correspondence).
//   3) VALIDATES the detector against Gazebo ground truth: publishes the
//      detector's range/bearing error vs the true range/bearing on
//      /landmarks/detector_error ([id, range_err, bearing_err, ...]) so the
//      detector accuracy can be plotted. GT is used ONLY here, for validation.
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class LandmarkDetector : public rclcpp::Node {
public:
  LandmarkDetector() : Node("landmark_detector") {
    declare_parameter<std::vector<int64_t>>("landmark_ids", {1, 2, 3});
    declare_parameter<std::vector<double>>("landmark_xs",  {-5.0,  3.0, -2.0});
    declare_parameter<std::vector<double>>("landmark_ys",  { 2.0, -2.0, -6.0});
    declare_parameter("max_range",       6.0);
    declare_parameter("frame_id",        std::string("map"));
    declare_parameter("base_frame",      std::string("base_footprint"));
    // Scan-cluster detector knobs:
    declare_parameter("cluster_gap",     0.20);  // m, split clusters on bigger jumps
    declare_parameter("max_cluster_size", 1.80); // m, very fat I-beams (1.40 wide)
    declare_parameter("min_cluster_pts", 2);     // ignore single stray returns
    declare_parameter("assoc_gate",      0.5);   // m, tight: only the real post associates

    const auto ids = get_parameter("landmark_ids").as_integer_array();
    const auto xs  = get_parameter("landmark_xs").as_double_array();
    const auto ys  = get_parameter("landmark_ys").as_double_array();
    for (std::size_t i = 0; i < std::min({ids.size(), xs.size(), ys.size()}); ++i) {
      lms_.push_back({static_cast<int>(ids[i]), xs[i], ys[i]});
    }
    max_range_       = get_parameter("max_range").as_double();
    frame_id_        = get_parameter("frame_id").as_string();
    base_frame_      = get_parameter("base_frame").as_string();
    cluster_gap_     = get_parameter("cluster_gap").as_double();
    max_cluster_sz_  = get_parameter("max_cluster_size").as_double();
    min_cluster_pts_ = static_cast<int>(get_parameter("min_cluster_pts").as_int());
    assoc_gate_      = get_parameter("assoc_gate").as_double();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    obs_pub_   = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/landmarks/observations", 10);
    err_pub_   = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/landmarks/detector_error", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/landmarks/markers", rclcpp::QoS(1).transient_local());
    publishMarkers();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr m) { onScan(*m); });

    RCLCPP_INFO(get_logger(),
      "landmark_detector: SCAN-CLUSTER mode, %zu posts, gap=%.2fm, "
      "max_size=%.2fm, gate=%.2fm", lms_.size(), cluster_gap_,
      max_cluster_sz_, assoc_gate_);
  }

private:
  struct Landmark { int id; double x; double y; };
  struct Pt { double x, y; };

  void publishMarkers() {
    visualization_msgs::msg::MarkerArray arr;
    for (const auto & lm : lms_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp    = now();
      m.ns = "landmarks"; m.id = lm.id;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = lm.x; m.pose.position.y = lm.y; m.pose.position.z = 0.5;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.15; m.scale.y = 0.15; m.scale.z = 1.0;
      m.color.r = 1.0; m.color.g = 0.7; m.color.b = 0.0; m.color.a = 0.9;
      arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
  }

  // Cache the static lidar TF (base <- scan frame) on first scan.
  bool ensureLidarTf(const std::string & scan_frame) {
    if (have_lidar_tf_) return true;
    try {
      auto tf = tf_buffer_->lookupTransform(base_frame_, scan_frame,
                                            tf2::TimePointZero);
      lidar_x_ = tf.transform.translation.x;
      lidar_y_ = tf.transform.translation.y;
      tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w);
      double r, p, y; tf2::Matrix3x3(q).getRPY(r, p, y);
      lidar_yaw_ = y; have_lidar_tf_ = true;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "landmark_detector: waiting for lidar TF %s <- %s: %s",
        base_frame_.c_str(), scan_frame.c_str(), ex.what());
      return false;
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan & m) {
    if (!ensureLidarTf(m.header.frame_id)) return;

    // 1) Beams -> (x, y) in base frame.
    const double cs_l = std::cos(lidar_yaw_), sn_l = std::sin(lidar_yaw_);
    std::vector<Pt> pts; pts.reserve(m.ranges.size());
    std::vector<bool> valid; valid.reserve(m.ranges.size());
    for (std::size_t i = 0; i < m.ranges.size(); ++i) {
      const double r = m.ranges[i];
      if (!std::isfinite(r) || r < m.range_min || r > max_range_) {
        pts.push_back({0, 0}); valid.push_back(false); continue;
      }
      const double a = m.angle_min + i * m.angle_increment;
      const double lx = r * std::cos(a), ly = r * std::sin(a);
      pts.push_back({lidar_x_ + cs_l * lx - sn_l * ly,
                     lidar_y_ + sn_l * lx + cs_l * ly});
      valid.push_back(true);
    }

    // 2) Segment into clusters: contiguous valid beams whose neighbours are
    //    within cluster_gap of each other. Post-sized, isolated clusters are
    //    landmark candidates.
    std::vector<Pt> candidates;  // base-frame centroids
    std::size_t i = 0;
    const std::size_t n = pts.size();
    while (i < n) {
      if (!valid[i]) { ++i; continue; }
      std::size_t j = i + 1;
      double cx = pts[i].x, cy = pts[i].y;
      double minx = pts[i].x, maxx = pts[i].x, miny = pts[i].y, maxy = pts[i].y;
      int count = 1;
      while (j < n && valid[j] &&
             std::hypot(pts[j].x - pts[j - 1].x, pts[j].y - pts[j - 1].y) < cluster_gap_) {
        cx += pts[j].x; cy += pts[j].y;
        minx = std::min(minx, pts[j].x); maxx = std::max(maxx, pts[j].x);
        miny = std::min(miny, pts[j].y); maxy = std::max(maxy, pts[j].y);
        ++count; ++j;
      }
      const double size = std::hypot(maxx - minx, maxy - miny);
      if (count >= min_cluster_pts_ && size <= max_cluster_sz_) {
        candidates.push_back({cx / count, cy / count});
      }
      i = j;
    }

    int valid_pts = 0;
    for (bool v : valid) if (v) ++valid_pts;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "scan: %d valid pts -> %zu post-sized candidates", valid_pts,
        candidates.size());

    // 3) Robot pose in map frame (for data association + validation only).
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(frame_id_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "no %s <- %s TF for association: %s", frame_id_.c_str(),
          base_frame_.c_str(), ex.what());
      return;  // no pose → can't associate this scan
    }
    const double rx = tf.transform.translation.x;
    const double ry = tf.transform.translation.y;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    double rr, rp, ryaw; tf2::Matrix3x3(q).getRPY(rr, rp, ryaw);
    const double cs_r = std::cos(ryaw), sn_r = std::sin(ryaw);

    // 4) Associate detections to known landmarks: for EACH landmark take the
    //    single CLOSEST candidate within the (tight) gate. Emits at most one
    //    observation per landmark — the real post (~0.07-0.2 m) — and rejects
    //    wall/shelf clusters that would otherwise be mis-associated and throw
    //    the landmark-based filters (KF/EKF) off. Range/bearing are the REAL
    //    detected values (base frame); GT is used only to validate the error.
    std_msgs::msg::Float32MultiArray obs, err;
    double dbg_min_d = std::numeric_limits<double>::infinity();
    double dbg_mx = 0, dbg_my = 0;
    std::vector<std::pair<double, double>> cmap;  // candidate (mx, my) in map
    cmap.reserve(candidates.size());
    for (const auto & c : candidates)
      cmap.emplace_back(rx + cs_r * c.x - sn_r * c.y, ry + sn_r * c.x + cs_r * c.y);
    for (const auto & lm : lms_) {
      int best = -1; double best_d = assoc_gate_;
      for (std::size_t k = 0; k < candidates.size(); ++k) {
        const double d = std::hypot(lm.x - cmap[k].first, lm.y - cmap[k].second);
        if (d < dbg_min_d) { dbg_min_d = d; dbg_mx = cmap[k].first; dbg_my = cmap[k].second; }
        if (d < best_d) { best_d = d; best = static_cast<int>(k); }
      }
      if (best < 0) continue;  // no candidate close enough to this landmark
      const auto & c = candidates[best];
      const double range   = std::hypot(c.x, c.y);
      const double bearing = std::atan2(c.y, c.x);
      obs.data.push_back(static_cast<float>(lm.id));
      obs.data.push_back(static_cast<float>(range));
      obs.data.push_back(static_cast<float>(bearing));
      // Validation: true range/bearing from GT pose to this known landmark.
      const double tdx = lm.x - rx, tdy = lm.y - ry;
      const double true_range = std::hypot(tdx, tdy);
      double true_bearing = std::atan2(tdy, tdx) - ryaw;
      while (true_bearing >  M_PI) true_bearing -= 2 * M_PI;
      while (true_bearing < -M_PI) true_bearing += 2 * M_PI;
      err.data.push_back(static_cast<float>(lm.id));
      err.data.push_back(static_cast<float>(range - true_range));
      err.data.push_back(static_cast<float>(bearing - true_bearing));
    }
    if (!obs.data.empty()) obs_pub_->publish(obs);
    if (!err.data.empty()) err_pub_->publish(err);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "robot@map=(%.2f,%.2f) -> %zu detections | closest candidate@map="
        "(%.2f,%.2f) is %.2fm from nearest landmark (gate=%.2f)",
        rx, ry, obs.data.size() / 3, dbg_mx, dbg_my, dbg_min_d, assoc_gate_);
  }

  std::vector<Landmark> lms_;
  double max_range_ {6.0}, cluster_gap_ {0.2}, max_cluster_sz_ {0.5}, assoc_gate_ {1.0};
  int min_cluster_pts_ {2};
  std::string frame_id_ {"map"}, base_frame_ {"base_footprint"};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool   have_lidar_tf_ {false};
  double lidar_x_ {0.0}, lidar_y_ {0.0}, lidar_yaw_ {0.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr err_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkDetector>());
  rclcpp::shutdown();
  return 0;
}
