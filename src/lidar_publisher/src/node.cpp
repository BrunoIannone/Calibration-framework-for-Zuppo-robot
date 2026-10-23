#include <cmath>
#include "node.h"
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

LidarPublisherNode::LidarPublisherNode()
    : Node("lidar_publisher"), _scan_count(0)

{
  RCLCPP_INFO(this->get_logger(), "LidarPublisherNode started");

  _lscan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

  _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  this->declare_parameter<double>("max_range", 12.0); // [meters]

  _max_range = this->get_parameter("max_range").as_double();

  this->declare_parameter<int>("lidar_idx", 0);

  _lidar_idx = this->get_parameter("lidar_idx").as_int();

  this->declare_parameter<int>("halt_line", 0);

  _halt_line = this->get_parameter("halt_line").as_int();

  RCLCPP_INFO(this->get_logger(), "Loading data");

  this->declare_parameter<std::string>(
      "map_path",
      "/data.txt");

  // Read parameter
  std::string path =
      this->get_parameter("map_path").as_string();

  _f.open(path);

  if (!_f)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", path.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Publishing started");

  int64_t line_cnt = 0;
  std::string line;
  while (std::getline(_f, line) && line_cnt < _halt_line)
  {
    ++line_cnt;
    std::istringstream ss(line);

    LidarPacket packet;
    std::string tag;
    uint16_t lidar_idx;
    ss >> tag;
    ss >> packet.timestamp >> lidar_idx;

    if (tag == "LIDAR" && lidar_idx == _lidar_idx)
    {

      ss >> packet.start_angle >> packet.end_angle >> packet.speed;

      float range, intensity;
      while (ss >> range >> intensity)
      {
        packet.ranges.push_back(range * 1e-3f);
        packet.intensities.push_back(intensity);
      }
      _packets.push_back(packet);
    }
  }
  _f.close();

  auto t(this->now());

  std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = t;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = 0;
  odom_tf.transform.translation.y = 0;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation.x = 0.0;
  odom_tf.transform.rotation.y = 0.0;
  odom_tf.transform.rotation.z = 0;
  odom_tf.transform.rotation.w = 1;
  static_transforms.push_back(odom_tf);

  odom_tf.header.stamp = t;
  odom_tf.header.frame_id = "base_footprint";
  odom_tf.child_frame_id = "laser";
  odom_tf.transform.translation.x = 0.0;
  odom_tf.transform.translation.y = 0.0;
  odom_tf.transform.translation.z = 0.0;

  // Apply calibration orientation (90 degrees yaw rotation)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI / 2);

  odom_tf.transform.rotation.x = q.x();
  odom_tf.transform.rotation.y = q.y();
  odom_tf.transform.rotation.z = q.z();
  odom_tf.transform.rotation.w = q.w();
  static_transforms.push_back(odom_tf);

  _static_tf_broadcaster->sendTransform(static_transforms);

  _timer = this->create_wall_timer(
      std::chrono::milliseconds(10), // 10 ms
      std::bind(&LidarPublisherNode::publishScan, this));
}

void LidarPublisherNode::publishScan()
{

  if (_scan_count >= _packets.size())
  {
    RCLCPP_INFO(this->get_logger(), "Replay ended");
    _timer->cancel();
    return;
  }

  const auto &packet = _packets[_scan_count];
  const size_t n = packet.ranges.size();
  if (n == 0)
  {
    return;
  }
  auto msg = sensor_msgs::msg::LaserScan();
  auto time_ = this->now();

  msg.header.stamp = time_;
  msg.header.frame_id = "laser";
  msg.ranges = packet.ranges;
  msg.intensities = packet.intensities;

  // convert angles from hundredths of degrees to radians
  msg.angle_min = packet.start_angle * _k;
  msg.angle_max = packet.end_angle * _k;

  float diff = msg.angle_max - msg.angle_min;
  if (diff < 0)
  {
    msg.angle_max += 2 * (float)M_PI;
    diff = msg.angle_max - msg.angle_min;
  }

  msg.angle_increment = diff / std::max(n - 1, size_t(1));
  msg.time_increment = 0.0f;
  msg.scan_time = 0.01f;
  msg.range_min = 0.01f;
  msg.range_max = _max_range;

  _lscan_publisher->publish(msg);

  ++_scan_count;
  if (_scan_count % 1000 == 0)
  {
    RCLCPP_INFO(this->get_logger(), "Replay progress: %.2f%%", _scan_count / (float)_packets.size() * 100);
  }
}
