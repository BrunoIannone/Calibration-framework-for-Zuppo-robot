#pragma once
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

/**
 * @brief Struct that contains all data of a lidar packet
 */
struct LidarPacket
{
  uint64_t timestamp;
  uint16_t start_angle; ///< Hundredths of degrees (0-36000)
  uint16_t end_angle;   ///< Hundredths of degrees (0-36000)
  uint16_t speed;       ///< hundredths of deg/s

  std::vector<float> ranges;      ///< [mm]
  std::vector<float> intensities; ///< Intensity of the point
};

/**
 * @brief ROS 2 node that reads pre-recorded offline LiDAR data logs and replays
 *        them as continuous sensor_msgs/msg/LaserScan
 */
class LidarPublisherNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Lidar Publisher Node object initializes subscribers, publishers, and timers
   */
  LidarPublisherNode();

private:
  // METHOD DEFINITIONS

  /**
   * @brief Wall-timer callback that publishes cached LiDAR packets sequentially
   *        at 100 Hz (every 10 ms)
   */
  void publishScan();

  // MEMBER VARIABLES
  uint64_t _scan_count; ///< Counter tracking current packet index during replay

  float _max_range; ///< Maximum valid LiDAR range in meters
  int _lidar_idx;
  int _halt_line;                         ///< Last line of the log before the robot starts moving
  const float _k = (float)M_PI / 18000.f; ///< Conversion factor: Converts hundredths of degrees to radians (pi / 18000)

  std::ifstream _f;                  ///< File stream object for log reading
  std::vector<LidarPacket> _packets; ///< In-memory cache of parsed LiDAR scans

  // ROS interfaces

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _lscan_publisher;
  rclcpp::TimerBase::SharedPtr _timer;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;
};