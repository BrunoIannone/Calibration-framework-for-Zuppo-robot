#pragma once

#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <vector>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/**
 * @brief ROS 2 node responsible for generating and publishing an Occupancy Grid map using 2D LaserScan data.
 */
class MapBuilderNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructs a new MapBuilderNode object and initializes subscribers, publishers, and timers
     */
    MapBuilderNode();

    /**
     * @brief Callback function to process incoming laser scan messages
     * @param msg Shared pointer to the received LaserScan message
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Raycasts an individual laser beam to update values in the occupancy grid
     * @param alpha Angle of the specific laser ray in radians
     * @param range Maximum distance of the ray
     * @param range_min Minimum valid range threshold for the sensor
     * @param origin_map 2D translation vector representing the origin position
     * @param laser_rot 2D rotation matrix representing the orientation of the sensor
     */
    void scanRay(const float alpha,
                 const float range,
                 const float range_min,
                 const Eigen::Vector2f &origin_map,
                 const Eigen::Matrix2f &laser_rot);

private:
    // METHOD DEFINITIONS

    /**
     * @brief Periodic timer callback to publish the updated occupancy grid map
     */
    void publishMap(); // timer callback

    // MEMBER VARIABLES

    uint32_t _width;
    uint32_t _height;

    float _resolution;

    float _origin_x; ///< Bottom left corner x
    float _origin_y; ///< Bottom left corner x

    // ROS interfaces
    nav_msgs::msg::OccupancyGrid _map; ///< Occupancy grid data structure to be published

    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub; ///< Subscriber for incoming laser scans

    rclcpp::Publisher<
        nav_msgs::msg::OccupancyGrid>::SharedPtr _map_pub; ///< Publisher for the calculated occupancy map

    rclcpp::TimerBase::SharedPtr _timer;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;

    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    bool _found_lidar_pose = false; ///< Flag indicating if the TF lookup for the LiDAR pose succeeded
    Eigen::Vector2f _lidar_origin; ///<  LiDAR frame origin
    Eigen::Matrix2f _laser_rot; ///< Rotation matrix for the LiDAR frame

    std::vector<Eigen::Vector2d> _counts; ///< Accumulation array for cell states: x = total score, y = total observation count

    const float WALL_THRESH = 0.2f; ///< Threshold for a cell to be a wall
    const float wall_score = 10.0;  ///< Score added for occupied cell observations
    const float free_score = 0.1;   ///< Score added for free space cell observations
};