#include <Eigen/Core>
#include <Eigen/Geometry>

#include "node.h"
#include <chrono>
#include <cmath>
#include <algorithm>

MapBuilderNode::MapBuilderNode() : Node("map_builder")
{

    this->declare_parameter<double>("resolution", 0.01); // [m/cell]

    _resolution = this->get_parameter("resolution").as_double();

    this->declare_parameter<double>("max_range", 12.0); // [meters]

    float max_range = this->get_parameter("max_range").as_double();

    _width = (2 * max_range) / _resolution;
    _height = (2 * max_range) / _resolution;

    // This is the origin of the map with respect to the world frame (w_O_map).
    // Remembering that ROS2 OccupancyGrid origin is defined in the bottom left corner,
    // we build a square of 2*max_range side in each direction and we place the robot in world (0,0).
    // Hence, the origin of the map is at (-max_range, -max_range) in world coordinates (the bottom left corner :) ).

    _origin_x = -(_width * _resolution) / 2.0;
    _origin_y = -(_height * _resolution) / 2.0;

    RCLCPP_INFO(
        get_logger(),
        "\nCreating map with: \n\n -resolution = %.2f; \n -max_range=%.1f;\n -(w,h) = (%d,%d);\n -origin = (%.1f,%.1f)\n", _resolution, max_range, _width, _height, _origin_x, _origin_y);

    _map.header.frame_id = "map";

    _map.info.resolution = _resolution;
    _map.info.width = _width;
    _map.info.height = _height;

    _map.info.origin.position.x = _origin_x;
    _map.info.origin.position.y = _origin_y;
    _map.info.origin.orientation.w = 1.0;

    _map.data.resize(_width * _height, -1);
    _counts.assign(_width * _height, Eigen::Vector2d(0.0, 0.0));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(); // This is needed if you want to save the map... Thanks Gemini...
    _map_pub =
        create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map",
            qos);

    _scan_sub =
        create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&MapBuilderNode::scanCallback, this, std::placeholders::_1));

    _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    _static_tf_broadcaster->sendTransform(t);

    _timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(
            &MapBuilderNode::publishMap,
            this));

    RCLCPP_INFO(
        get_logger(),
        "Persistent map builder node started");
};

void MapBuilderNode::publishMap()
{
    for (size_t idx = 0; idx < _counts.size(); ++idx)
    {
        const int counter = _counts[idx].x();
        const int total = _counts[idx].y();

        if (total < 5)
        {
            _map.data[idx] = -1; // never observed
            continue;
        }

        const float ratio = static_cast<float>(counter) / static_cast<float>(wall_score * total);
        _map.data[idx] = (ratio > WALL_THRESH) ? 100 : 0;
    }

    _map.header.stamp = now();

    _map_pub->publish(_map);
}

void MapBuilderNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Check if we already have the cached pose
    if (!_found_lidar_pose)
    {
        try
        {
            // Lookup latest available transform
            geometry_msgs::msg::TransformStamped tf_stamped = _tf_buffer->lookupTransform(
                _map.header.frame_id, // "map"
                msg->header.frame_id, // "laser"
                tf2::TimePointZero);

            // Extract 2D translation
            _lidar_origin = Eigen::Vector2f(
                tf_stamped.transform.translation.x,
                tf_stamped.transform.translation.y);

            // Extract 2D rotation (yaw angle)
            const auto &q = tf_stamped.transform.rotation;
            const float yaw = std::atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
            _laser_rot = Eigen::Rotation2Df(yaw).toRotationMatrix();

            _found_lidar_pose = true;
            RCLCPP_INFO(get_logger(), "Successfully cached LiDAR TF pose!");
        }
        catch (const tf2::TransformException &ex)
        {
            // If the TF broadcaster isn't ready yet, log a throttled warning and wait for the next scan
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "LiDAR pose TF not available yet (%s). Retrying on next scan...", ex.what());
            return; // Gracefully drop scan processing until TF becomes available
        }
    }

    // Process scan using cached variables
    float alpha = msg->angle_min;
    for (const auto &range : msg->ranges)
    {
        if (std::isnan(range) || std::isinf(range) ||
            range < msg->range_min || range > msg->range_max)
        {
            alpha += msg->angle_increment;
            continue;
        }

        MapBuilderNode::scanRay(alpha, range, msg->range_min, _lidar_origin, _laser_rot);
        alpha += msg->angle_increment;
    }
}
void MapBuilderNode::scanRay(
    const float alpha,
    const float range,
    const float range_min,

    const Eigen::Vector2f &origin_map,
    const Eigen::Matrix2f &laser_rot)
{
    // Direction vector in laser local frame
    const Eigen::Vector2f dir_local(std::cos(alpha), -std::sin(alpha));

    // Direction vector rotated into target frame
    const Eigen::Vector2f dir_map = laser_rot * dir_local;

    // Raycast starting from the dynamic sensor position
    Eigen::Vector2f p = origin_map + dir_map * range_min;
    float l = range_min;

    while (l < range)
    {
        const int x = std::floor((p.x() - _origin_x) / _resolution);
        const int y = std::floor((p.y() - _origin_y) / _resolution);
        if (x >= 0 && x < _width && y >= 0 && y < _height)
        {
            const size_t idx = y * _width + x;
            _counts[idx].x() -= free_score;
            _counts[idx].y() += 1.0; // total observations
        }
        l += _resolution;
        p += dir_map * _resolution;
    }

    // Set obstacle endpoint
    p = origin_map + dir_map * range;
    const int x = std::floor((p.x() - _origin_x) / _resolution);
    const int y = std::floor((p.y() - _origin_y) / _resolution);
    if (x >= 0 && x < _width && y >= 0 && y < _height)
    {
        const size_t idx = y * _width + x;
        _counts[idx].x() += wall_score; // occupied evidence
        _counts[idx].y() += 1.0;        // total observations
    }
}