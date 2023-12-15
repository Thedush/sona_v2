#include "lidar_obstacle/lidar_obstacle.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
/*
Lidar polar cordinates and its range value
            | +y(1.57)
            |
(+3.14)     |
(-3.14)-----|----------------+x  (0)
            |
            |
            | -y(-1.57)




*/

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector")
{
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ObstacleDetector::laser_callback, this, std::placeholders::_1));
    // pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
    obstacles_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("inflated_obstacles", 10);
    // obstacle_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("obstacles", 10);

    // Advertise the output topic
    zone_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("zone", 10);

    // Advertise the marker topic
    // marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);

    // three zones for saftey values in meter
    collision_zone_range = 0.35;
    slow_zone_range = 1;
    free_zone_range = 2;
    collisionzone_probability_counter = 0;
    slowzone_probability_counter = 0;
    // freezone_probability_counter = 0;
    obstacle_probability = 10;
}

void ObstacleDetector::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{

    collisionzone_probability_counter = 0;
    slowzone_probability_counter = 0;
    // freezone_probability_counter = 0;
    for (size_t i = 0; i < laser_scan->ranges.size(); ++i)
    {
        const float angle = laser_scan->angle_min + i * laser_scan->angle_increment;
        const float range = laser_scan->ranges[i];
        if (!std::isinf(range))
        {
            // RCLCPP_INFO(this->get_logger(), "angle %f, range %f", angle, range);
            //collision zone
            if (range < collision_zone_range)
            {
                // counter which will be used to reduce noise
                collisionzone_probability_counter++;
            }
            //slow zone
            if (range < slow_zone_range)
            {
                // counter which will be used to reduce noise
                slowzone_probability_counter++;
            }
            // safezone
            // if (range < free_zone_range)
            // {
            //     // counter which will be used to reduce noise
            //     freezone_probability_counter++;
            // }
            
        }
    }
    std_msgs::msg::Int16MultiArray zoneData;
    zoneData.data.push_back(0); // collision zone
    zoneData.data.push_back(0); // slow zone
    // zoneData.data.push_back(0); // safe zone

    if (collisionzone_probability_counter > obstacle_probability)
    {
        zoneData.data[0] = 1;
        // zone_pub_->publish(zoneData);
        // RCLCPP_INFO(this->get_logger(), "collision zone obsctacle true");
    }
    else
    {
        zoneData.data[0] = 0;
        // zone_pub_->publish(zoneData);
        // RCLCPP_INFO(this->get_logger(), "collision zone obsctacle false");
    }

    if (slowzone_probability_counter > obstacle_probability)
    {
        zoneData.data[1] = 1;
        // zone_pub_->publish(zoneData);
        // RCLCPP_INFO(this->get_logger(), "slow zone obsctacle true");
    }
    else
    {
        zoneData.data[1] = 0;
        
        // RCLCPP_INFO(this->get_logger(), "slow zone obsctacle false");
    }

    zone_pub_->publish(zoneData);

    // exit(0);
}
