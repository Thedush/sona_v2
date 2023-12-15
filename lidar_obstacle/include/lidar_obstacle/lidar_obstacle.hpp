#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "visualization_msgs/msg/marker.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/int16_multi_array.hpp"

// #include "pcl_conversions/pcl_conversions.h"
// #include "pcl/filters/voxel_grid.h"
// #include "pcl/filters/passthrough.h"
// #include "pcl/kdtree/kdtree_flann.h"
// #include "pcl/segmentation/extract_clusters.h"

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector() ;

private:
    
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
    // void publish_marker(double range, double r, double g, double b);
    // void publish_marker(double range, double r, double g, double b, double min_range, double max_range, double angle_min, double angle_max);

    float collision_zone_range,slow_zone_range,free_zone_range;
    int collisionzone_probability_counter,slowzone_probability_counter;
    int obstacle_probability;
    int min_cluster_size_;
    int max_cluster_size_;
    double inflation_radius_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacles_grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr zone_pub_;
    visualization_msgs::msg::Marker marker_msg_;
    visualization_msgs::msg::Marker marker_;
    double zone_size_;
    // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};