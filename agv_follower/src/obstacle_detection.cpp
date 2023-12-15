#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "visualization_msgs/msg/marker.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
// #include "pcl_conversions/pcl_conversions.h"
// #include "pcl/filters/voxel_grid.h"
// #include "pcl/filters/passthrough.h"
// #include "pcl/kdtree/kdtree_flann.h"
// #include "pcl/segmentation/extract_clusters.h"

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector() : Node("obstacle_detector")
    {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ObstacleDetector::laser_callback, this, std::placeholders::_1));
        // pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
        obstacles_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("inflated_obstacles", 10);
        obstacle_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("obstacles", 10);

        cluster_tolerance_ = 0.1;
        min_cluster_size_ = 5;
        max_cluster_size_ = 1000;
        inflation_radius_ = 0.3;
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
    {

        std::vector<geometry_msgs::msg::Point> obstacles;
        double min_obstacle_size = 0.2; // Set minimum obstacle size to 0.2 meters
        geometry_msgs::msg::Point last_obstacle_point;
        bool last_point_set = false;
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            if (scan_msg->ranges[i] < 1.0)
            { // Threshold for obstacle detection
                // Convert polar coordinates to Cartesian coordinates
                double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                double x = scan_msg->ranges[i] * std::cos(angle);
                double y = scan_msg->ranges[i] * std::sin(angle);
                // Add the obstacle point to the list
                geometry_msgs::msg::Point obstacle;
                obstacle.x = x;
                obstacle.y = y;
                obstacle.z = 0.0;
                if (last_point_set)
                {
                    double dist = std::sqrt(std::pow(obstacle.x - last_obstacle_point.x, 2) + std::pow(obstacle.y - last_obstacle_point.y, 2));
                    if (dist > min_obstacle_size)
                    {
                        obstacles.push_back(obstacle);
                        last_obstacle_point = obstacle;
                    }
                }
                else
                {
                    obstacles.push_back(obstacle);
                    last_obstacle_point = obstacle;
                    last_point_set = true;
                }
            }
        }

        nav_msgs::msg::OccupancyGrid grid;
        grid.header = laser_scan->header;
        grid.info.resolution = 0.01;        // Set the resolution of the grid
        grid.info.width = 1000;             // Set the width of the grid
        grid.info.height = 1000;            // Set the height of the grid
        grid.info.origin.position.x = 0.0; // Set the origin of the grid
        grid.info.origin.position.y = 0.0;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.x = 0.0;
        grid.info.origin.orientation.y = 0.0;
        grid.info.origin.orientation.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.resize(grid.info.width * grid.info.height);

        // Populate the occupancy grid using the LaserScan data
        for (int i = 0; i < int(laser_scan->ranges.size()); i++)
        {
            float range = laser_scan->ranges[i];
            // Convert range to grid coordinates
            int x = (int)((range * std::cos(laser_scan->angle_min + i * laser_scan->angle_increment)) / grid.info.resolution + grid.info.width / 2);
            int y = (int)((range * std::sin(laser_scan->angle_min + i * laser_scan->angle_increment)) / grid.info.resolution + grid.info.height / 2);
            // Set the occupancy value at the corresponding grid cell
            if (x >= 0 && x < int(grid.info.width) && y >= 0 && y < int(grid.info.height))
            {
                grid.data[x + y * int(grid.info.width)] = 100;
            }
        }


        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header = scan_msg->header;
        marker_msg.ns = "obstacles";
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::msg::Marker::POINTS;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.points = std::vector<geometry_msgs::msg::Point32>(obstacles.begin(), obstacles.end());
        marker_msg.color.r = 1.0;
        marker_msg.color.a = 1.0;
        marker_msg.scale.x = 0.1;
        marker_msg.scale.y = 0.1;
        marker_msg.scale.z = 0.1;
        obstacle_marker_publisher_->publish(marker_msg);
        // Inflate the occupancy grid
        // int inflation_radius = 5;
        // for (int x = 0; x < int(grid.info.width); x++)
        // {
        //     for (int y = 0; y < int(grid.info.height); y++)
        //     {
        //         if (grid.data[x + y * int(grid.info.width)] == 100)
        //         {
        //             for (int dx = -inflation_radius; dx <= inflation_radius; dx++)
        //             {
        //                 for (int dy = -inflation_radius; dy <= inflation_radius; dy++)
        //                 {
        //                     int nx = x + dx;
        //                     int ny = y + dy;
        //                     if (nx >= 0 && nx < int(grid.info.width) && ny >= 0 && ny < int(grid.info.height))
        //                     {
        //                         double dist = std::sqrt(std::pow(nx - x, 2) + std::pow(ny - y, 2));
        //                         if (dist <= inflation_radius)
        //                         {
        //                             grid.data[nx + ny * int(grid.info.width)] = std::max(grid.data[nx + ny * int(grid.info.width)], 50);
        //                         }
        //                     }
        //                 }
        //             }
        //         }
        //     }
        // }

        // Publish the occupancy grid
        
        obstacles_grid_pub_->publish(std::move(grid));
    }
    // pcl::PointXYZ inflatePoint(const pcl::PointXYZ &point)
    // {
    //     tf2::Vector3 point_tf(point.x, point.y, point.z);
    //     try
    //     {
    //         geometry_msgs::msg::TransformStamped transform_stamped =
    //             tf_buffer_->lookupTransform("base_link", "laser", this->now());
    //         tf2::Transform transform;
    //         tf2::fromMsg(transform_stamped.transform, transform);
    //         tf2::Vector3 inflated_point_tf = transform * point_tf;
    //         inflated_point_tf.normalize();
    //         inflated_point_tf *= inflation_radius_;
    //         pcl::PointXYZ inflated_point(inflated_point_tf.x(), inflated_point_tf.y(), inflated_point_tf.z());
    //         return inflated_point;
    //     }
    //     catch (tf2::TransformException &ex)
    //     {
    //         RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    //         return point;
    //     }
    // }

    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double inflation_radius_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacles_grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_publisher_;
    // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
