#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "onix_msgs/msg/drive.hpp"

using namespace std::chrono_literals;

class MyRobotControlNode : public rclcpp::Node
{
public:
    MyRobotControlNode()
        : Node("my_robot_control")
    {
        
        pub_ = this->create_publisher<onix_msgs::msg::Drive>("/cmd_drive", 10);
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/onix_velocity_controller/cmd_vel_unstamped", 10, std::bind(&MyRobotControlNode::cmd_vel_callback, this, std::placeholders::_1));
        linear = 0.0;
        angular = 0.0;
        received = false;


        // Create a timer with a 100ms period
        timer_ = this->create_wall_timer(100ms, std::bind(&MyRobotControlNode::timer_callback, this));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linear = msg->linear.x;
        angular = msg->angular.z;
        received = true;
        
    }

    void timer_callback()
    {
        if (!received )
        {
            // No data received, publish zero velocity
            auto zero_velocity = std::make_unique<onix_msgs::msg::Drive>();
            // auto vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
            zero_velocity->drivers[0] = 0.0;
            zero_velocity->drivers[1] = 0.0;
            // zero_velocity->linear.x = 0.0;
            // zero_velocity->angular.z = 0.0;
            pub_->publish(std::move(zero_velocity));
            
        }
        else
        {
            auto external_velocity = std::make_unique<onix_msgs::msg::Drive>();
            external_velocity->drivers[0] = linear;
            external_velocity->drivers[1] = angular;
            // Publish the last received cmd_vel message as motor velocity
            pub_->publish(std::move(external_velocity));
            received = false;
        }
    }

    
    rclcpp::Publisher<onix_msgs::msg::Drive>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    // geometry_msgs::msg::Twist::SharedPtr last_cmd_vel_;
    float linear,angular;
    bool received; 
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyRobotControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
