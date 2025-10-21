/**
 * Obstacle Monitor Node
 * Monitors LiDAR scan data and triggers emergency stop when obstacles are too close
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <algorithm>

class ObstacleMonitor : public rclcpp::Node
{
public:
    ObstacleMonitor() : Node("obstacle_monitor")
    {
        // Declare parameters
        this->declare_parameter("use_sim_time", true);
        this->declare_parameter("safety_distance", 0.18);
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter("enable_safety", true);
        
        // Get parameters
        safety_distance_ = this->get_parameter("safety_distance").as_double();
        enable_safety_ = this->get_parameter("enable_safety").as_bool();
        
        // Create subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_topic").as_string(), 10,
            std::bind(&ObstacleMonitor::scanCallback, this, std::placeholders::_1));
        
        cmd_vel_input_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_input", 10,
            std::bind(&ObstacleMonitor::cmdVelInputCallback, this, std::placeholders::_1));
        
        // Create publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            this->get_parameter("cmd_vel_topic").as_string(), 10);
        
        emergency_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/emergency_stop", 10);
        
        // Initialize state
        emergency_stop_active_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Obstacle Monitor Node started");
        RCLCPP_INFO(this->get_logger(), "Safety distance: %.2f m", safety_distance_);
        RCLCPP_INFO(this->get_logger(), "Safety enabled: %s", enable_safety_ ? "true" : "false");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!enable_safety_) {
            return;
        }
        
        // Find minimum distance in scan
        float min_distance = std::numeric_limits<float>::max();
        for (const auto& range : msg->ranges) {
            if (std::isfinite(range) && range > msg->range_min && range < msg->range_max) {
                min_distance = std::min(min_distance, range);
            }
        }
        
        // Check if obstacle is too close
        bool should_stop = (min_distance < safety_distance_);
        
        if (should_stop && !emergency_stop_active_) {
            RCLCPP_WARN(this->get_logger(), 
                "EMERGENCY STOP! Obstacle detected at %.2f m", min_distance);
            emergency_stop_active_ = true;
            
            // Publish emergency stop status
            auto stop_msg = std_msgs::msg::Bool();
            stop_msg.data = true;
            emergency_stop_pub_->publish(stop_msg);
            
            // Publish zero velocity
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_cmd);
        }
        else if (!should_stop && emergency_stop_active_) {
            RCLCPP_INFO(this->get_logger(), "Obstacle cleared, resuming normal operation");
            emergency_stop_active_ = false;
            
            // Publish emergency stop deactivated
            auto stop_msg = std_msgs::msg::Bool();
            stop_msg.data = false;
            emergency_stop_pub_->publish(stop_msg);
        }
        
        // Store latest scan data
        latest_scan_ = msg;
    }
    
    void cmdVelInputCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Filter command velocity based on emergency stop state
        if (emergency_stop_active_) {
            // Publish zero velocity during emergency stop
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_cmd);
        }
        else {
            // Forward command velocity normally
            cmd_vel_pub_->publish(*msg);
        }
    }
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_input_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
    
    // Parameters
    double safety_distance_;
    bool enable_safety_;
    
    // State
    bool emergency_stop_active_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

