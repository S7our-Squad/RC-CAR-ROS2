#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        // Create ROS subscribers and publishers
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, std::placeholders::_1));
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));
        brake_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/brake", 10);
    }

private:
    double speed = 0.0;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr brake_publisher_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // Update current speed
        speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // Calculate Time-To-Collision (TTC)
        double ttc_threshold = 1.0; // threshold in seconds
        bool need_to_brake = false;

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            double distance = scan_msg->ranges[i];
            if (distance < scan_msg->range_min || distance > scan_msg->range_max) {
                continue;
            }

            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double relative_speed = speed * cos(angle);

            if (relative_speed > 0) {
                double ttc = distance / relative_speed;
                if (ttc < ttc_threshold) {
                    need_to_brake = true;
                    break;
                }
            }
        }

        if (need_to_brake) {
            // Publish drive/brake message
            auto brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
            brake_msg.drive.speed = 0.0;
            brake_publisher_->publish(brake_msg);
            RCLCPP_INFO(this->get_logger(), "Emergency brake applied!");
        }
    }
};