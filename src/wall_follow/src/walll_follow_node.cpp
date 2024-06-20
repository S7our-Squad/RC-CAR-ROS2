#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <vector>
#include <cmath>
#include <limits>

using namespace std;

class WallFollow : public rclcpp::Node {
public:
    WallFollow() : Node("wall_follow_node")
    {
        // Initialize the subscriber to LIDAR scan data
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, bind(&WallFollow::scan_callback, this, placeholders::_1));

        // Initialize the publisher for the drive commands
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
        
        // Initialize PID control parameters
        kp = 1.0;
        kd = 0.1;
        ki = 0.0;
    }

private:
    // PID Control Parameters
    double kp;
    double kd;
    double ki;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double integral = 0.0;

    // Topics
    string lidarscan_topic = "/scan";
    string drive_topic = "/drive";

    // ROS Subscriber and Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    double get_range(const vector<float>& ranges, double angle, const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Find the index of the angle in the range data
        int index = static_cast<int>((angle - scan_msg->angle_min) / scan_msg->angle_increment);

        if (index < 0 || static_cast<size_t>(index) >= ranges.size())
        {
            return numeric_limits<double>::quiet_NaN();  // Return NaN if index is out of bounds
        }

        return ranges[index];
    }

    double get_error(const vector<float>& ranges, double desired_distance, const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Define the angles to get range data from LIDAR
        double theta = 45.0 * M_PI / 180.0;  // 45 degrees in radians

        // Get ranges at 45 degrees and 0 degrees
        double front_range = get_range(ranges, 0, scan_msg);
        double side_range = get_range(ranges, theta, scan_msg);

        // Calculate error using the difference between actual and desired distances
        double error = desired_distance - side_range;

        return error;
    }

    void pid_control(double error, double velocity)
    {
        double angle = 0.0;
        
        // PID calculations
        integral += error;
        double derivative = error - prev_error;
        angle = kp * error + ki * integral + kd * derivative;
        prev_error = error;

        // Create and publish drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;
        drive_publisher_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        double desired_distance = 1.0;  // Desired distance from the wall
        double velocity = 1.0;  // Desired car velocity

        // Calculate the error
        double error = get_error(scan_msg->ranges, desired_distance, scan_msg);

        // Perform PID control
        pid_control(error, velocity);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}