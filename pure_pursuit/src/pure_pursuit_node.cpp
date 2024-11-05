#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

class PurePursuit : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher;

    // Define params
    double lookahead_distance = 1.0; // Lookahead distance
    double max_steering_angle = 20.0; // Max steering angle
    double speed = 1.0; // Desired speed
    std::string pose_topic = "/pose";       // Our car pose topic
    std::string drive_topic = "/drive";     // Drive topic

public: 
    PurePursuit() : Node("pure_pursuit_node")
    {


        // Initialize the publisher and subscriber
        publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10, std::bind(&PurePursuit::pose_callback, this, _1)
        );

        // Declare and get parameters from launch file 
        this->declare_parameter("distance", lookahead_distance);
        this->declare_parameter("speed", speed);
        this->get_parameter("distance", lookahead_distance);
        this->get_parameter("speed", speed);
    }



void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{
    // Calculate the lookahead target point
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = pose_msg->pose.position.x + lookahead_distance;
    target_pose.position.y = pose_msg->pose.position.y;

    // Calculate the x and y
    double dx = target_pose.position.x - pose_msg->pose.position.x; //target x - our pose x
    double dy = target_pose.position.y - pose_msg->pose.position.y; //target y - our pose y
    
    
    //this in slide formula (day 17 slide 25) in a sence our triangle to a curve

    double L = std::sqrt(dx * dx + dy * dy); //our L lookahead our (euclidean distance) ex tringle in picture
    double y = std::abs(dy); //we need |dy| 
    double r = (L * L) / (2 * y); //our r = ( L^2 ) / 2 |dy|

    // r = C/2sin(angle) -> angle = asin(C/2r)

    float steering_angle = std::asin(L/((float)2.0*r));


    // double curvature = 1.0 / r; //we calculate our curvature on slide 26

    // Convert curvature to steering angle in degrees
    // double steering_angle_deg = -std::atan(curvature) * (180.0 / M_PI);
    //clamping steering angle
    steering_angle = std::max(std::min(steering_angle, max_steering_angle), -max_steering_angle);

    // Create and publish the drive message
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.steering_angle = steering_angle_deg; // now in degrees
    drive_msg.drive.speed = speed;

    publisher->publish(drive_msg);
}


    ~PurePursuit() {}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
