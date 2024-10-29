//
//  tester.cpp
//  
//
//  Created by Taylor Gauthier on 10/24/24.
//

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <ctime>
// #include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <string>

using std::placeholders::_1;

using namespace std;

class Logger : public rclcpp::Node
{
    private:
    std::string logFileName;

    public:
    Logger() : Node("logger_node")
    {
        logFileName = getLogFileName();
        file.open(logFileName.c_str());
        subscription = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom",1000,std::bind(&Logger::saveWaypoint, this, _1));

        timer_ = this->create_wall_timer(
            1000ms, std::bind(&Logger::timer_callback, this)
        );
    }
    rclcpp::TimerBase::SharedPtr timer_;

    std::ofstream file;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;

    std::string getLogFileName() {
        time_t now = time(0);
        struct tm tstruct;
        char buf[80];
        tstruct = *localtime(&now);
        strftime(buf, sizeof(buf), "/home/%Y-%m-%d-%H-%M-%S.csv", &tstruct);
        return std::string(buf);
    }

    nav_msgs::msg::Odometry::ConstSharedPtr data;

    void saveWaypoint(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        data = msg;
    }

    void timer_callback() {
        // Extract quaternion
        double qx = data->pose.pose.orientation.x;
        double qy = data->pose.pose.orientation.y;
        double qz = data->pose.pose.orientation.z;
        double qw = data->pose.pose.orientation.w;

        // Convert quaternion to Euler angles
        tf2::Quaternion quat(qx, qy, qz, qw);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        // Calculate speed
        double speed = std::sqrt(std::pow(data->twist.twist.linear.x, 2) +
                                std::pow(data->twist.twist.linear.y, 2) +
                                std::pow(data->twist.twist.linear.z, 2));

        if (data->twist.twist.linear.x > 0.0) {
            std::cout << data->twist.twist.linear.x << std::endl;
        }

        // Write to file
        file << data->pose.pose.position.x << ", "
            << data->pose.pose.position.y << ", "
            << yaw << ", "
            << speed << std::endl;
    }

    ~Logger() {
        if (file.is_open()) {
            file.close();
        }
        // ROS_INFO("Goodbye");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Logger>());
    // auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(100));
    // ROS_INFO("Saving waypoints...");
    rclcpp::shutdown();
    return 0;
}

