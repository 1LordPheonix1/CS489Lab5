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
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

using namespace std;

class Logger : public rclcpp::Node
{
    private:
    std::string logFileName;

    public:
    Logger() : Node("logger_node")
    {
        this->declare_parameter("file", "/sim_ws/src/pure_pursuit/src/waypoints.csv");
        this->declare_parameter("logging", false);
        this->declare_parameter("mode", "sim");

        get_parameter("file", fileName);
        get_parameter("logging", logging);
        get_parameter("mode", mode);

        if(!logging) {return;}
        RCLCPP_INFO(this->get_logger(), "opening file");

        file.open(fileName.c_str());
        publisher_marker = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker_array", 100);

        if(mode == "sim") {
            subscription = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom",1000,std::bind(&Logger::saveWaypoint, this, _1));
 
            // subscriber_ecoracecarOdom = this->create_subscription<nav_msgs::msg::Odometry>(
            // "/pf/pose/odom", 1000, std::bind(&PurePursuit::pose_callback, this, _1)
            // ); 
        } else if(mode == "v") {
            subscription = this->create_subscription<nav_msgs::msg::Odometry>("/pf/pose/odom",1000,std::bind(&Logger::saveWaypoint, this, _1));

        }
        RCLCPP_INFO(this->get_logger(), "beginning reading");

        timer_ = this->create_wall_timer(
            500ms, std::bind(&Logger::timer_callback, this)
        );
    }
    std::string mode;
    std::string fileName;
    rclcpp::TimerBase::SharedPtr timer_;
    bool logging = false;
    std::ofstream file;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker;

    // std::string getLogFileName() {
    //     time_t now = time(0);
    //     struct tm tstruct;
    //     char buf[80];
    //     tstruct = *localtime(&now);
    //     strftime(buf, sizeof(buf), "/home/%Y-%m-%d-%H-%M-%S.csv", &tstruct);
    //     return std::string(buf);
    // }

    visualization_msgs::msg::Marker visualizer(float x_position, float y_position, int ID){
        // RCLCPP_INFO(this->get_logger(), "marker: %d", ID);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp =  this->now();
        marker.ns = "waypoints";
        marker.id = ID;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x_position;
        marker.pose.position.y = y_position;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0; //can get rid of default zero ones
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
        return marker;
        // publisher_markerArray->publish(marker);
    }

    nav_msgs::msg::Odometry::ConstSharedPtr data;

    void saveWaypoint(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        data = msg;
    }

    int ID = 0;
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

        if(speed < 0.1) {
            return;
            // ignore point
        }

        visualization_msgs::msg::Marker marker = visualizer(data->pose.pose.position.x, data->pose.pose.position.y, ID);
        publisher_marker->publish(marker);
        ID++;


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

