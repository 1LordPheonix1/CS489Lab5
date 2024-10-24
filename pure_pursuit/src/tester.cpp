//
//  tester.cpp
//  
//
//  Created by Taylor Gauthier on 10/24/24.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <ctime>
#include <tf/tf.h>
#include <cmath>
#include <string>

std::ofstream file;

std::string getLogFileName() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "/home/%Y-%m-%d-%H-%M-%S.csv", &tstruct);
    return std::string(buf);
}

void saveWaypoint(const nav_msgs::Odometry::ConstPtr& data) {
    // Extract quaternion
    double qx = data->pose.pose.orientation.x;
    double qy = data->pose.pose.orientation.y;
    double qz = data->pose.pose.orientation.z;
    double qw = data->pose.pose.orientation.w;

    // Convert quaternion to Euler angles
    tf::Quaternion quat(qx, qy, qz, qw);
    tf::Matrix3x3 mat(quat);
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

void shutdown() {
    if (file.is_open()) {
        file.close();
    }
    ROS_INFO("Goodbye");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoints_logger");
    ros::NodeHandle nh;

    std::string logFileName = getLogFileName();
    file.open(logFileName.c_str());

    ros::Subscriber sub = nh.subscribe("pf/pose/odom", 1000, saveWaypoint);
    ros::onShutdown(shutdown);

    ROS_INFO("Saving waypoints...");
    ros::spin();

    return 0;
}

