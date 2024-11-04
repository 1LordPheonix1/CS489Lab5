#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

//using namespace std;
using std::placeholders::_1;


class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
    
    //main variables
    double lookahead_distance = 1.0; // Lookahead distance
    double max_steering_angle = 20.0; // Max steering angle
    double speed = 1.0; // Desired speed
    
    float deg_to_rad(float degree){
            return degree * M_PI / 180.0;
    }
    float rad_to_deg(float rad){
            return rad * 180.0 / M_PI;
    }
    
    //find a way to access the excel file
    
    void read_record(){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        publisher2_->publish(marker);

        // File pointer
        std::fstream fin;
        
        // Open an existing file
        fin.open("/sim_ws/src/pure_pursuit/src/waypoints.csv", std::ios::in);
        RCLCPP_INFO(this->get_logger(), "file opened");

        // Get the roll number
        // of which the data is required
        float x_pos, y_pos, yaw, curr_speed = 0;
        int id = 0;
        
        // Read the Data from the file
        // as String Vector
        std::vector<std::string> row;
        std::string line, word, temp;
        
        while (getline(fin, line)){
            row.clear();
            
            // read an entire row and
            // store it in a string variable 'line'
            
            // used for breaking words
            std::istringstream s(line);
            
            // read every column data of a row and
            // store it in a string variable, 'word'
            while (getline(s, word, ','))
            {
                // add all the column data
                // of a row to a vector
                row.push_back(word);
            }
            
            // convert string to integer for comparison
            x_pos = stof(row[0]);
            y_pos = stof(row[1]);
            yaw = stof(row[2]);
            curr_speed = stof(row[3]);
            visualizer(x_pos, y_pos, id);
            id++;
        }
        fin.close();
        RCLCPP_INFO(this->get_logger(), "done");
    }
    
    
    void visualizer(float x_position, float y_position, int ID){
        RCLCPP_INFO(this->get_logger(), "marker: %d", ID);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp =  this->now();
        marker.ns = "my_namespace";
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
        marker.color.r = (float)ID/100.0;
        marker.color.g = 1.0-((float)ID/100.0);
        marker.color.b = 0.0;
        publisher2_->publish(marker);
    }
    
    /*
     topic options:
     "visualization_marker_array"
     
     Line Strip
     LINE_STRIP=4
     only scale.x is used and it controls the width of the line segments.
     need points set up
     
     visualization_msgs::Marker line_strip;
       48     line_strip.header.frame_id = "/my_frame";
       49     line_strip.header.stamp = this->now();
       50     line_strip.ns = "points_and_lines";
       51     line_strip.action = visualization_msgs::Marker::ADD;
       52     line_strip.pose.orientation.w = 1.0;
     
     
     line_strip.type = visualization_msgs::Marker::LINE_STRIP;
     line_strip.scale.x = 0.1;
     
     // Create the vertices for the points and lines
       93     for (uint32_t i = 0; i < 100; ++i)
       94     {
       95       float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
       96       float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
       97
       98       geometry_msgs::Point p;
       99       p.x = (int32_t)i - 50;
      100       p.y = y;
      101       p.z = z;
      102
      103       points.points.push_back(p);
      104       line_strip.points.push_back(p);
      105
      106       // The line list needs two points for each line
      107       line_list.points.push_back(p);
      108       p.z += 1.0;
      109       line_list.points.push_back(p);
      110     }
     
     
     
    */


    float dist(float x1, float x2, float y1, float y2){
        return std::sqrt(std::pow((x2-x1), 2) + std::pow((y2-y1), 2));
    }
    
    //
    vector<float> find_best_waypoint(float l, float x_pos, float y_pos){
        float x_target = pose_msg->pose.position.x + lookahead_distance;
        float y_target = pose_msg->pose.position.y;
        float min_diff = 100.0;
        int index = 0;
        
        for (int i=0; i < waypoint_data.size(); i++){
            //
            float distance = dist(x_target, waypoint_data[i][0], y_target, waypoint_data[i][1]);
            //float difference = std::fabs(waypoint_data[i][0] - target_pose.position.x);
            if (distance < min_diff){
                min_diff = distance;
                index = i;
            }
        return waypoint_data[index];
    }

public:
    PurePursuit() : Node("pure_pursuit_node"){
        // TODO: create ROS subscribers and publishers
            //add parameters here if needed
        
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        publisher2_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker_array", 1000);
        subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pf/odom/pose", 1000, std::bind(&PurePursuit::pose_callback, this, _1)
        );        ////pf/viz/inferred_pose (maybe)
        RCLCPP_INFO(this->get_logger(), "Reading records");
        read_record();
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        // read_record(); //read and visualize the way_points
        //pose_msg -> pose -> orientation -> x,y,z,w //quaternion (all floats)
       float L = 0.5;
        float steering_angle = 0.0;
        float speed = 0.0;
        float x_pos = 0.0;
        float y_pos = 0.0;
        geometry_msgs::msg::Pose target_pose;
        // TODO: find the current waypoint to track using methods mentioned in lecture
        //use L and try to find the farsthest point within that circle
        // all data is in waypoint_data
        //add transformation here to correct frame if needed
        x_pos = pose_msg->pose.position.x;
        y_pos = pose_msg->pose.position.y;
        
        vector<float> best_vector = find_best_waypoint(L, x_pos, y_pos);
        
    
    
    // Calculate the lookahead target point
        target_pose.position.x = best_vector[1];
        target_pose.position.y = best_vector[2];
        //target_pose.position.x = pose_msg->pose.position.x + lookahead_distance;
        //target_pose.position.y = pose_msg->pose.position.y;

        // Calculate the x and y
        double dx = target_pose.position.x - pose_msg->pose.position.x; //target x - our pose x
        double dy = target_pose.position.y - pose_msg->pose.position.y; //target y - our pose y
        
        
        //this in slide formula (day 17 slide 25) in a sence our triangle to a curve

        double L = std::sqrt(dx * dx + dy * dy); //our L lookahead our (euclidean distance) ex tringle in picture
        double y = std::abs(dy); //we need |dy|
        double r = (L * L) / (2 * y); //our r = ( L^2 ) / 2 |dy|


        double curvature = 1.0 / r; //we calculate our curvature on slide 26

        // Convert curvature to steering angle in degrees
        double steering_angle_deg = std::atan(curvature) * (180.0 / M_PI);
        //clamping steering angle
        steering_angle_deg = std::max(std::min(steering_angle_deg, max_steering_angle), -max_steering_angle);
    
    // TODO: transform goal point to vehicle frame of reference
        

        // TODO: calculate curvature/steering angle
        //curvature of arc = 2|y|/L^2
        
        
        
        
        //larger L more smooth, but more close calls (make L a parameter) or a function of vehicle speed
        
        

        // TODO: publish drive message, don't forget to limit the steering angle.
        ackermann_msgs::msg::AckermannDriveStamped ackermann_drive_result;
        ackermann_drive_result.drive.steering_angle = steering_angle;
        ackermann_drive_result.drive.speed = speed;
        publisher_->publish(ackermann_drive_result);
        
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
