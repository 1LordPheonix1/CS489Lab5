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
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher2_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher3_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber2_;
    
    //main variables
    double lookahead_distance = 1.0; // Lookahead distance
    float max_steering_angle = 20.0; // Max steering angle
    double speed = 1.0; // Desired speed

    float angle_range = deg_to_rad(120.0);

    std::vector<float> last_best_point;

    // sim pose topic
    std::string sim_post_topic = "/ego_racecar/odom";

    // waypoint data
    std::vector<std::vector<float>> waypoint_data;
    
    float deg_to_rad(float degree){
            return degree * M_PI / 180.0;
    }
    float rad_to_deg(float rad){
            return rad * 180.0 / M_PI;
    }
    
    //find a way to access the excel file
    
    void read_record(){
        visualization_msgs::msg::MarkerArray MarkerArray;
        // marker.header.frame_id = "map";
        // marker.action = visualization_msgs::msg::Marker::DELETEALL;
        // publisher2_->publish(marker);

        // File pointer
        std::fstream fin;
        
        // Open an existing file
        fin.open("/sim_ws/src/pure_pursuit/src/waypoints.csv", std::ios::in);
        RCLCPP_INFO(this->get_logger(), "file opened: %d", fin.is_open());

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

            MarkerArray.markers.push_back(visualizer(x_pos, y_pos, id));
            waypoint_data.push_back({x_pos,y_pos,yaw,curr_speed});
            id++;
        }
        fin.close();
        publisher2_->publish(MarkerArray);
        RCLCPP_INFO(this->get_logger(), "done");
    }
    
    
    visualization_msgs::msg::Marker visualizer(float x_position, float y_position, int ID){
        // RCLCPP_INFO(this->get_logger(), "marker: %d", ID);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp =  this->now();
        if(ID == 1000) {
            marker.ns = "target";
        } else {
            marker.ns = "waypoints";
        }
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
        if(ID == 1000) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        return marker;
        // publisher2_->publish(marker);
    }


    float dist(float x1, float y1, float x2, float y2){
        return std::sqrt(std::pow((x2-x1), 2) + std::pow((y2-y1), 2));
    }

    // given 2 points
    std::vector<float> interpolate_onto_circle(float l, float x_pos, float y_pos, float x1, float y1, float x2, float y2, float d1, float d2) {
        // l is lookahead we want point for
        // x_pos, y_pos is our location (p0)
        // x1,y1 is waypoint with dist() <= l -> d1 (p1)
        // x2,y2 is waypoint wiht dist() >= l -> d2 (p2)
        
        // get d -> dist((x1,y1) to (x2,y2))
        float d = dist(x1,y1,x2,y2);
        
        // let Theta be angle between p2,p1,p0 - cosine rule
        float theta = std::acos((d*d + d1*d1 - d2*d2)/(2*d*d1));

        // let alpha be angle between p1,p2,p0 - sine rule
        float alpha = std::asin(d1*std::sin(theta)/l);

        // let beta be angle between p1,p0,p2 = pi - alpha - theta
        float beta = M_PI - theta - alpha;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "d: %f, theta: %f, alpha: %f, beta: %f", d, theta, alpha, beta);

        // then let z = dist(p1,p2) -> sine rule
        float z  = std::sin(beta)*d1/std::sin(alpha);

        // slope of line (p1,p2) easily determined
        float m = (y2-y1)/(x2-x1);

        // get magnitude of line <1,m> = sqrt(1 + m^2)
        float mag = std::sqrt(1.0+m*m);

        // parametrization vector is then <1/mag, m/mag>
        std::vector<float> N = {(float)1.0/mag, m/mag};

        // So our choices are f(z), we want it to fall between point
        float target_x = z*N[0] + x1;
        float target_y = z*N[1] + y1;

        return {target_x, target_y};
    }

    // given 2 points
    std::vector<float> interpolate_onto_circle_cartesian(float l, float x_pos, float y_pos, float x1, float y1, float x2, float y2, float d1, float d2) {
        // l is lookahead we want point for
        // x_pos, y_pos is our location (p0)
        // x1,y1 is waypoint with dist() <= l -> d1 (p1)
        // x2,y2 is waypoint wiht dist() >= l -> d2 (p2)
        
        // get m is slope (line p1->p2)
        float m = (y2-y1)/(x2-x1);

        // in quadratic formula, a = (m^2+1)
        float a = m*m+1.0;

        // b -> 2m(y1-y0) -2m^2x1 - 2x0
        float b = 2.0*m*(y1-y_pos) - 2.0*m*m*x1 - 2.0*x_pos;

        // c -> x0^2 + m^2x1^2 -2mx1(y1-y0) + (y1-y0)^2 - l^2
        float c = x_pos*x_pos + m*m*x1*x1 - 2.0*m*x1*(y1-y_pos) + (y1-y_pos)*(y1-y_pos) - l*l;

        // xp = quadratic formula with a,b,c
        float deter = std::sqrt(b*b - 4.0*a*c);
        std::vector<float> xp = {(-b-deter)/((float)2.0*a),(-b+deter)/((float)2.0*a)};
        std::vector<float> yp = {y1+m*(xp[0]-x1),y1+m*(xp[1]-x1)};
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "point %d: %f, %f", 0, xp[0], yp[0]);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "point %d: %f, %f", 1, xp[1], yp[1]);

        // correct choice of xp will be the fall that is between the points
        float target_x = xp[1];
        float target_y = yp[1];
        if(xp[0] >= std::min(x1,x2) && xp[0] <= std::max(x1,x2) && yp[0] >= std::min(y1,y2) && yp[0] <= std::max(y1,y2)) {
            // point found, otherwise other choice
            target_x = xp[0];
            target_y = yp[0];
        }
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "chosen point: %f, %f", target_x, target_y);

        return {target_x, target_y};
    }
    
    //
    std::vector<float> find_best_waypoint(float l, float x_pos, float y_pos, float yaw){
        // chance of waypoint being exactly l away is unlikely
        // find 2 waypoints, one under l, one just over l, and interpolate
        int index1 = 0;
        int index2 = 0;
        tf2::Vector3 point1;
        tf2::Vector3 point2;

        // we want closest points to (x_pos+l, y_pos), and distance to (x_pos, y_pos) be as close to l

        float dist1_l = 100.0;
        float dist1_pos = 0.0;
        float dist2_l = 100.0;
        float dist2_pos = 100.0;

        bool reverse = false;
        for (int i=0; i < waypoint_data.size(); i++){
            // only consider points within +/- angle_range of current yaw
            float data_yaw = waypoint_data[i][2];
            
            if(data_yaw >= yaw - angle_range && data_yaw <= yaw + angle_range) {
                tf2::Matrix3x3 rotation(std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0, 1);
                rotation = rotation.inverse();

                // distance of car frame w.r.t. map frame
                tf2::Vector3 translation(waypoint_data[i][0]-x_pos, waypoint_data[i][1]-y_pos, 0);

                tf2::Vector3 output = rotation*translation;                
                
                // distance 1 is in robot frame
                // float distance = dist(l, 0, output.getX(), output.getY());
                float distance = dist(l, 0, output.getX(), output.getY());

                // RCLCPP_INFO(this->get_logger(), "map: x: %f, y: %f, car: x: %f, y: %f, z: %f", waypoint_data[i][0], waypoint_data[i][1],output.getX(), output.getY(), output.getZ());

                // distance 2
                float distance2 = dist(x_pos, y_pos, waypoint_data[i][0], waypoint_data[i][1]);

                // test for waypoint below l
                if(distance <= dist1_l) {
                    if (distance2 <= l){
                        dist1_pos = distance2;
                        dist1_l = distance;
                        index1 = i;
                        point1 = output;
                    }
                }

                // test for waypoint above l
                if(distance <= dist2_l) {
                    if(distance2 >= l){
                        dist2_l = distance;
                        dist2_pos = distance2;
                        index2 = i;
                        point2 = output;
                    }
                }
            }
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "x_pos: %f, y_pos: %f, yaw: %f, x1: %f, y1: %f, x2: %f, y2: %f, d1: %f, d2: %f", x_pos, y_pos, yaw, waypoint_data[index1][0], waypoint_data[index1][1], waypoint_data[index2][0], waypoint_data[index2][1], dist1_pos, dist2_pos);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "point1: x: %f, y: %f, z: %f",point1.getX(), point1.getY(), point1.getZ());
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "point2: x: %f, y: %f, z: %f",point2.getX(), point2.getY(), point2.getZ());

        std::vector<float> point = interpolate_onto_circle_cartesian(l, x_pos, y_pos, waypoint_data[index1][0], waypoint_data[index1][1], waypoint_data[index2][0], waypoint_data[index2][1], dist1_pos, dist2_pos);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "point found: %f, %f", point[0], point[1]);
        visualization_msgs::msg::Marker marker = visualizer(point[0], point[1], 1000);
        publisher3_->publish(marker);
        if(isnan(point[0]) || isnan(point[1])) {
            return last_best_point;
        }
        last_best_point = point;

        // speed is avg of the 2 waypoints
        float speed = (waypoint_data[index1][3]+waypoint_data[index2][3])/2.0;

        return {point[0], point[1], speed};
    }

public:
    PurePursuit() : Node("pure_pursuit_node"){
        // TODO: create ROS subscribers and publishers
            //add parameters here if needed
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        publisher2_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array_array", 100);
        publisher3_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker_array", 100);
        RCLCPP_INFO(this->get_logger(), "Reading records");
        read_record();
        RCLCPP_INFO(this->get_logger(), "waypoints size: %d", waypoint_data.size());
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 1000, std::bind(&PurePursuit::pose_callback, this, _1)
        );        ////pf/viz/inferred_pose (maybe)
        // subscriber2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "/pf/viz/inferred_pose", 1000, std::bind(&PurePursuit::pose2_callback, this, _1)
        // ); 
    }

    void pose2_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "pose2 x: %f, y: %f", pose_msg->pose.position.x, pose_msg->pose.position.y);
    }

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
    {
        //pose_msg -> pose -> orientation -> x,y,z,w //quaternion (all floats)
        // float L = 1.0;
        float x_pos = 0.0;
        float y_pos = 0.0;

        // TODO: find the current waypoint to track using methods mentioned in lecture
        //use L and try to find the farsthest point within that circle
        // all data is in waypoint_data
        x_pos = pose_msg->pose.pose.position.x;
        y_pos = pose_msg->pose.pose.position.y;

        // Extract quaternion
        double qx = pose_msg->pose.pose.orientation.x;
        double qy = pose_msg->pose.pose.orientation.y;
        double qz = pose_msg->pose.pose.orientation.z;
        double qw = pose_msg->pose.pose.orientation.w;

        // Convert quaternion to Euler angles
        tf2::Quaternion quat(qx, qy, qz, qw);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);


        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "pose: x: %f, y: %f, yaw: %f", pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, yaw);
        
        std::vector<float> best_vector = find_best_waypoint(lookahead_distance, x_pos, y_pos, yaw);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "Best waypoint x: %f, y: %f", best_vector[0], best_vector[1]);

        // robot frame is in ego_racecar/base_link
        // position is in map frame 
        // orientation is in robot frame (0,0,0,1)
        // to convert, need to translate robot pos to point pos
        // then rotate by yaw, to get dx,dy
        
        tf2::Matrix3x3 rotation(std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0, 1);
        rotation = rotation.inverse();

        // distance of car frame w.r.t. map frame
        tf2::Vector3 translation(best_vector[0]-x_pos, best_vector[1]-y_pos, 0);

        tf2::Vector3 output = rotation*translation; 
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "x: %f, y: %f, z: %f", output.getX(), output.getY(), output.getZ());

        // Calculate the x and y
        double dx = output.getX(); //target x - our pose x
        double dy = output.getY(); //target y - our pose y
        
        
        //this in slide formula (day 17 slide 25) in a sence our triangle to a curve

        double l = std::sqrt(dx * dx + dy * dy); //our L lookahead our (euclidean distance) ex tringle in picture
        double y = std::abs(dy); //we need |dy|
        double r = (l * l) / (2 * y); //our r = ( L^2 ) / 2 |dy|

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "dx: %f, dy: %f, l: %f", dx, dy, l);

        // r = C/2sin(angle) -> angle = asin(C/2r)

        float steering_angle = std::asin(l/((float)2.0*r));

        // assume going right
        bool left = (dy > 0) ? true : false;
        if(!left) {
            steering_angle *= -1;
        }

        //clamping steering angle
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "angle: %f, radius: %f, speed: %f", rad_to_deg(steering_angle), r, speed);

        steering_angle = std::max(std::min(steering_angle, deg_to_rad(max_steering_angle)), -deg_to_rad(max_steering_angle));        
        //larger L more smooth, but more close calls (make L a parameter) or a function of vehicle speed
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "angle: %f, radius: %f, speed: %f", rad_to_deg(steering_angle), r, speed);
        

        // TODO: publish drive message, don't forget to limit the steering angle.
        ackermann_msgs::msg::AckermannDriveStamped ackermann_drive_result;
        ackermann_drive_result.drive.steering_angle = steering_angle;
        // ackermann_drive_result.drive.speed = best_vector[2];
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
