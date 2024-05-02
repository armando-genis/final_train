
#include <rclcpp/rclcpp.hpp>

// Ros2
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/msg/marker_array.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/exceptions.h>


// C++
#include <map>
#include <deque>
#include <chrono>


#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>


enum class RobotState {
    IDLE,
    MOVING,
    SCANING,
    BACKHOME
};

using namespace std;

class MoveBaseTrain : public rclcpp::Node
{
private:
    /* data */

    // Robot variables for navigation
    RobotState state_ = RobotState::IDLE;
    size_t current_target_index_ = 0;
    double position_tolerance_ = 0.1;
    double target_x;
    double current_x;

    // vecoor of the id of the arucos
    vector<int64_t> id_arucos_tmp;
    // vector of x of the id_arucos
    vector<double> id_arucos_x_tmp;


    // variables for get the id of the aruco marker
    int id_arucos;
    double id_aruco_x_location;

    // variables for get the cordinate of the aruco marker 
    geometry_msgs::msg::Transform transform_aruco;
    double distance_x_front;
    double distance_y_front;
    double distance_z_front;

    // variables for get the arucos visited
    vector<int> arucos_visited;

    //store the distance of multiples arucos markers
    vector<double> distance_arucos;

    // history of the arucos
    std::map<int, std::deque<bool>> marker_detection_history;
    int history_length = 1; // Number of cycles to remember the detection of an aruco marker in this case is one because we are only interested in one past of the car. 

    int number_of_arucos;

    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    double currentDriftX = 0.0;
    double arucos_pos_x;
    double car_arucos_pos_x;

    //variables for odom
    nav_msgs::msg::Odometry odom_msgs;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // funtions
    void get_aruco_cordinate();
    void get_station_cordinate_visualization();
    void get_odom_cordinate(const double x);
    void get_new_station_tar();
    void kalman_filter();
    void master_control();

    // callback function 
    void callback_id_arucos(const std_msgs::msg::Int32::SharedPtr msg);
    void callback_distance(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_color_int(const std_msgs::msg::String::SharedPtr msg);
    void timer_callback();


    Eigen::Vector2d calculateRobotPosition(const std::vector<double>& marker_x, const std::vector<double>& marker_y, const std::vector<double>& distances_x, const std::vector<double>& distances_y);

    // Publishers and subscriber
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_for_id_arucos;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_for_distance;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_for_color_int;

    // visualization publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_markers_stations;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;

public:
    MoveBaseTrain(/* args */);
    ~MoveBaseTrain();
};

MoveBaseTrain::MoveBaseTrain(/* args */) : Node("move_base_train"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{

    this->declare_parameter("id_arucos", vector<int>() );
    this->declare_parameter("id_arucos_x", vector<float>());
    this->declare_parameter("number_of_arucos", 0);

    
    this->get_parameter("id_arucos", id_arucos_tmp);
    this->get_parameter("id_arucos_x", id_arucos_x_tmp);
    this->get_parameter("number_of_arucos", number_of_arucos);

    // Publisher for odom
    publisher_odom = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Subscriber for /id_arucos
    subscription_for_id_arucos = this->create_subscription<std_msgs::msg::Int32>("/id_arucos", 10, bind(&MoveBaseTrain::callback_id_arucos, this, placeholders::_1));

    // Subscriber for /distance
    subscription_for_distance = this->create_subscription<std_msgs::msg::Float32>("/distance", 10, bind(&MoveBaseTrain::callback_distance, this, placeholders::_1));

    // Subscriber for /color_int
    subscription_for_color_int = this->create_subscription<std_msgs::msg::String>("/color_int", 10, bind(&MoveBaseTrain::callback_color_int, this, placeholders::_1));

    // Timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&MoveBaseTrain::timer_callback, this));

    // Publisher for visualization
    publisher_markers_stations = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_markers_stations", 10);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> move_base_train initialized.\033[0m");

    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->number_of_arucos: %i \033[0m", number_of_arucos);

    for (auto& id : id_arucos_tmp) 
    {
        marker_detection_history[id] = std::deque<bool>(history_length, false);
    }

    
    // distance_arucos.resize(number_of_arucos);


}

MoveBaseTrain::~MoveBaseTrain()
{

}

void MoveBaseTrain::get_aruco_cordinate()
{

    distance_arucos.clear();
    distance_arucos.resize(number_of_arucos);


    std::string toFrameRel = "base_link";
    // std::string fromFrameRel = "aruco_marker";
    std::string fromFrameRel = "aruco_marker_" + std::to_string(id_arucos);

    try {
        // transformStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, this->get_clock()->now(), std::chrono::milliseconds(50)); 

        transform_aruco = tf2_buffer.lookupTransform(toFrameRel, fromFrameRel, this->get_clock()->now(), std::chrono::milliseconds(50)).transform;

        // RCLCPP_INFO(get_logger(), "Transform is available.");

        distance_x_front = transform_aruco.translation.x;
        distance_y_front = transform_aruco.translation.y;
        distance_z_front = transform_aruco.translation.z;

        tf2::Quaternion quat;
        tf2::fromMsg(transform_aruco.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        auto it = find(id_arucos_tmp.begin(), id_arucos_tmp.end(), id_arucos);

        if (it != id_arucos_tmp.end())
        {
            id_aruco_x_location = id_arucos_x_tmp[it - id_arucos_tmp.begin()];
            // RCLCPP_INFO(this->get_logger(), "Index of the aruco marker: %f", id_aruco_x_location);
            auto& history = marker_detection_history[id_arucos];
            history.push_back(true); 
            if (history.size() > history_length){
                history.pop_front();
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Index of the aruco marker: Not found");
            
        }

        arucos_pos_x = id_aruco_x_location + distance_x_front;

        // RCLCPP_INFO(this->get_logger(), "\033[1;35m ----- > Distance of the aruco %i : , %f \033[0m", id_arucos, arucos_pos_x);

        // hanlging multiples arucos markers =======================================

        if(it != id_arucos_tmp.end())
        {
            distance_arucos[it - id_arucos_tmp.begin()] = arucos_pos_x;
        }

        // for (size_t i = 0; i <distance_arucos.size() ; i++)
        // {
        //     RCLCPP_INFO(this->get_logger(), "\033[1;35m ----- > Distance of the arucos %i : , %f \033[0m", i, distance_arucos[i]);
        // }

        // for (auto& [marker_id, history] : marker_detection_history) {
        //     if (marker_id != id_arucos) {
        //         history.push_back(false);
        //         if (history.size() > history_length)
        //             history.pop_front();
        //     }
        // }

    } catch (tf2::TransformException& ex) {
        // RCLCPP_ERROR(this->get_logger(), "Failed to get transform from %s to %s within the time limit: %s",
        //              fromFrameRel.c_str(), toFrameRel.c_str(), ex.what());
    }

    int last_processed_id = -1; 

    // print the marker_detection_history
    for (auto& [marker_id, history] : marker_detection_history) {
        // std::string history_str;
        // for (bool detected : history) {
        //     history_str += detected ? "1" : "0";
        // }
        // RCLCPP_INFO(this->get_logger(), "Marker %d: %s", marker_id, history_str.c_str());
         if (all_of(history.begin(), history.end(), [](bool detected) { return detected; })) {
            last_processed_id = marker_id;
        }
    }

    if (last_processed_id != -1) {
        // RCLCPP_INFO(this->get_logger(), "Last processed marker ID: %d", last_processed_id);
        // get the aruco robot position
        car_arucos_pos_x = distance_arucos[last_processed_id];
    }

}


void MoveBaseTrain::get_station_cordinate_visualization()
{
    visualization_msgs::msg::MarkerArray marker_array;

    for(size_t i = 0; i < id_arucos_x_tmp.size(); ++i)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "odom";
        marker.id = i + 50;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = id_arucos_x_tmp[i];  
        marker.pose.position.y = -1.0; 
        marker.pose.position.z = 0.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 0.6;
        marker.color.r = 0.58;
        marker.color.g = 1.0;
        marker.color.b = 0.2;
        marker_array.markers.push_back(marker);
    }
    publisher_markers_stations->publish(marker_array);
}

void MoveBaseTrain::get_odom_cordinate(const double x)
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = x;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0); 
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    odom_msgs.header.stamp = this->now();
    odom_msgs.header.frame_id = "odom";
    odom_msgs.child_frame_id = "base_link";

    odom_msgs.pose.pose.position.x = x;
    odom_msgs.pose.pose.position.y = 0.0;
    odom_msgs.pose.pose.position.z = 0.0;

    odom_msgs.pose.pose.orientation.x = q.x();
    odom_msgs.pose.pose.orientation.y = q.y();
    odom_msgs.pose.pose.orientation.z = q.z();
    odom_msgs.pose.pose.orientation.w = q.w();

    publisher_odom->publish(odom_msgs);

    tf_broadcaster_->sendTransform(t);
}

void MoveBaseTrain::kalman_filter(){
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> move_base_train initialized.\033[0m");
}

void MoveBaseTrain::get_new_station_tar()
{
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> move_base_train initialized.\033[0m");
}

void MoveBaseTrain::callback_id_arucos(const std_msgs::msg::Int32::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Received /id_arucos: '%d'", msg->data);
    id_arucos = msg->data;
}

void MoveBaseTrain::callback_distance(const std_msgs::msg::Float32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received /distance: '%f'", msg->data);
}

void MoveBaseTrain::callback_color_int(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received /color_int: '%s'", msg->data.c_str());
}



void MoveBaseTrain::timer_callback()
{
    master_control();
    get_aruco_cordinate();
    get_station_cordinate_visualization();
    get_odom_cordinate(car_arucos_pos_x);
}

// ====================================================================================================


void MoveBaseTrain::master_control()
{
    switch (state_)
    {

        case RobotState::IDLE:

            RCLCPP_INFO(this->get_logger(), "\033[1;33m---->RobotState: IDLE \033[0m");

            if(current_target_index_ < id_arucos_x_tmp.size() -1)
            {   
                state_ = RobotState::MOVING;

            }else{
                state_ = RobotState::BACKHOME;
            }
            break;

        case RobotState::MOVING:

            target_x = id_arucos_x_tmp[current_target_index_];
            current_x = odom_msgs.pose.pose.position.x;

            if (abs(target_x - current_x) < position_tolerance_)
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;35m---->Reached target: %i \033[0m", current_target_index_);

                state_ = RobotState::SCANING;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;35m---->Moving to target: %i \033[0m", current_target_index_);
            }

            break;


        case RobotState::SCANING:
                RCLCPP_INFO(this->get_logger(), "\033[1;37m---->RobotState: SCANING \033[0m");
                // if(dej de escanear los productod){
                //     current_target_index_++;
                //     state_ = RobotState::MOVING;
                // }else{  
                //     //Call the services. 
                // }
                current_target_index_++;
                state_ = RobotState::IDLE;

            break;

        case RobotState::BACKHOME:
            current_x = odom_msgs.pose.pose.position.x;
            if(current_x <  0.3){
                RCLCPP_INFO(this->get_logger(), "\033[1;36m---->Arrived Home \033[0m");
                current_target_index_ = 0;
            }

            // condtiino to send negative to the services

            break;

        default:
            // Handle unexpected state
            RCLCPP_ERROR(this->get_logger(), "Unknown state encountered in control logic.");
            break;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveBaseTrain>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

