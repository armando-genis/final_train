#include <rclcpp/rclcpp.hpp>

// Ros2
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
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
#include "std_msgs/msg/bool.hpp"

// C++
#include <map>
#include <deque>
#include <chrono>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>

#include "kalman_filter.hpp"

enum class RobotState {
    IDLE,
    MOVING,
    SCANING,
    BACKHOME,
    STAY
};

using namespace std;

class MoveBaseTrain : public rclcpp::Node
{
private:
    KalmanFilter kalman_filter;

    // Robot variables for navigation
    RobotState state_ = RobotState::IDLE;
    size_t current_target_index_ = 0;
    double position_tolerance_ = 0.3;
    double target_x;
    double current_x;
    double motor_x_distance = 0.0;

    // vector of the id of the arucos
    vector<int64_t> id_arucos_tmp;
    // vector of x of the id_arucos
    vector<double> id_arucos_x_tmp;

    vector<int64_t> id_arucos_bool_detection;

    // variables for get the id of the aruco marker
    int id_arucos;
    double id_aruco_x_location;

    // variables for get the coordinate of the aruco marker 
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
    bool market_fist_detection = false;
    bool distance_sent = false;
    bool final_distance_sent = false;
    bool bool_input_control = false;

    int number_of_arucos;

    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    double currentDriftX = 0.0;
    double arucos_pos_x;
    double car_arucos_pos_x;

    //variables for odom
    nav_msgs::msg::Odometry odom_msgs;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // current shell
    int current_shell = 0;

    std_msgs::msg::Int32 current_shell_msg;
    // robot move 
    std_msgs::msg::Int32 robot_move_msg;
    std_msgs::msg::Float64 robot_position_msg;
    std_msgs::msg::Bool bool_msg;
    bool any_aruco_detected = false;


    int down_count = 50;

    // functions
    void get_aruco_cordinate();
    void get_station_cordinate_visualization();
    void get_odom_cordinate(const double x);
    void get_new_station_tar();
    void master_control();

    // callback function 
    void callback_id_arucos(const std_msgs::msg::Int32::SharedPtr msg);
    void callback_distance(const std_msgs::msg::Float64::SharedPtr msg); // Updated to Float64
    void timer_callback();
    void callback_bool(const std_msgs::msg::Bool::SharedPtr msg);

    // Publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_for_id_arucos;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_for_distance; // Updated to Float64
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr num_vueltas_to_station;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_motor;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_bool;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_bool;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_current_shell;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_markers_stations;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    MoveBaseTrain(/* args */);
    ~MoveBaseTrain();
};

MoveBaseTrain::MoveBaseTrain(/* args */) : Node("move_base_train"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer), kalman_filter(0.1, 0.1, 0.1)
{
    this->declare_parameter("id_arucos", vector<int>());
    this->declare_parameter("id_arucos_x", vector<float>());
    this->declare_parameter("number_of_arucos", 0);

    this->get_parameter("id_arucos", id_arucos_tmp);
    this->get_parameter("id_arucos_x", id_arucos_x_tmp);
    this->get_parameter("number_of_arucos", number_of_arucos);

    // Publisher for odom
    publisher_odom = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // publisher for current shell 
    publisher_current_shell = this->create_publisher<std_msgs::msg::Int32>("/current_shell", 10);

    // Subscriber for /id_arucos
    subscription_for_id_arucos = this->create_subscription<std_msgs::msg::Int32>("/id_arucos", 10, bind(&MoveBaseTrain::callback_id_arucos, this, placeholders::_1));


// =================================================================================    

    // Subscriber for /distance
    subscription_for_distance = this->create_subscription<std_msgs::msg::Float64>("/distance", 10, bind(&MoveBaseTrain::callback_distance, this, placeholders::_1)); // Updated to Float64

    // Publisher for num_vueltas_to_station
    num_vueltas_to_station = this->create_publisher<std_msgs::msg::Float64>("/go_to", 10);

    // Publisher to motor 
    publisher_motor = this->create_publisher<std_msgs::msg::Int32>("/rpm", 10);

    // Publisher for Bool message
    publisher_bool = this->create_publisher<std_msgs::msg::Bool>("/aruco_reached", 10);

    // Subscriber for Bool message
    subscription_bool = this->create_subscription<std_msgs::msg::Bool>("/pos_reached", 10, std::bind(&MoveBaseTrain::callback_bool, this, std::placeholders::_1));


// =================================================================================

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

    kalman_filter.initialize(0.0, 1.0);
}

MoveBaseTrain::~MoveBaseTrain()
{
}

void MoveBaseTrain::get_aruco_cordinate()
{
    distance_arucos.clear();
    distance_arucos.resize(number_of_arucos);

    std::string toFrameRel = "camera_link";
    std::string fromFrameRel = "aruco_marker_" + std::to_string(id_arucos);

    id_arucos_bool_detection.clear();
    id_arucos_bool_detection.resize(number_of_arucos, 0); 

    try {
        transform_aruco = tf2_buffer.lookupTransform(toFrameRel, fromFrameRel, this->get_clock()->now(), std::chrono::milliseconds(50)).transform;

        distance_x_front = transform_aruco.translation.x - 0.04;
        distance_y_front = transform_aruco.translation.y;
        distance_z_front = transform_aruco.translation.z;

        // RCLCPP_INFO(this->get_logger(), "distance_x_front: %f", distance_x_front);
        // RCLCPP_INFO(this->get_logger(), "distance_y_front: %f", distance_y_front);
        // RCLCPP_INFO(this->get_logger(), "distance_z_front: %f", distance_z_front);

        tf2::Quaternion quat;
        tf2::fromMsg(transform_aruco.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        auto it = find(id_arucos_tmp.begin(), id_arucos_tmp.end(), id_arucos);

        if (it != id_arucos_tmp.end())
        {
            id_aruco_x_location = id_arucos_x_tmp[it - id_arucos_tmp.begin()];

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

        arucos_pos_x = id_aruco_x_location - distance_x_front;

        // print the arucos_pos_x in green 
        // RCLCPP_INFO(this->get_logger(), "\033[1;32m---->arucos_pos_x: %f \033[0m", arucos_pos_x);

        

        // handling multiples arucos markers
        if(it != id_arucos_tmp.end())
        {
            distance_arucos[it - id_arucos_tmp.begin()] = arucos_pos_x;
        }

    } catch (tf2::TransformException& ex) {
        // RCLCPP_ERROR(this->get_logger(), "Failed to get transform from %s to %s within the time limit: %s",
        //              fromFrameRel.c_str(), toFrameRel.c_str(), ex.what());
    }

    int last_processed_id = -1; 

    // print the marker_detection_history 
    
    int index = 0;
    for (auto& [marker_id, history] : marker_detection_history) {
         if (all_of(history.begin(), history.end(), [](bool detected) { return detected; })) {
            last_processed_id = marker_id;
            id_arucos_bool_detection[index] = 1;
        }
        index++;
    }


    
    car_arucos_pos_x = distance_arucos[last_processed_id];
    // RCLCPP_INFO(this->get_logger(), "\033[1;33m---->car_arucos_pos_x: %f \033[0m", car_arucos_pos_x);


    // print the vector id_arucos_bool_detection
    // for (auto& id : id_arucos_bool_detection)
    // {
    //     RCLCPP_INFO(this->get_logger(), "id_arucos_bool_detection: %i", id);
    // }
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

void MoveBaseTrain::get_new_station_tar()
{
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> move_base_train initialized.\033[0m");
}

void MoveBaseTrain::callback_id_arucos(const std_msgs::msg::Int32::SharedPtr msg)
{
    id_arucos = msg->data;
}

void MoveBaseTrain::callback_distance(const std_msgs::msg::Float64::SharedPtr msg) // Updated to Float64
{
    // RCLCPP_INFO(this->get_logger(), "Received /distance: '%f'", msg->data);
    motor_x_distance = msg->data;
}

void MoveBaseTrain::callback_bool(const std_msgs::msg::Bool::SharedPtr msg) {
    bool_input_control = msg->data;
}

void MoveBaseTrain::timer_callback()
{
    master_control();
    get_aruco_cordinate();
    get_station_cordinate_visualization();

    if (id_arucos != -1)
    {
        any_aruco_detected = true;
    }
    else
    {
        any_aruco_detected = false;
    }

    double filtered_x = kalman_filter.update(motor_x_distance, car_arucos_pos_x, any_aruco_detected);

    // print the filtered_x in red
    // RCLCPP_INFO(this->get_logger(), "\033[1;33m---->filtered_x: %f \033[0m", filtered_x);

    get_odom_cordinate(filtered_x);
}

void MoveBaseTrain::master_control()
{
    switch (state_)
    {
        case RobotState::IDLE:
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;33m---->RobotState: IDLE \033[0m");
            down_count = 50;

            if(current_target_index_ < id_arucos_x_tmp.size() - 1)
            {   
                RCLCPP_INFO(this->get_logger(), "\033[1;33m---------->RobotState: Enter to Moving \033[0m");
                bool_msg.data = false;
                publisher_bool->publish(bool_msg);
                state_ = RobotState::MOVING;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;33m---------->RobotState: Enter to BACKHOME \033[0m");

                state_ = RobotState::BACKHOME;
            }
            break;
        }

        case RobotState::MOVING:
        {


            target_x = id_arucos_x_tmp[current_target_index_];
            current_x = odom_msgs.pose.pose.position.x;
            double current_x_motor = motor_x_distance;

            for (auto& id : id_arucos_bool_detection)
            {
                RCLCPP_INFO(this->get_logger(), "id_arucos_bool_detection: %i", id);
            }

            bool aruco_detected = id_arucos_bool_detection[current_target_index_];

            // Log the detection status and position information
            RCLCPP_INFO(this->get_logger(), "\033[1;34m---->id_arucos_bool_detection: %i \033[0m", aruco_detected);
            // RCLCPP_INFO(this->get_logger(), "\033[1;31m---->abs(target_x - current_x_motor) < position_tolerance_: %i \033[0m", abs(target_x - current_x) < position_tolerance_);
            // print distace_sent
            RCLCPP_INFO(this->get_logger(), "\033[1;31m------------------>distance_sent: %i \033[0m", distance_sent);

            if (aruco_detected)
            {
                // RCLCPP_INFO(this->get_logger(), "\033[1;35m---->Aruco target detected: %i \033[0m", current_target_index_);
                // print the distance of the aruco marker
                RCLCPP_INFO(this->get_logger(), "\033[1;32m---->distance_arucos: %f \033[0m", distance_arucos[current_target_index_]);

                

                if (!distance_sent)
                {
                    // double distance_to_aruco_marker = distance_arucos[current_target_index_];
                    double distance_to_aruco_marker = id_arucos_x_tmp[current_target_index_];


                    RCLCPP_INFO(this->get_logger(), "\033[1;32m- ---> DIST SENT =================================== \033[0m");
                    RCLCPP_INFO(this->get_logger(), "\033[1;32m- ---> DIST SENT: %f \033[0m", distance_to_aruco_marker);

                    distance_sent = true;
                    bool_msg.data = true;
                    robot_position_msg.data = distance_to_aruco_marker;
                    num_vueltas_to_station->publish(robot_position_msg);  // Send the position message
                    publisher_bool->publish(bool_msg);  // Send the bool message
                }

                RCLCPP_INFO(this->get_logger(), "\033[1;32m---->num_vueltas_to_station: %f \033[0m", robot_position_msg.data);
                RCLCPP_INFO(this->get_logger(), "\033[1;32m---->publisher_bool: %i \033[0m", bool_msg.data);

            }

            
            else if(!distance_sent)
            {
                if (abs(target_x - current_x) < position_tolerance_)
                {
                    RCLCPP_INFO(this->get_logger(), "\033[1;35m---->Moving to target slowly: %i \033[0m", current_target_index_);
                    current_shell = current_target_index_;
                    current_shell_msg.data = current_shell;
                    robot_move_msg.data = 25;  // Slow down
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "\033[1;35m---->Moving to target (setting 30 rpm): %i \033[0m", current_target_index_);
                    robot_move_msg.data = 35;  // Regular speed
                }

                publisher_motor->publish(robot_move_msg);  // Send the RPM command

                // RCLCPP_INFO(this->get_logger(), "\033[1;35m---->publisher_motor: %i \033[0m", robot_move_msg.data);
            }


            if (bool_input_control)
            {
                state_ = RobotState::SCANING;
            }

            current_shell = current_target_index_;
            current_shell_msg.data = current_shell;
            publisher_current_shell->publish(current_shell_msg);

            // Log the published data

            

            RCLCPP_INFO(this->get_logger(), "\033[1;31m------------------------> All data sent \033[0m");

            break;
        }

        case RobotState::SCANING:
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;37m---->RobotState: SCANING \033[0m");
            
            if(down_count == 0)
            {
                distance_sent = false;
                bool_msg.data = false;
                current_target_index_++;
                bool_msg.data = false;
                publisher_bool->publish(bool_msg);
                state_ = RobotState::IDLE;
            }

            down_count--;


            break;

        }
        case RobotState::BACKHOME:
        {

            if (!final_distance_sent)
            {
                final_distance_sent = true;
                bool_msg.data = true;
                robot_position_msg.data = 0.0;
                num_vueltas_to_station->publish(robot_position_msg);  // Send the position message
                publisher_bool->publish(bool_msg);  // Send the bool message
                RCLCPP_INFO(this->get_logger(), "\033[1;32m---->position send: %f \033[0m", robot_position_msg.data);
            }  
        
            if(odom_msgs.pose.pose.position.x < 0.5)
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;36m---->Arrived Home \033[0m");
                current_target_index_ = 0;
                current_shell = 0;

                if (bool_input_control)
                {
                    state_ = RobotState::STAY;
                }
            }


            break;
        }

        case RobotState::STAY:
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;36m---->RobotState: STAY \033[0m");

            if(down_count == 0)
            {
                distance_sent = false;
                bool_msg.data = false;
                final_distance_sent = false;
                publisher_bool->publish(bool_msg);
                // reset to ccero id_arucos_bool_detection
                id_arucos_bool_detection.clear();
                id_arucos_bool_detection.resize(number_of_arucos, 0);
                // print the vector id_arucos_bool_detection

                // clear the marker_detection_history
                for (auto& [marker_id, history] : marker_detection_history) {
                    fill(history.begin(), history.end(), false);
                }
                for (auto& id : id_arucos_bool_detection)
                {
                    RCLCPP_INFO(this->get_logger(), "id_arucos_bool_detection: %i", id);
                }
                state_ = RobotState::IDLE;
            }

            down_count--;

            break;
        }
        default:
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown state encountered in control logic.");
            break;
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveBaseTrain>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
