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
    BACKHOME
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

    // Subscriber for /distance
    subscription_for_distance = this->create_subscription<std_msgs::msg::Float64>("/distance", 10, bind(&MoveBaseTrain::callback_distance, this, placeholders::_1)); // Updated to Float64

    // Publisher for num_vueltas_to_station
    num_vueltas_to_station = this->create_publisher<std_msgs::msg::Float64>("/num_vueltas_to_station", 10);

    // Publisher to motor 
    publisher_motor = this->create_publisher<std_msgs::msg::Int32>("/robot_move_none", 10);

    // Publisher for Bool message
    publisher_bool = this->create_publisher<std_msgs::msg::Bool>("/bool_status", 10);

    // Subscriber for Bool message
    subscription_bool = this->create_subscription<std_msgs::msg::Bool>("/bool_input", 10, std::bind(&MoveBaseTrain::callback_bool, this, std::placeholders::_1));

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

    std::string toFrameRel = "base_link";
    std::string fromFrameRel = "aruco_marker_" + std::to_string(id_arucos);

    id_arucos_bool_detection.clear();
    id_arucos_bool_detection.resize(number_of_arucos, 0); 

    try {
        transform_aruco = tf2_buffer.lookupTransform(toFrameRel, fromFrameRel, this->get_clock()->now(), std::chrono::milliseconds(50)).transform;

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

    // print the vector id_arucos_bool_detection
    for (auto& id : id_arucos_bool_detection)
    {
        RCLCPP_INFO(this->get_logger(), "id_arucos_bool_detection: %i", id);
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
    RCLCPP_INFO(this->get_logger(), "Received /distance: '%f'", msg->data);
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

    double filtered_x = kalman_filter.update(motor_x_distance, car_arucos_pos_x, id_arucos_bool_detection[current_target_index_]);

    get_odom_cordinate(filtered_x);
}

void MoveBaseTrain::master_control()
{
    switch (state_)
    {
        case RobotState::IDLE:
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;33m---->RobotState: IDLE \033[0m");

            if(current_target_index_ < id_arucos_x_tmp.size() - 1)
            {   
                state_ = RobotState::MOVING;
            }
            else
            {
                state_ = RobotState::BACKHOME;
            }
            break;
        }

        case RobotState::MOVING:
        {
            target_x = id_arucos_x_tmp[current_target_index_];
            current_x = odom_msgs.pose.pose.position.x;
            double curent_x_motor = motor_x_distance;

            bool aruco_detected = id_arucos_bool_detection[current_target_index_];

            // if the currect target is still not detecting the aruco marker it will keep moving and if it is detected it will send the 30% of the rpms to send the distance to the position control 
            // print the id_arucos_bool_detection

            if (abs(target_x - curent_x_motor) < position_tolerance_)
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;35m---->Moving to target slowly: %i \033[0m", current_target_index_);
                current_shell = current_target_index_;
                current_shell_msg.data = current_shell;
                robot_move_msg.data = 15;

                bool_msg.data = aruco_detected;

                if (aruco_detected)
                {
                    // if the aruco is detected send the 30% of the rpms to the robot
                    RCLCPP_INFO(this->get_logger(), "\033[1;35m---->Aruco target detected: %i \033[0m", current_target_index_);

                    // condition para una vez compleado el control ir al estado de scaning
                    
                    //sent the distace to the position control only once
                    if (!distance_sent)
                    {
                        // this this have to change by the distance fusioneted - the distance of the aruco marker 
                        double distance_to_aruco_marker = distance_arucos[current_target_index_];
                        RCLCPP_INFO(this->get_logger(), "\033[1;35m- DIST SENT \033[0m");
                        distance_sent = true;
                        robot_position_msg.data = distance_to_aruco_marker;
                    }
                }
            }
            else
            {
                // if the aruco is not detected send the 100% of the rpms to the robot

                RCLCPP_INFO(this->get_logger(), "\033[1;35m---->Moving to target: %i \033[0m", current_target_index_);
                robot_move_msg.data = 45;
            }

            if(bool_input_control)
            {
                state_ = RobotState::SCANING;
                bool_msg.data = false;
            }

            current_shell = current_target_index_;
            current_shell_msg.data = current_shell;
            publisher_current_shell->publish(current_shell_msg);

            publisher_motor->publish(robot_move_msg);
            num_vueltas_to_station->publish(robot_position_msg);
            publisher_bool->publish(bool_msg);
            
            
            RCLCPP_INFO(this->get_logger(), "\033[1;35m- HALOO PERROS \033[0m");

            break;
        }
        case RobotState::SCANING:
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;37m---->RobotState: SCANING \033[0m");
            current_target_index_++;
            state_ = RobotState::IDLE;
            distance_sent = false;
            break;
        }
        case RobotState::BACKHOME:
        {
            current_x = odom_msgs.pose.pose.position.x;
            if(current_x < 0.3)
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;36m---->Arrived Home \033[0m");
                current_target_index_ = 0;
                current_shell = 0;
            }
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
