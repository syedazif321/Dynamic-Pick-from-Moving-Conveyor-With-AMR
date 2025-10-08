#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include "msg_gazebo/msg/box_state.hpp" 
#include "msg_gazebo/srv/attach_detach.hpp"
#include <mutex> 
#include <thread>
#include <atomic> // Must be included for std::atomic

namespace pipeline_manipulator
{

class DynamicPickController : public rclcpp::Node
{
public:
    explicit DynamicPickController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // State machine definition
    enum State { 
        IDLE, 
        INITIAL_MOVE, 
        READY_TO_PICK,
        TRACKING,     
        GRASPING,      
        RETRACTING,    
        PICK_IN_PROGRESS
    };

    // --- Callbacks and Control Loop ---
    void box_state_callback(const msg_gazebo::msg::BoxState::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void check_setup_and_start_initial_move(); 
    void control_loop(); 
    
    // --- Helper Functions ---
    bool initialize_move_group(); 
    bool execute_initial_move(); 
    void execute_pick_sequence(const geometry_msgs::msg::Pose& target_pose);

    // --- Member Variables ---
    rclcpp::Logger logger_;
    
    // Subscriptions
    rclcpp::Subscription<msg_gazebo::msg::BoxState>::SharedPtr box_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Services and Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // MoveIt and TF
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedPtr attach_client_; 

    // Shared State Data
    msg_gazebo::msg::BoxState latest_box_state_;
    nav_msgs::msg::Odometry latest_odom_;
    std::atomic<State> control_state_; // std::atomic for thread-safe state
    std::mutex data_mutex_; 
    
    bool setup_complete_ = false; 
    
    // Configuration members
    std::string arm_group_name_;
    std::string base_link_frame_;
};

} // namespace pipeline_manipulator