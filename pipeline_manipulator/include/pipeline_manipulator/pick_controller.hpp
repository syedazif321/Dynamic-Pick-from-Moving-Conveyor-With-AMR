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

namespace pipeline_manipulator
{

class DynamicPickController : public rclcpp::Node
{
public:
    explicit DynamicPickController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // State machine definition
    enum State { IDLE, TRACKING, GRASPING, RETRACTING };

    // --- Callbacks and Control Loop ---
    void box_state_callback(const msg_gazebo::msg::BoxState::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_loop(); 
    bool start_pick_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        
    // --- Helper Functions ---
    bool initialize_move_group(); 
    void initiate_grasp_sequence(const geometry_msgs::msg::Pose& final_pose);

    // --- Member Variables ---
    rclcpp::Logger logger_;
    
    // Subscriptions
    rclcpp::Subscription<msg_gazebo::msg::BoxState>::SharedPtr box_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Services and Timer
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // MoveIt and TF
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedPtr attach_client_; 

    // Shared State Data
    msg_gazebo::msg::BoxState latest_box_state_;
    nav_msgs::msg::Odometry latest_odom_;
    State control_state_;
    std::mutex data_mutex_; 
    
    // Configuration members (NOT CONST, will be set in constructor)
    std::string arm_group_name_;
    std::string base_link_frame_;
    
    // Constants (Should remain const)
    const double GRASP_VELOCITY_TOLERANCE = 0.05; // 5 cm/s
    const double GRASP_POSITION_TOLERANCE = 0.40; // This is not used in the final logic, but remains defined.
    const double APPROACH_HEIGHT = 0.10; // 10 cm (The height used for tracking and pre-grasp)
};

} // namespace pipeline_manipulator