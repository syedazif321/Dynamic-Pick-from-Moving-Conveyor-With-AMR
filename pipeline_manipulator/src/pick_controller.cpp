#include "pipeline_manipulator/pick_controller.hpp"
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread> 

using namespace std::chrono_literals;

namespace pipeline_manipulator
{

DynamicPickController::DynamicPickController(const rclcpp::NodeOptions & options)
    : Node("dynamic_pick_controller", options),
      logger_(get_logger()),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
      tf_listener_(*tf_buffer_),
      control_state_(IDLE)
{
    
    box_state_sub_ = create_subscription<msg_gazebo::msg::BoxState>(
        "/box_state_dynamic", 10,
        std::bind(&DynamicPickController::box_state_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, 
        std::bind(&DynamicPickController::odom_callback, this, std::placeholders::_1));
    
    // 3. Service
    start_service_ = create_service<std_srvs::srv::Trigger>(
        "start_dynamic_pick",
        std::bind(&DynamicPickController::start_pick_service, this, std::placeholders::_1, std::placeholders::_2));
        
    
    control_timer_ = create_wall_timer(
        20ms, 
        std::bind(&DynamicPickController::control_loop, this));

    RCLCPP_INFO(logger_, "DynamicPickController ready. Control loop active at 50Hz.");
}


bool DynamicPickController::initialize_move_group()
{
    RCLCPP_INFO(logger_, "Attempting to initialize MoveGroupInterface...");
    
 
    try {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), arm_group_name_);
            
        move_group_->setPoseReferenceFrame(base_link_frame_);
        move_group_->setPlanningTime(0.05); // Fast planning
        
        RCLCPP_INFO(logger_, "MoveGroupInterface successfully initialized for group: %s.", arm_group_name_.c_str());
        return true;
    } catch(const std::exception& e) {
        RCLCPP_FATAL(logger_, "Failed to create MoveGroupInterface: %s", e.what());
        move_group_.reset();
        return false;
    }
}

void DynamicPickController::box_state_callback(const msg_gazebo::msg::BoxState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_box_state_ = *msg;
}

void DynamicPickController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_odom_ = *msg;
}

bool DynamicPickController::start_pick_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // *** FIX: Initialization moved here, where 'shared_from_this()' is safe. ***
    if (!move_group_ && !initialize_move_group()) {
        RCLCPP_ERROR(logger_, "Service rejected: MoveGroup initialization failed.");
        response->success = false;
        response->message = "MoveGroup failed to initialize.";
        return true;
    }

    if (!move_group_) {
        RCLCPP_ERROR(logger_, "Service rejected: MoveGroupInterface is NULL.");
        response->success = false;
        response->message = "MoveGroup not initialized.";
        return true;
    }
    
    if (control_state_ != IDLE) {
        response->success = false;
        response->message = "System is already busy.";
        return true;
    }
    
    control_state_ = TRACKING;
    RCLCPP_INFO(logger_, "Dynamic pick sequence started. Entering TRACKING state.");
    response->success = true;
    response->message = "Started tracking the moving box.";
    return true;
}


void DynamicPickController::control_loop()
{
    if (control_state_ != TRACKING || !move_group_) return;
    
    msg_gazebo::msg::BoxState box_state;
    nav_msgs::msg::Odometry odom;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        box_state = latest_box_state_;
        odom = latest_odom_;
    }
    
    if (this->now() - box_state.header.stamp > 100ms) {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Box state data is too old.");
        return;
    }

    geometry_msgs::msg::Vector3 v_box_world = box_state.velocity_world.twist.linear;
    geometry_msgs::msg::Vector3 v_base_world = odom.twist.twist.linear;
    
    tf2::Transform T_w_b;
    tf2::fromMsg(odom.pose.pose, T_w_b);
    tf2::Matrix3x3 R_b_w = T_w_b.getBasis().inverse(); // R_b_w is transpose of R_w_b

    tf2::Vector3 v_box_tf(v_box_world.x, v_box_world.y, v_box_world.z);
    tf2::Vector3 v_base_tf(v_base_world.x, v_base_world.y, v_base_world.z);
    
    tf2::Vector3 v_diff_world = v_box_tf - v_base_tf; 
    tf2::Vector3 v_target_base_tf = R_b_w * v_diff_world; // Velocity of Box relative to Base in Base Frame

    geometry_msgs::msg::PoseStamped target_pose_base;
    try {
        tf_buffer_->transform(
            box_state.pose_world, target_pose_base, base_link_frame_, 50ms);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "TF transform failed: %s", ex.what());
        return;
    }
    
    double V_rel_norm = v_target_base_tf.length();
    
    tf2::Vector3 target_pos_base_tf2(
        target_pose_base.pose.position.x, 
        target_pose_base.pose.position.y, 
        target_pose_base.pose.position.z
    );
    double P_rel_norm = target_pos_base_tf2.length(); 

    if (V_rel_norm < GRASP_VELOCITY_TOLERANCE && P_rel_norm < GRASP_POSITION_TOLERANCE) {
        control_state_ = GRASPING;
        RCLCPP_INFO(logger_, "Grasp condition met! V_rel: %.3f, P_rel: %.3f. Initiating GRASP.", V_rel_norm, P_rel_norm);
        initiate_grasp_sequence(target_pose_base.pose);
        return;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose approach_pose = target_pose_base.pose;
    approach_pose.position.z += 0.05; // Approach from slightly above
    waypoints.push_back(approach_pose);
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01; 
    
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, 0.0, trajectory);
    
    if (fraction > 0.8) {
         move_group_->execute(trajectory); 
    } else {
         RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 2000, "Could not compute full Cartesian path (Fraction: %.2f)", fraction);
    }
}

void DynamicPickController::initiate_grasp_sequence(const geometry_msgs::msg::Pose& /*final_pose*/)
{
    RCLCPP_INFO(logger_, "GRASP: Commanding suction ON.");

    control_state_ = RETRACTING;
    RCLCPP_INFO(logger_, "Grasp complete. Retracting arm to home position.");
    
    move_group_->setNamedTarget("home"); 
    moveit::planning_interface::MoveGroupInterface::Plan retract_plan;
    if (move_group_->plan(retract_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_->execute(retract_plan);
    } else {
         RCLCPP_ERROR(logger_, "Failed to plan retract motion!");
    }
    
    control_state_ = IDLE;
    RCLCPP_INFO(logger_, "Pick cycle complete. System IDLE.");
}

} 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<pipeline_manipulator::DynamicPickController>(options);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}