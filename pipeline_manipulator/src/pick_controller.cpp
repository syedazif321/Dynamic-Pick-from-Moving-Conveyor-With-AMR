#include "pipeline_manipulator/pick_controller.hpp"
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread> 
#include "msg_gazebo/srv/attach_detach.hpp" 
#include <chrono>
#include <cmath> 
#include "rclcpp/executors/single_threaded_executor.hpp" 
#include "rclcpp/node.hpp" 

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
    this->declare_parameter("arm_group_name", "rm_group");
    this->declare_parameter("base_link_frame", "base_link");

    this->get_parameter("arm_group_name", arm_group_name_);
    this->get_parameter("base_link_frame", base_link_frame_);

    box_state_sub_ = create_subscription<msg_gazebo::msg::BoxState>(
        "/box_state_dynamic", 10,
        std::bind(&DynamicPickController::box_state_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, 
        std::bind(&DynamicPickController::odom_callback, this, std::placeholders::_1));
    
    // 3. Services
    start_service_ = create_service<std_srvs::srv::Trigger>(
        "start_dynamic_pick",
        std::bind(&DynamicPickController::start_pick_service, this, std::placeholders::_1, std::placeholders::_2));
        
    attach_client_ = create_client<msg_gazebo::srv::AttachDetach>("/AttachDetach");
    
    control_timer_ = create_wall_timer(
        50ms, 
        std::bind(&DynamicPickController::control_loop, this));

    RCLCPP_INFO(logger_, "DynamicPickController ready. Tracking loop active at 20Hz.");
}

bool DynamicPickController::initialize_move_group()
{
    RCLCPP_INFO(logger_, "Attempting to initialize MoveGroupInterface...");
    
    if (move_group_) return true;

    try {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), arm_group_name_);
            
        move_group_->setPoseReferenceFrame(base_link_frame_);
        move_group_->setPlanningTime(0.5); 
        move_group_->setMaxVelocityScalingFactor(0.8);
        move_group_->setMaxAccelerationScalingFactor(0.8);
        
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
    
    if (!initialize_move_group()) {
        RCLCPP_ERROR(logger_, "Service rejected: MoveGroup initialization failed.");
        response->success = false;
        response->message = "MoveGroup failed to initialize.";
        return true;
    }
    
    if (!attach_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(logger_, "Service rejected: /AttachDetach service not available.");
        response->success = false;
        response->message = "/AttachDetach service not available.";
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
    
    if (this->now() - box_state.header.stamp > 200ms) {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Box state data is too old. Skipping loop iteration.");
        return;
    }

    geometry_msgs::msg::Vector3 v_base_world = odom.twist.twist.linear;
    tf2::Transform T_w_b;
    tf2::fromMsg(odom.pose.pose, T_w_b);
    tf2::Matrix3x3 R_b_w = T_w_b.getBasis().inverse(); 

    tf2::Vector3 v_box_tf(box_state.velocity_world.twist.linear.x, 
                         box_state.velocity_world.twist.linear.y, 
                         box_state.velocity_world.twist.linear.z);
    
    tf2::Vector3 v_base_tf(v_base_world.x, v_base_world.y, v_base_world.z);
    tf2::Vector3 v_diff_world = v_box_tf - v_base_tf; 
    tf2::Vector3 v_target_base_tf = R_b_w * v_diff_world; 

    geometry_msgs::msg::PoseStamped target_pose_base;
    try {
        tf_buffer_->transform(
            box_state.pose_world, target_pose_base, base_link_frame_, 200ms);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "TF transform failed: %s", ex.what());
        return;
    }
    

    double V_rel_norm = v_target_base_tf.length();

    geometry_msgs::msg::Pose desired_approach_pose = target_pose_base.pose;
    desired_approach_pose.position.z += APPROACH_HEIGHT; 

    geometry_msgs::msg::Pose current_eef_pose = move_group_->getCurrentPose().pose;
    
    tf2::Vector3 current_eef_pos(current_eef_pose.position.x, current_eef_pose.position.y, current_eef_pose.position.z);
    tf2::Vector3 desired_pos(desired_approach_pose.position.x, desired_approach_pose.position.y, desired_approach_pose.position.z);
    
    tf2::Vector3 error_vector = desired_pos - current_eef_pos;
    double alignment_error = error_vector.length();

    const double ALIGNMENT_TOLERANCE_TOTAL = 0.20; // 5 cm total error

    if (V_rel_norm < GRASP_VELOCITY_TOLERANCE && 
        alignment_error < ALIGNMENT_TOLERANCE_TOTAL) {
        
        control_state_ = GRASPING;
        RCLCPP_INFO(logger_, "Grasp condition met! V_rel: %.3f, Alignment Error: %.3f. Initiating GRASP.", V_rel_norm, alignment_error);

        initiate_grasp_sequence(target_pose_base.pose);
        return;
    }

    
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(desired_approach_pose); // Keep tracking the approach pose
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01; 
    
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, 0.0, trajectory);
    
    if (fraction > 0.5) {
         move_group_->execute(trajectory); 
    } else {
         RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 2000, "Could not compute sufficient Cartesian path (Fraction: %.2f).", fraction);
    }
}

// initiate_grasp_sequence (FIXED)
void DynamicPickController::initiate_grasp_sequence(const geometry_msgs::msg::Pose& final_pose)
{
    if (control_state_ != GRASPING) return;
    
    // Move all blocking operations (MoveIt Execute, Service call) into a new thread
    std::thread([this, final_pose]() {
        RCLCPP_INFO(this->logger_, "GRASP: Starting descent and attachment thread...");

        RCLCPP_INFO(this->logger_, "GRASP: Executing final Cartesian descent to box surface.");

        // Preserve the current orientation for the descent motion
        geometry_msgs::msg::Pose current_eef_pose = this->move_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose vertical_final_pose = final_pose;
        // Ensure the final pose uses the orientation that the tracking loop stabilized on
        vertical_final_pose.orientation = current_eef_pose.orientation; 

        std::vector<geometry_msgs::msg::Pose> descent_waypoints;
        descent_waypoints.push_back(vertical_final_pose); // Target: Box surface (0cm offset)
        
        moveit_msgs::msg::RobotTrajectory descent_trajectory;
        const double eef_step = 0.005; 
        const double jump_threshold = 2.0; 
        
        double fraction = this->move_group_->computeCartesianPath(descent_waypoints, eef_step, jump_threshold, descent_trajectory);

        if (fraction > 0.95) { 
            this->move_group_->execute(descent_trajectory);
            RCLCPP_INFO(this->logger_, "GRASP: Descent complete. Attempting attachment.");
        } else {
            RCLCPP_ERROR(this->logger_, "GRASP: Failed to compute final descent path (Fraction: %.2f). Aborting attachment.", fraction);
            this->control_state_ = IDLE;
            return;
        }
        
        bool attachment_success = false;
        
        if (this->attach_client_->wait_for_service(std::chrono::milliseconds(500))) {
            auto request = std::make_shared<msg_gazebo::srv::AttachDetach::Request>();
            request->model1 = "mobile_manipulator"; 
            request->link1 = "Link7";            
            request->model2 = "aruco_box";      
            request->link2 = "link_0";           
            request->attach = true;

            auto future = this->attach_client_->async_send_request(request);
            
            auto temp_executor_node = std::make_shared<rclcpp::Node>("temp_attachment_executor");

            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(temp_executor_node); 

            if (executor.spin_until_future_complete(future, std::chrono::seconds(2)) == rclcpp::FutureReturnCode::SUCCESS) {
                
                auto result = future.get(); 
                
                if (result->success) {
                    RCLCPP_INFO(this->logger_, "GRASP: Attachment successful!");
                    attachment_success = true;
                } else {
                    RCLCPP_ERROR(this->logger_, "GRASP: Attachment service failed: %s", result->message.c_str());
                }
            } else {
                RCLCPP_ERROR(this->logger_, "GRASP: Attachment service call interrupted or timed out.");
            }
            
            
        } else {
            RCLCPP_ERROR(this->logger_, "GRASP: AttachDetach service not available during grasping phase.");
        }

        if (attachment_success) {
            this->control_state_ = RETRACTING;
            RCLCPP_INFO(this->logger_, "Attachment successful. Retracting arm to home position.");
            
            this->move_group_->clearPoseTargets();
            this->move_group_->setNamedTarget("home"); 
            
            moveit::planning_interface::MoveGroupInterface::Plan retract_plan;
            
            if (this->move_group_->plan(retract_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                this->move_group_->execute(retract_plan);
            } else {
                 RCLCPP_ERROR(this->logger_, "Failed to plan retract motion!");
            }
        } else {
            RCLCPP_ERROR(this->logger_, "Skipping retraction due to failed attachment. Resetting state.");
        }
        
        this->control_state_ = IDLE;
        RCLCPP_INFO(this->logger_, "Pick cycle complete. System IDLE.");

    }).detach(); 
}

} // namespace pipeline_manipulator

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