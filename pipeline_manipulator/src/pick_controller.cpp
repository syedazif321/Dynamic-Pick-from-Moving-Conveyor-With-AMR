// File: /home/azif/projetcs/Dynamic-Pick-from-Moving-Conveyor-With-AMR/pipeline_manipulator/src/pick_controller.cpp

#include "pipeline_manipulator/pick_controller.hpp"
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h> 
#include <thread> 
#include "msg_gazebo/srv/attach_detach.hpp" 
#include <chrono>
#include <cmath> 
#include "rclcpp/executors/single_threaded_executor.hpp" 
#include "rclcpp/node.hpp" 
// REMOVED: #include <moveit/core/moveit_error_codes/moveit_error_codes.h> (This was the source of the error)

using namespace std::chrono_literals;

namespace pipeline_manipulator
{

// Define the desired initial joint positions (from your ros2 topic pub command)
const std::vector<double> INITIAL_JOINT_POSITIONS = {
    0, 0, 0, 0, 0, 0, 0 // All joints at zero
};

// Define the joint names in the order expected by the MoveGroupInterface
const std::vector<std::string> JOINT_NAMES = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"
};

DynamicPickController::DynamicPickController(const rclcpp::NodeOptions & options)
    : Node("dynamic_pick_controller", options),
      logger_(get_logger()),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
      tf_listener_(*tf_buffer_),
      control_state_(IDLE) // Start in IDLE
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
    
    attach_client_ = create_client<msg_gazebo::srv::AttachDetach>("/AttachDetach");
    
    // Control loop frequency set low, as most moves are now blocking
    control_timer_ = create_wall_timer(
        500ms, 
        std::bind(&DynamicPickController::control_loop, this));

    RCLCPP_INFO(logger_, "DynamicPickController ready. Waiting for initial setup.");
}

bool DynamicPickController::initialize_move_group()
{
    RCLCPP_INFO(logger_, "Attempting to initialize MoveGroupInterface...");
    
    if (move_group_) return true;

    try {
        auto move_group_handle = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), arm_group_name_);
            
        move_group_handle->setPoseReferenceFrame(base_link_frame_);
        move_group_handle->setPlanningTime(0.3); 
        move_group_handle->setMaxVelocityScalingFactor(0.8);
        move_group_handle->setMaxAccelerationScalingFactor(0.8);
        
        move_group_ = std::move(move_group_handle);
        
        RCLCPP_INFO(logger_, "MoveGroupInterface successfully initialized for group: %s.", arm_group_name_.c_str());
        return true;
    } catch(const std::exception& e) {
        RCLCPP_FATAL(logger_, "Failed to create MoveGroupInterface: %s", e.what());
        move_group_.reset();
        return false;
    }
}

void DynamicPickController::check_setup_and_start_initial_move()
{
    if (setup_complete_ || control_state_ != IDLE) {
        return; 
    }
    
    if (latest_odom_.header.stamp.nanosec == 0) {
        return; 
    }

    // Only start if we have a box pose AND MoveIt is ready
    if (latest_box_state_.header.stamp.nanosec != 0 && initialize_move_group() && attach_client_->wait_for_service(std::chrono::seconds(1))) {
        setup_complete_ = true;
        control_state_ = INITIAL_MOVE; 
        RCLCPP_INFO(logger_, "Initial setup complete. Auto-starting INITIAL_MOVE state.");
    }
}

void DynamicPickController::box_state_callback(const msg_gazebo::msg::BoxState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_box_state_ = *msg;
    check_setup_and_start_initial_move(); 
}

void DynamicPickController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_odom_ = *msg;
    check_setup_and_start_initial_move(); 
}

// Function to perform the initial joint move using MoveIt (Blocking)
bool DynamicPickController::execute_initial_move()
{
    if (!move_group_) return false;

    RCLCPP_INFO(logger_, "INITIAL_MOVE: Moving arm to initial joint pose...");

    move_group_->setJointValueTarget(JOINT_NAMES, INITIAL_JOINT_POSITIONS);

    moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
    
    // Using moveit::core::MoveItErrorCode to avoid deprecation warnings
    moveit::core::MoveItErrorCode success = move_group_->plan(initial_plan);

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger_, "INITIAL_MOVE: Plan successful. Executing...");
        
        // Using moveit::core::MoveItErrorCode
        moveit::core::MoveItErrorCode execution_success = move_group_->execute(initial_plan);
        move_group_->clearPoseTargets();

        if (execution_success == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger_, "INITIAL_MOVE: Execution complete. Transitioning to READY_TO_PICK.");
            return true;
        } else {
            RCLCPP_ERROR(logger_, "INITIAL_MOVE: Execution failed. Transitioning to IDLE.");
            return false;
        }
    } else {
        RCLCPP_ERROR(logger_, "INITIAL_MOVE: Planning failed. Transitioning to IDLE.");
        return false;
    }
}

// --- CORE PICK SEQUENCE (Blocking execution is essential here) ---
void DynamicPickController::execute_pick_sequence(const geometry_msgs::msg::Pose& target_pose)
{
    // Use a detached thread to prevent blocking the main ROS 2 spin loop permanently
    std::thread([this, target_pose]() {
        RCLCPP_INFO(this->logger_, "PICK SEQUENCE: Thread started.");
        
        this->control_state_ = TRACKING;

        // --- 1. GO TO TF POSE (Blocking Move) ---
        RCLCPP_INFO(this->logger_, "TRACKING: Moving to vision target pose.");
        this->move_group_->setPoseTarget(target_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan tracking_plan;
        
        // Using moveit::core::MoveItErrorCode
        moveit::core::MoveItErrorCode plan_success = this->move_group_->plan(tracking_plan);

        if (plan_success == moveit::core::MoveItErrorCode::SUCCESS) { 
            
            // Using moveit::core::MoveItErrorCode
            moveit::core::MoveItErrorCode execution_success = this->move_group_->execute(tracking_plan);
            this->move_group_->clearPoseTargets(); 
            
            if (execution_success != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(this->logger_, "TRACKING: Failed to execute move to target. Aborting.");
                this->control_state_ = IDLE;
                return;
            }
        } else {
            RCLCPP_ERROR(this->logger_, "TRACKING: Failed to plan move to target. Aborting.");
            this->control_state_ = IDLE;
            return;
        }
        
        // --- 2. ATTACH SERVICE CALL ---
        this->control_state_ = GRASPING;
        RCLCPP_INFO(this->logger_, "GRASPING: Target reached. Calling attachment service.");
        bool attachment_success = false;
        
        if (this->attach_client_->wait_for_service(std::chrono::milliseconds(500))) {
            auto request = std::make_shared<msg_gazebo::srv::AttachDetach::Request>();
            // IMPORTANT: Ensure these names match your URDF/SDF models!
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
                    RCLCPP_INFO(this->logger_, "GRASPING: Attachment successful!");
                    attachment_success = true;
                } else {
                    RCLCPP_ERROR(this->logger_, "GRASPING: Attachment service failed: %s", result->message.c_str());
                }
            } else {
                RCLCPP_ERROR(this->logger_, "GRASPING: Attachment service call interrupted or timed out.");
            }
        } else {
            RCLCPP_ERROR(this->logger_, "GRASPING: AttachDetach service not available.");
        }

        // --- 3. GO TO HOME POSE (Blocking Move) ---
        if (attachment_success) {
            this->control_state_ = RETRACTING;
            RCLCPP_INFO(this->logger_, "RETRACTING: Moving arm to home position.");
            
            this->move_group_->setNamedTarget("home"); 
            
            moveit::planning_interface::MoveGroupInterface::Plan retract_plan;
            
            // Using moveit::core::MoveItErrorCode
            if (this->move_group_->plan(retract_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                this->move_group_->execute(retract_plan);
            } else {
                 RCLCPP_ERROR(this->logger_, "RETRACTING: Failed to plan home motion!");
            }
        } else {
            RCLCPP_ERROR(this->logger_, "Skipping home move due to failed attachment.");
        }
        
        this->control_state_ = IDLE;
        this->setup_complete_ = false; 
        RCLCPP_INFO(this->logger_, "Pick cycle complete. System IDLE.");

    }).detach(); 
}


// --- Control Loop ---
void DynamicPickController::control_loop()
{
    if (!setup_complete_ || !move_group_) return;
    
    // 1. Handle the initial move state
    if (control_state_ == INITIAL_MOVE) {
        if (execute_initial_move()) {
            control_state_ = READY_TO_PICK; // New state to trigger the sequence
        } else {
            control_state_ = IDLE;
        }
        return; 
    }
    
    // 2. Trigger the entire pick sequence once the arm is ready and we have data
    if (control_state_ == READY_TO_PICK) {
        msg_gazebo::msg::BoxState box_state;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            box_state = latest_box_state_;
        }
        
        // Final check for fresh data (100ms tolerance) before starting the sequence
        if (box_state.header.stamp.nanosec == 0 || (this->now() - box_state.header.stamp) > 100ms) { 
             RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 500, "Box state data is too old/missing. Waiting for fresh data.");
             return;
        }

        // --- Transform Target Pose to Base Link ---
        geometry_msgs::msg::PoseStamped target_pose_base;
        try {
            tf_buffer_->transform(
                box_state.pose_world, target_pose_base, base_link_frame_, 
                tf2::TimePointZero, box_state.pose_world.header.frame_id, 200ms);                                     
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 500, "TF transform failed: %s", ex.what());
            return;
        }
        
        // Start the blocking sequence in a thread
        RCLCPP_INFO(logger_, "READY_TO_PICK: Initiating full pick sequence (Go to TF -> Attach -> Go Home).");
        execute_pick_sequence(target_pose_base.pose);
        
        // Sequence thread is now running; transition to a waiting state.
        control_state_ = PICK_IN_PROGRESS;
        return;
    }
    
    // 3. Prevent any new action while the pick is running
    // Fix: Explicitly load the atomic value and cast it to int for printing
    if (control_state_ == PICK_IN_PROGRESS || control_state_ == GRASPING || control_state_ == RETRACTING) {
        RCLCPP_DEBUG(logger_, "Pick sequence is in progress. Current state: %d", (int)control_state_.load());
        return;
    }
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