#include "pipeline_manipulator/pick_controller.hpp"
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace pipeline_manipulator
{

PickController::PickController(const rclcpp::NodeOptions & options)
: Node("pick_controller", options),
  logger_(get_logger()),
  received_pose_(false),
  move_group_initialized_(false)
{
    RCLCPP_INFO(logger_, "PickController started. Waiting for /detected_box_pose and /start_picking trigger.");
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/detected_box_pose", 10,
        std::bind(&PickController::pose_callback, this, std::placeholders::_1));

    trigger_srv_ = create_service<std_srvs::srv::Trigger>(
        "start_picking",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            if(!received_pose_) {
                response->success = false;
                response->message = "No box pose received yet!";
                RCLCPP_WARN(logger_, "%s", response->message.c_str());
                return;
            }

            if(!move_group_initialized_) {
                initialize_move_group();
                if(!move_group_) {
                    response->success = false;
                    response->message = "Failed to initialize MoveGroupInterface!";
                    RCLCPP_ERROR(logger_, "%s", response->message.c_str());
                    return;
                }
                move_group_initialized_ = true;
            }

            RCLCPP_INFO(logger_, "Executing pick to [%.3f, %.3f, %.3f] in frame '%s'",
                        latest_pose_.pose.position.x,
                        latest_pose_.pose.position.y,
                        latest_pose_.pose.position.z,
                        latest_pose_.header.frame_id.c_str());


            if(move_to_pose(latest_pose_)) {
                response->success = true;
                response->message = "Picking completed successfully!";
                RCLCPP_INFO(logger_, " Picking succeeded.");
            } else {
                response->success = false;
                response->message = "Failed to plan or execute motion!";
                RCLCPP_ERROR(logger_, " Picking failed.");
            }
        });
}

void PickController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    latest_pose_ = *msg;
    received_pose_ = true;
    RCLCPP_INFO_ONCE(logger_, " Received first box pose in frame '%s'", msg->header.frame_id.c_str());
}

void PickController::initialize_move_group()
{
    try {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "rm_group");
        RCLCPP_INFO(logger_, " MoveGroupInterface initialized for group 'rm_group'");
    } catch(const std::exception& e) {
        RCLCPP_ERROR(logger_, " Failed to initialize MoveGroupInterface: %s", e.what());
        move_group_.reset();
    }
}


bool PickController::move_to_pose(const geometry_msgs::msg::PoseStamped& pose_msg)
{
    if(!move_group_) {
        RCLCPP_ERROR(logger_, " MoveGroupInterface not initialized!");
        return false;
    }


    move_group_->setPoseReferenceFrame(pose_msg.header.frame_id);


    move_group_->setPoseTarget(pose_msg.pose);

  
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(logger_, " Motion planning failed!");
        return false;
    }
    RCLCPP_INFO(logger_, " Motion plan found. Executing...");

    // Execute
    auto exec_result = move_group_->execute(plan);

    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger_, " Motion executed successfully.");
        return true;
    } else {
        RCLCPP_ERROR(logger_, " Motion execution failed! Error code: %d", exec_result.val);
        return false;
    }
}

} // namespace pipeline_manipulator

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pipeline_manipulator::PickController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
