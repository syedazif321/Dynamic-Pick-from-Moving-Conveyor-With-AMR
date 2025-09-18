// File: include/pipeline_manipulator/pick_controller.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace pipeline_manipulator
{

class PickController : public rclcpp::Node
{
public:
    explicit PickController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void initialize_move_group();
    bool move_to_pose(const geometry_msgs::msg::PoseStamped& pose_msg);

    rclcpp::Logger logger_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;

    geometry_msgs::msg::PoseStamped latest_pose_;
    bool received_pose_;
    bool move_group_initialized_;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

} // namespace pipeline_manipulator