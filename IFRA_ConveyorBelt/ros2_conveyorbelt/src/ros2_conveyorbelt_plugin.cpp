#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_conveyorbelt/ros2_conveyorbelt_plugin.hpp"

#include <conveyorbelt_msgs/srv/conveyor_belt_control.hpp>
#include <conveyorbelt_msgs/srv/move_distance.hpp>
#include <conveyorbelt_msgs/msg/conveyor_belt_state.hpp>

#include <memory>

namespace gazebo_ros
{

class ROS2ConveyorBeltPluginPrivate
{
public:
  gazebo_ros::Node::SharedPtr ros_node_;
  gazebo::physics::JointPtr belt_joint_;

  double belt_velocity_{0.0};
  double max_velocity_{1.0};
  double power_{0.0};
  double limit_{1.0};

  bool move_active_{false};
  double target_position_{0.0};
  double move_velocity_{0.0};
  double remaining_distance_{0.0};

  rclcpp::Publisher<conveyorbelt_msgs::msg::ConveyorBeltState>::SharedPtr status_pub_;
  conveyorbelt_msgs::msg::ConveyorBeltState status_msg_;

  rclcpp::Service<conveyorbelt_msgs::srv::ConveyorBeltControl>::SharedPtr enable_service_;
  rclcpp::Service<conveyorbelt_msgs::srv::MoveDistance>::SharedPtr move_service_;

  rclcpp::Time last_publish_time_;
  int update_ns_{1000000};

  gazebo::event::ConnectionPtr update_connection_;

  // Always use stepwise logic for all conveyors
  void PublishStatus();
  void SetConveyorPower(
    conveyorbelt_msgs::srv::ConveyorBeltControl::Request::SharedPtr,
    conveyorbelt_msgs::srv::ConveyorBeltControl::Response::SharedPtr);
  void MoveDistance(
    conveyorbelt_msgs::srv::MoveDistance::Request::SharedPtr,
    conveyorbelt_msgs::srv::MoveDistance::Response::SharedPtr);
  void OnUpdate();

  void SoftResetJointPosition();
};

ROS2ConveyorBeltPlugin::ROS2ConveyorBeltPlugin()
: impl_(std::make_unique<ROS2ConveyorBeltPluginPrivate>()) {}

ROS2ConveyorBeltPlugin::~ROS2ConveyorBeltPlugin() = default;

void ROS2ConveyorBeltPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
  impl_->belt_joint_ = _model->GetJoint("belt_joint");

  if (!impl_->belt_joint_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Failed to find belt_joint.");
    return;
  }

  impl_->max_velocity_ = _sdf->Get<double>("max_velocity", 1.0).first;
  double publish_rate = _sdf->Get<double>("publish_rate", 1000.0).first;
  impl_->update_ns_ = static_cast<int>((1.0 / publish_rate) * 1e9);
  impl_->limit_ = impl_->belt_joint_->UpperLimit();

  // Always use stepwise logic — remove namespace check
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Stepwise movement mode ENABLED for all conveyors");

  const std::string ns = impl_->ros_node_->get_namespace();
  std::string topic_name = ns + "/CONVEYORSTATE";
  std::string power_srv = ns + "/CONVEYORPOWER";
  std::string move_srv = ns + "/MoveDistance";

  impl_->status_pub_ = impl_->ros_node_->create_publisher<conveyorbelt_msgs::msg::ConveyorBeltState>(topic_name, 10);

  impl_->enable_service_ = impl_->ros_node_->create_service<conveyorbelt_msgs::srv::ConveyorBeltControl>(
    power_srv,
    std::bind(&ROS2ConveyorBeltPluginPrivate::SetConveyorPower, impl_.get(), std::placeholders::_1, std::placeholders::_2));

  impl_->move_service_ = impl_->ros_node_->create_service<conveyorbelt_msgs::srv::MoveDistance>(
    move_srv,
    std::bind(&ROS2ConveyorBeltPluginPrivate::MoveDistance, impl_.get(), std::placeholders::_1, std::placeholders::_2));

  impl_->last_publish_time_ = impl_->ros_node_->get_clock()->now();

  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ROS2ConveyorBeltPluginPrivate::OnUpdate, impl_.get()));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Conveyor plugin loaded in namespace [%s]", ns.c_str());
}

void ROS2ConveyorBeltPluginPrivate::OnUpdate()
{
  double current_position = belt_joint_->Position(0);

  if (move_active_) {
    // Check if we've reached the current step target
    if (std::abs(current_position - target_position_) < 0.0005) {
      // Stop and reset joint position
      belt_joint_->SetVelocity(0, 0.0);
      SoftResetJointPosition();

      // Decrease remaining distance by the step size (which was |target_position_| since we start from 0)
      remaining_distance_ -= std::abs(target_position_);

      if (remaining_distance_ <= 0.0005) {
        move_active_ = false;
        RCLCPP_INFO(ros_node_->get_logger(), "All distance moved. Done.");
      } else {
        // Plan next step
        double step = std::min(limit_, remaining_distance_);
        target_position_ = step;
        RCLCPP_DEBUG(ros_node_->get_logger(), "Next step: Target=%.4f, Remaining=%.4f", target_position_, remaining_distance_);
      }
    } else {
      // Move toward current step target
      double dir = (target_position_ > current_position) ? 1.0 : -1.0;
      belt_joint_->SetVelocity(0, dir * move_velocity_);
    }
  } else {
    // Normal power-based movement
    belt_joint_->SetVelocity(0, belt_velocity_);
  }

  // Publish status at desired rate
  rclcpp::Time now = ros_node_->get_clock()->now();
  if ((now - last_publish_time_).nanoseconds() >= update_ns_) {
    PublishStatus();
    last_publish_time_ = now;
  }
}

void ROS2ConveyorBeltPluginPrivate::SoftResetJointPosition()
{
  // Stop motion
  belt_joint_->SetVelocity(0, 0.0);
  // Reset position to 0 (soft reset)
  belt_joint_->SetPosition(0, 0.0);
  // Ensure velocity stays zero after reset
  belt_joint_->SetVelocity(0, 0.0);
}

void ROS2ConveyorBeltPluginPrivate::SetConveyorPower(
  conveyorbelt_msgs::srv::ConveyorBeltControl::Request::SharedPtr req,
  conveyorbelt_msgs::srv::ConveyorBeltControl::Response::SharedPtr res)
{
  if (req->power >= 0 && req->power <= 100) {
    power_ = req->power;
    belt_velocity_ = max_velocity_ * (power_ / 100.0);
    // Disable move mode when setting power
    move_active_ = false;
    res->success = true;
  } else {
    RCLCPP_WARN(ros_node_->get_logger(), "Invalid power value %.2f", req->power);
    res->success = false;
  }
}

void ROS2ConveyorBeltPluginPrivate::MoveDistance(
  conveyorbelt_msgs::srv::MoveDistance::Request::SharedPtr req,
  conveyorbelt_msgs::srv::MoveDistance::Response::SharedPtr res)
{
  if (req->distance <= 0.0001) {
    RCLCPP_WARN(ros_node_->get_logger(), "Requested distance too small.");
    res->success = false;
    return;
  }

  // Always use stepwise logic
  remaining_distance_ = req->distance;
  move_velocity_ = 0.2 * max_velocity_;  // Slower speed for precision
  belt_velocity_ = 0.0;
  move_active_ = true;

  // First step: move up to limit or remaining distance
  target_position_ = std::min(limit_, remaining_distance_);

  res->success = true;
  RCLCPP_INFO(ros_node_->get_logger(), "Stepwise move start: Total=%.3f, First Step=%.3f", req->distance, target_position_);
}

void ROS2ConveyorBeltPluginPrivate::PublishStatus()
{
  status_msg_.power = power_;
  status_msg_.enabled = (power_ > 0) || move_active_;
  status_pub_->publish(status_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(ROS2ConveyorBeltPlugin)

}  // namespace gazebo_ros