#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_conveyorbelt/ros2_conveyorbelt_plugin.hpp"

#include <conveyorbelt_msgs/srv/conveyor_belt_control.hpp>
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
  double limit_{0.015}; // From SDF upper limit

  rclcpp::Publisher<conveyorbelt_msgs::msg::ConveyorBeltState>::SharedPtr status_pub_;
  conveyorbelt_msgs::msg::ConveyorBeltState status_msg_;

  rclcpp::Service<conveyorbelt_msgs::srv::ConveyorBeltControl>::SharedPtr enable_service_;

  rclcpp::Time last_publish_time_;
  int update_ns_{1000000};  // nanoseconds (default 1 ms)

  gazebo::event::ConnectionPtr update_connection_;

  void PublishStatus();
  void SetConveyorPower(
    conveyorbelt_msgs::srv::ConveyorBeltControl::Request::SharedPtr,
    conveyorbelt_msgs::srv::ConveyorBeltControl::Response::SharedPtr);
  void OnUpdate();
};

ROS2ConveyorBeltPlugin::ROS2ConveyorBeltPlugin()
: impl_(std::make_unique<ROS2ConveyorBeltPluginPrivate>())
{
}

ROS2ConveyorBeltPlugin::~ROS2ConveyorBeltPlugin() = default;

void ROS2ConveyorBeltPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get joint directly by name from flat structure
  impl_->belt_joint_ = _model->GetJoint("belt_joint");

  if (!impl_->belt_joint_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Failed to find belt_joint. Conveyor plugin will not work.");
    return;
  }

  // Read parameters
  impl_->max_velocity_ = _sdf->Get<double>("max_velocity");
  double publish_rate = _sdf->Get<double>("publish_rate");
  impl_->update_ns_ = static_cast<int>((1.0 / publish_rate) * 1e9);  // Hz to nanoseconds

  impl_->limit_ = impl_->belt_joint_->UpperLimit();

  // Init status message
  impl_->status_msg_.enabled = false;
  impl_->status_msg_.power = 0;

  // Publisher
  impl_->status_pub_ = impl_->ros_node_->create_publisher<conveyorbelt_msgs::msg::ConveyorBeltState>(
    "CONVEYORSTATE", 10);

  // Service
  impl_->enable_service_ = impl_->ros_node_->create_service<conveyorbelt_msgs::srv::ConveyorBeltControl>(
    "CONVEYORPOWER",
    std::bind(&ROS2ConveyorBeltPluginPrivate::SetConveyorPower, impl_.get(), std::placeholders::_1, std::placeholders::_2));

  impl_->last_publish_time_ = impl_->ros_node_->get_clock()->now();

  // World update hook
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ROS2ConveyorBeltPluginPrivate::OnUpdate, impl_.get()));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "ROS2 Conveyor Belt plugin loaded.");
}

void ROS2ConveyorBeltPluginPrivate::OnUpdate()
{
  // Drive the belt joint
  belt_joint_->SetVelocity(0, belt_velocity_);

  // Loop the joint movement
  double belt_position = belt_joint_->Position(0);
  if (belt_position >= limit_) {
    belt_joint_->SetPosition(0, 0);
  }

  // Periodic status publishing
  rclcpp::Time now = ros_node_->get_clock()->now();
  if ((now - last_publish_time_).nanoseconds() >= update_ns_) {
    PublishStatus();
    last_publish_time_ = now;
  }
}

void ROS2ConveyorBeltPluginPrivate::SetConveyorPower(
  conveyorbelt_msgs::srv::ConveyorBeltControl::Request::SharedPtr req,
  conveyorbelt_msgs::srv::ConveyorBeltControl::Response::SharedPtr res)
{
  if (req->power >= 0 && req->power <= 100) {
    power_ = req->power;
    belt_velocity_ = max_velocity_ * (power_ / 100.0);
    res->success = true;
  } else {
    RCLCPP_WARN(ros_node_->get_logger(), "Invalid power value %.2f (must be between 0 and 100).", req->power);
    res->success = false;
  }
}

void ROS2ConveyorBeltPluginPrivate::PublishStatus()
{
  status_msg_.power = power_;
  status_msg_.enabled = (power_ > 0);
  status_pub_->publish(status_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(ROS2ConveyorBeltPlugin)

}  // namespace gazebo_ros
