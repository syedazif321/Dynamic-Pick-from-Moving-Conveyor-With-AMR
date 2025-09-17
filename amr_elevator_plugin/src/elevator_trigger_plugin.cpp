#include "elevator_trigger_plugin.hpp"
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  void AmrElevatorTriggerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    model = _model;

    gzlog << "[AmrElevatorTriggerPlugin] Successfully loaded!" << std::endl;
    
    gzlog << "[AmrElevatorTriggerPlugin] Model Name: " << _model->GetName() << std::endl;

    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    

    ros_node = std::make_shared<rclcpp::Node>("amr_elevator_trigger_plugin");
    elevator_client = ros_node->create_client<std_srvs::srv::SetBool>("/elevator_cmd");

    if (_sdf->HasElement("target_pose_enter"))
    {
      auto elem = _sdf->GetElement("target_pose_enter");
      target_pose_enter = ignition::math::Vector3d(
        elem->Get<double>("x"),
        elem->Get<double>("y"),
        elem->Get<double>("z"));
    }
    else
    {
      gzerr << "[AmrElevatorTriggerPlugin] Missing <target_pose_enter> in SDF.\n";
      return;
    }

    if (_sdf->HasElement("target_pose_exit"))
    {
      auto elem = _sdf->GetElement("target_pose_exit");
      target_pose_exit = ignition::math::Vector3d(
        elem->Get<double>("x"),
        elem->Get<double>("y"),
        elem->Get<double>("z"));
    }
    else
    {
      gzerr << "[AmrElevatorTriggerPlugin] Missing <target_pose_exit> in SDF.\n";
      return;
    }

    tolerance = _sdf->HasElement("tolerance") ? _sdf->Get<double>("tolerance") : 0.5;

    updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&AmrElevatorTriggerPlugin::OnUpdate, this));
  }

  void AmrElevatorTriggerPlugin::OnUpdate()
  {
    if (!ros_node) return;

    auto pose = model->WorldPose().Pos();

    if (!enter_called && pose.Distance(target_pose_enter) <= tolerance)
    {
      RCLCPP_INFO(ros_node->get_logger(), "Robot reached ENTER target. Calling elevator open service...");
      callElevatorService(true);
      enter_called = true;
    }

    if (!exit_called && pose.Distance(target_pose_exit) <= tolerance)
    {
      RCLCPP_INFO(ros_node->get_logger(), "Robot reached EXIT target. Calling elevator close service...");
      callElevatorService(false);
      exit_called = true;
    }

    rclcpp::spin_some(ros_node);
  }

  void AmrElevatorTriggerPlugin::callElevatorService(bool state)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = state;

    if (!elevator_client->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(ros_node->get_logger(), "Elevator service not available!");
      return;
    }

    auto result = elevator_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(ros_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(ros_node->get_logger(), "Elevator service called successfully!");
    }
    else
    {
      RCLCPP_ERROR(ros_node->get_logger(), "Failed to call elevator service.");
    }
  }

  // REGISTER the plugin
  GZ_REGISTER_MODEL_PLUGIN(AmrElevatorTriggerPlugin)
}
