#ifndef AMR_ELEVATOR_TRIGGER_PLUGIN_HPP
#define AMR_ELEVATOR_TRIGGER_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace gazebo
{
  class AmrElevatorTriggerPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void OnUpdate();
  
  private:
    void callElevatorService(bool state);

    physics::ModelPtr model;
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr elevator_client;

    event::ConnectionPtr updateConnection;

    ignition::math::Vector3d target_pose_enter;
    ignition::math::Vector3d target_pose_exit;
    double tolerance = 0.5;

    bool enter_called = false;
    bool exit_called = false;
  };
}

#endif
