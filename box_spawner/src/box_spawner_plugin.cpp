#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <sdf/sdf.hh>
#include <sstream>

namespace gazebo
{

class BoxSpawnerPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr) override
  {
    world_ = world;
    ros_node_ = gazebo_ros::Node::Get();

    srv_ = ros_node_->create_service<std_srvs::srv::Trigger>(
      "spawn_box",
      std::bind(&BoxSpawnerPlugin::SpawnBoxCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(ros_node_->get_logger(), "BoxSpawnerPlugin loaded. Service 'spawn_box' is ready.");
  }

private:
  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  int box_counter_ = 0;  // Sequential box counter

  void SpawnBoxCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    std::ostringstream name;
    name << "aruco_box_" << box_counter_++;

    // Fixed spawn pose
    double x = 0.0;
    double y = 0.0;
    double z = 0.05;
    double roll = 0.0;
    double pitch = -0.0000;
    double yaw = 0.0;

    // Use relative model path (must be in GAZEBO_MODEL_PATH)
    std::string model_uri = "model://aruco_box";

    std::ostringstream sdf_str;
    sdf_str << "<sdf version='1.6'>"
            << "  <model name='" << name.str() << "'>"
            << "    <include>"
            << "      <uri>" << model_uri << "</uri>"
            << "      <pose>" << x << " " << y << " " << z << " "
            << roll << " " << pitch << " " << yaw << "</pose>"
            << "      <name>" << name.str() << "</name>"
            << "    </include>"
            << "  </model>"
            << "</sdf>";

    sdf::SDF model_sdf;
    model_sdf.SetFromString(sdf_str.str());
    world_->InsertModelSDF(model_sdf);

    res->success = true;
    res->message = "Spawned model: " + name.str();
    RCLCPP_INFO(ros_node_->get_logger(), "Spawned %s at pose (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
                name.str().c_str(), x, y, z, roll, pitch, yaw);
  }
};

GZ_REGISTER_WORLD_PLUGIN(BoxSpawnerPlugin)

}  // namespace gazebo
