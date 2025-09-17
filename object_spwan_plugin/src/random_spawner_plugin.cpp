#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <random>
#include <string>
#include <vector>

namespace gazebo
{
  class FixedSpawnerPlugin : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      world_ = _world;
      node_ = gazebo_ros::Node::Get(_sdf);

      rng_.seed(std::random_device{}());
      update_connection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&FixedSpawnerPlugin::OnUpdate, this));

      // Define models and their two poses
      models_ = {
        {"aws_robomaker_warehouse_TrashCanC_01", 
         {{6.686410, 5.400180, 0, 0, 0, 0},
          {-4.237031, 10.930869, 0, 0, 0, 0}}},
        {"aws_robomaker_warehouse_ClutteringA_01", 
         {{-4.948304, 3.357823, -0.017477, 0, 0, -1.583191},
          {6.686410, 5.400180, 0, 0, 0, 0}}}
      };

      last_spawn_time_ = world_->SimTime();
      spawn_interval_ = 5.0;  // seconds
    }

  private:
    struct Pose6D {
      double x, y, z, roll, pitch, yaw;
    };

    struct ModelData {
      std::string name;
      std::vector<Pose6D> poses;
      int current_pose_index = 0;
      bool spawned = false;
    };

    void OnUpdate()
    {
      auto sim_time = world_->SimTime();
      double elapsed = (sim_time - last_spawn_time_).Double();

      if (elapsed >= spawn_interval_)
      {
        for (auto &model : models_)
        {
          // Remove existing model if spawned
          if (model.spawned)
          {
            world_->RemoveModel(model.name);
            model.spawned = false;
          }

          // Choose the next pose
          Pose6D &p = model.poses[model.current_pose_index];
          SpawnModel(model.name, p);

          // Toggle to the other pose for next spawn
          model.current_pose_index = 1 - model.current_pose_index;
        }

        last_spawn_time_ = sim_time;
      }
    }

    void SpawnModel(const std::string &model_name, const Pose6D &pose)
    {
      sdf::SDF modelSDF;
      std::string model_uri = "model://" + model_name;

      modelSDF.SetFromString(
        "<sdf version='1.6'>\
           <model name='" + model_name + "'>\
             <include>\
               <uri>" + model_uri + "</uri>\
             </include>\
             <pose>" + std::to_string(pose.x) + " " +
                         std::to_string(pose.y) + " " +
                         std::to_string(pose.z) + " " +
                         std::to_string(pose.roll) + " " +
                         std::to_string(pose.pitch) + " " +
                         std::to_string(pose.yaw) + "</pose>\
           </model>\
         </sdf>"
      );

      world_->InsertModelSDF(modelSDF);

      // Mark as spawned
      for (auto &m : models_)
        if (m.name == model_name) m.spawned = true;
    }

    physics::WorldPtr world_;
    gazebo_ros::Node::SharedPtr node_;
    event::ConnectionPtr update_connection_;
    std::mt19937 rng_;

    std::vector<ModelData> models_;
    common::Time last_spawn_time_;
    double spawn_interval_;
  };

  GZ_REGISTER_WORLD_PLUGIN(FixedSpawnerPlugin)
}
