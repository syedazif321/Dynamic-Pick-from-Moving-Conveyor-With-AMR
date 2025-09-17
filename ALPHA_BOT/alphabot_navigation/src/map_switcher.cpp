#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;

class FloorMapSwitcher : public rclcpp::Node
{
public:
  FloorMapSwitcher()
  : Node("map_switcher")
  {
    std::string pkg_share = ament_index_cpp::get_package_share_directory("alphabot_navigation");
    std::string config_dir = pkg_share + "/config";

    map_paths_[false] = config_dir + "/floor_0.yaml";
    map_paths_[true]  = config_dir + "/floor_1.yaml";

    map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    clear_global_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/global_costmap/clear_entirely_global_costmap");
    clear_local_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/local_costmap/clear_entirely_local_costmap");

    wait_for_service(map_client_, "map_server/load_map");
    wait_for_service(clear_global_client_, "global_costmap/clear_entirely_global_costmap");
    wait_for_service(clear_local_client_, "local_costmap/clear_entirely_local_costmap");

    floor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/use_floor_1", 10, std::bind(&FloorMapSwitcher::floor_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "FloorMapSwitcher node started. Listening on /use_floor_1 (bool).");
  }

private:
  void floor_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    bool use_floor1 = msg->data;
    std::string map_yaml = map_paths_[use_floor1];
    RCLCPP_INFO(this->get_logger(), "Switching to map: %s", map_yaml.c_str());

    auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    req->map_url = map_yaml;

    // Async service call
    map_client_->async_send_request(req,
      [this](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future)
      {
        auto result = future.get();
        if (result->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
          RCLCPP_ERROR(this->get_logger(), "LoadMap failed with code: %d", result->result);
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Map switched successfully.");

        // Clear costmaps asynchronously
        clear_costmap(clear_global_client_, "global_costmap");
        clear_costmap(clear_local_client_, "local_costmap");
      });
  }

  void wait_for_service(rclcpp::ClientBase::SharedPtr client, const std::string & name)
  {
    while (!client->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) return;
      RCLCPP_INFO(this->get_logger(), "Waiting for %s service...", name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Connected to %s service", name.c_str());
  }

  void clear_costmap(rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client,
                     const std::string & name)
  {
    auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    client->async_send_request(req,
      [this, name](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
      {
        try {
          future.get();
          RCLCPP_INFO(this->get_logger(), "Cleared %s", name.c_str());
        } catch (const std::exception & e) {
          RCLCPP_WARN(this->get_logger(), "Failed to clear %s: %s", name.c_str(), e.what());
        }
      });
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr floor_sub_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_client_;
  std::unordered_map<bool, std::string> map_paths_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FloorMapSwitcher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
