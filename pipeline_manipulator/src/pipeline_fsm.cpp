#include "pipeline_manipulator/pipeline_fsm.hpp"
#include <unistd.h>
#include <limits.h>

#include <unordered_map>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <sqlite3.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <msg_gazebo/srv/attach_detach.hpp>

using namespace std::chrono_literals;

// ----------------- PipelineFSM Implementation -----------------
PipelineFSM::PipelineFSM()
    : Node("pipeline_fsm_node"), current_state_(State::IDLE),
      analytics_db_(this->get_logger(), "pipeline_analytics.db") // 
{
    // Initialize action clients
    slider_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this, "/slider_position_controller/follow_joint_trajectory");
    arm_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this, "/rm_group_controller/follow_joint_trajectory");
    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    // Initialize service clients
    spawn_box_client_ = this->create_client<std_srvs::srv::Trigger>("/spawn_box");
    start_picking_client_ = this->create_client<std_srvs::srv::Trigger>("/start_picking");
    start_detection_client_ = this->create_client<std_srvs::srv::Trigger>("/start_detection");
    attach_detach_client_ = this->create_client<msg_gazebo::srv::AttachDetach>("/AttachDetach");
    elevator_client_ = this->create_client<std_srvs::srv::SetBool>("/elevator_cmd");

    // Initialize publisher
    floor_pub_ = this->create_publisher<std_msgs::msg::Bool>("/use_floor_1", 10);

    // Wait for servers
    while (!slider_client_->wait_for_action_server(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for slider trajectory server...");
    }
    while (!arm_client_->wait_for_action_server(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for arm trajectory server...");
    }
    while (!nav2_client_->wait_for_action_server(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 /navigate_to_pose action server...");
    }
    while (!spawn_box_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /spawn_box service...");
    }
    while (!start_picking_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /start_picking service...");
    }
    while (!start_detection_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /start_detection service...");
    }
    while (!attach_detach_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /AttachDetach service...");
    }
    while (!elevator_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /elevator_cmd service...");
    }

    // Load targets
    loadTargets();

    // Start Database Session — LOG ALL GOALS FROM Targets.json
    std::string session_name = "pipeline_" + std::to_string(static_cast<long long>(std::time(nullptr)));
    std::string start_time = getCurrentTimestamp();
    analytics_session_id_ = analytics_db_.startSession(session_name, start_time);
    if (analytics_session_id_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start AnalyticsDB session!");
    } else {
        //  LOG ALL ROBOT GOAL LOCATIONS FROM Targets.json
        if (!analytics_db_.logAllGoals(analytics_session_id_, targets_json_)) {
            RCLCPP_WARN(this->get_logger(), "Failed to log all robot goals to database.");
        }
        RCLCPP_INFO(this->get_logger(), "AnalyticsDB session started with ID: %d", analytics_session_id_);
    }

    // Start FSM
    timer_ = this->create_wall_timer(200ms, std::bind(&PipelineFSM::runFSM, this));
    RCLCPP_INFO(this->get_logger(), "Pipeline FSM started with AnalyticsDB integration.");
}

void PipelineFSM::loadTargets()
{
    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len != -1) {
        exe_path[len] = '\0';
        std::string exec_dir(exe_path);
        size_t last_slash = exec_dir.find_last_of("/");
        if (last_slash != std::string::npos) {
            exec_dir = exec_dir.substr(0, last_slash);
        }
        std::string json_path = exec_dir + "/../../robot_data/Targets.json";
        std::ifstream f(json_path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Targets.json at %s", json_path.c_str());
            throw std::runtime_error("JSON file not found");
        }
        try {
            targets_json_ = nlohmann::json::parse(f);
            RCLCPP_INFO(this->get_logger(), "Targets loaded from %s", json_path.c_str());
        } catch (const nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
            throw std::runtime_error("Failed to parse Targets.json");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not resolve executable path");
        throw std::runtime_error("Cannot resolve path");
    }
}

void PipelineFSM::navigateTo(const std::string& target_name)
{
    if (!targets_json_["amr_poses"].contains(target_name)) {
        RCLCPP_ERROR(this->get_logger(), "AMR pose '%s' not found in JSON", target_name.c_str());
        return;
    }

    auto& pose_data = targets_json_["amr_poses"][target_name];
    nav2_msgs::action::NavigateToPose::Goal goal;

    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();

    goal.pose.pose.position.x = pose_data["position"][0];
    goal.pose.pose.position.y = pose_data["position"][1];
    goal.pose.pose.position.z = pose_data["position"][2];

    goal.pose.pose.orientation.x = pose_data["orientation"][0];
    goal.pose.pose.orientation.y = pose_data["orientation"][1];
    goal.pose.pose.orientation.z = pose_data["orientation"][2];
    goal.pose.pose.orientation.w = pose_data["orientation"][3];

    // Reset flags
    navigation_complete_ = false;
    navigation_succeeded_ = false;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Navigation goal REJECTED");
                navigation_complete_ = true;
                navigation_succeeded_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Navigation goal ACCEPTED");
            }
        };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
            navigation_complete_ = true;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Navigation SUCCEEDED");
                    navigation_succeeded_ = true;
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Navigation ABORTED");
                    navigation_succeeded_ = false;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Navigation CANCELED");
                    navigation_succeeded_ = false;
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Navigation UNKNOWN result");
                    navigation_succeeded_ = false;
                    break;
            }
        };

    nav2_client_->async_send_goal(goal, send_goal_options);
    navigation_goal_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "Navigation goal sent to '%s'", target_name.c_str());
}

void PipelineFSM::sendSliderTrajectory()
{
    if (slider_goal_active_) {
        RCLCPP_WARN(this->get_logger(), "Slider goal already active — skipping");
        return;
    }

    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"slider_joint"};
    goal_msg.trajectory.points.resize(2);

    goal_msg.trajectory.points[0].positions = {0.0};
    goal_msg.trajectory.points[0].velocities = {0.0};
    goal_msg.trajectory.points[0].accelerations = {0.0};
    goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration(1s);

    goal_msg.trajectory.points[1].positions = {0.5};
    goal_msg.trajectory.points[1].velocities = {0.0};
    goal_msg.trajectory.points[1].accelerations = {0.0};
    goal_msg.trajectory.points[1].time_from_start = rclcpp::Duration(6s);

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Slider goal rejected");
                slider_goal_active_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Slider goal accepted");
            }
        };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
            slider_goal_active_ = false;

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), " Slider trajectory succeeded");
                this->transitionTo(State::SPAWN_BOX);
            } else {
                RCLCPP_ERROR(this->get_logger(), " Slider trajectory failed with code: %d", static_cast<int>(result.code));
                this->transitionTo(State::SPAWN_BOX);
            }
        };

    slider_client_->async_send_goal(goal_msg, send_goal_options);
    slider_goal_active_ = true;
    RCLCPP_INFO(this->get_logger(), "Sent slider trajectory goal");
}

void PipelineFSM::callTriggerService(const std::string& service_name)
{
    auto client = (service_name == "/spawn_box") ? spawn_box_client_ : start_picking_client_;
    if (!client || !client->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), " Service %s is not ready!", service_name.c_str());
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    client->async_send_request(request,
        [this, service_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), " %s succeeded", service_name.c_str());

                // If spawning box, log it (assume height = 0.1m, color = unknown, size = unknown)
                if (service_name == "/spawn_box") {
                    if (analytics_session_id_ >= 0) {
                        analytics_db_.logBoxSpawn(analytics_session_id_, "model_0", "unknown", 0.1);
                    }
                }

            } else {
                RCLCPP_ERROR(this->get_logger(), " %s failed: %s", service_name.c_str(), response->message.c_str());
            }
        });

    RCLCPP_INFO(this->get_logger(), " Triggered service: %s", service_name.c_str());
}

void PipelineFSM::callStartDetection()
{
    if (!start_detection_client_ || !start_detection_client_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), " /start_detection service is NOT ready!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Calling /start_detection service...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    start_detection_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Vision detection STARTED successfully");
                // Detection data will be logged via onBoxInfo or later transition
            } else {
                RCLCPP_ERROR(this->get_logger(), " Vision detection failed: %s", response->message.c_str());
            }
        });
}

void PipelineFSM::callStartPicking()
{
    if (!start_picking_client_ || !start_picking_client_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), " /start_picking service is NOT ready!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), " Calling /start_picking service...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    start_picking_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), " /start_picking succeeded — transitioning to ATTACH_OBJECT");
                this->transitionTo(State::ATTACH_OBJECT);
            } else {
                RCLCPP_ERROR(this->get_logger(), " /start_picking failed: %s", response->message.c_str());
            }
        });
}

void PipelineFSM::callAttachDetach(bool attach)
{
    if (!attach_detach_client_ || !attach_detach_client_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), " /AttachDetach service is NOT ready!");
        return;
    }

    auto request = std::make_shared<msg_gazebo::srv::AttachDetach::Request>();
    request->model1 = "mobile_manipulator";
    request->link1 = "Link7";
    request->model2 = "model_0"; 
    request->link2 = "link";
    request->attach = attach;

    attach_detach_client_->async_send_request(request,
        [this, attach](rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), " Attach/Detach %s succeeded", attach ? "ATTACH" : "DETACH");

                //  Log placement on detach
                if (!attach && analytics_session_id_ >= 0) {
                    analytics_db_.logBoxPlace(analytics_session_id_, "model_0");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), " Attach/Detach failed");
            }
        });

    RCLCPP_INFO(this->get_logger(), " Sent %s request to /AttachDetach", attach ? "ATTACH" : "DETACH");
}

void PipelineFSM::callSetBoolService(const std::string& service_name, bool data)
{
    if (!elevator_client_ || !elevator_client_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), " %s service is NOT ready!", service_name.c_str());
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = data;

    elevator_client_->async_send_request(request,
        [this, service_name, data](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            bool success = response->success;
            int floor_target = data ? 1 : 0; // true → floor 1

            if (response->success) {
                RCLCPP_INFO(this->get_logger(), " %s set to %s", service_name.c_str(), data ? "true" : "false");
            } else {
                RCLCPP_ERROR(this->get_logger(), " %s failed: %s", service_name.c_str(), response->message.c_str());
            }

            //  LOG LIFT USAGE
            if (analytics_session_id_ >= 0) {
                std::string action = "SWITCH_FLOOR"; // Since SetBool toggles floor
                analytics_db_.logLiftUsage(analytics_session_id_, action, floor_target, success);
            }
        });

    RCLCPP_INFO(this->get_logger(), " Setting %s to %s", service_name.c_str(), data ? "true" : "false");
}

void PipelineFSM::publishBoolTopic(const std::string& topic_name, bool data)
{
    auto msg = std_msgs::msg::Bool();
    msg.data = data;
    floor_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), " Published %s to %s", data ? "true" : "false", topic_name.c_str());
}

void PipelineFSM::sendArmTrajectory(const std::string& joint_target_name)
{

    if (arm_goal_active_) {
        RCLCPP_WARN(this->get_logger(), "Arm goal already active — skipping send for '%s'", joint_target_name.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "🔧 ENTERED sendArmTrajectory for: %s", joint_target_name.c_str());

    if (!targets_json_["arm_positions"].contains(joint_target_name)) {
        RCLCPP_ERROR(this->get_logger(), " Arm position '%s' not found in JSON!", joint_target_name.c_str());
        return;
    }

    auto& joint_values = targets_json_["arm_positions"][joint_target_name];
    std::vector<double> positions;
    for (auto& v : joint_values) {
        positions.push_back(v.get<double>());
    }

    RCLCPP_INFO(this->get_logger(), " Target positions: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]);

    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    goal_msg.trajectory.points.resize(1);

    goal_msg.trajectory.points[0].positions = positions;
    goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration(3s);

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this, joint_target_name](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), " Arm goal '%s' REJECTED", joint_target_name.c_str());
                this->arm_goal_active_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), " Arm goal '%s' ACCEPTED", joint_target_name.c_str());
            }
        };

    send_goal_options.result_callback =
        [this, joint_target_name](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
            this->arm_goal_active_ = false;  //  Clear flag on result

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), " Arm movement to '%s' SUCCEEDED", joint_target_name.c_str());
                switch (this->current_state_) {
                    case State::MOVE_ARM_SAFE_PICK:
                        this->transitionTo(State::START_VISION);
                        break;
                    case State::MOVE_ARM_HOME:
                        this->transitionTo(State::NAV_TO_MIDDLE);
                        break;
                    case State::MOVE_ARM_DROP_PRE:
                        this->transitionTo(State::MOVE_ARM_DROP);
                        break;
                    case State::MOVE_ARM_DROP:
                        this->transitionTo(State::DETACH_OBJECT);
                        break;
                    case State::MOVE_ARM_HOME_FINAL:
                        this->transitionTo(State::DONE);
                        break;
                    default:
                        break;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), " Arm movement to '%s' FAILED with code: %d", joint_target_name.c_str(), static_cast<int>(result.code));
            }
        };

    RCLCPP_INFO(this->get_logger(), " Sending arm trajectory goal to server...");

    if (!arm_client_->action_server_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), " Arm action server is NOT ready!");
        return;
    }

    arm_client_->async_send_goal(goal_msg, send_goal_options);
    arm_goal_active_ = true;  //  Set flag
    RCLCPP_INFO(this->get_logger(), " Arm goal sent successfully for '%s'", joint_target_name.c_str());
}

//  CORRECTLY SCOPED transitionTo()
void PipelineFSM::transitionTo(State next)
{
    RCLCPP_INFO(this->get_logger(), " TRANSITION: %d → %d", static_cast<int>(current_state_), static_cast<int>(next));
    current_state_ = next;
    
    if (timer_ && !timer_->is_canceled()) {
        timer_->cancel();
    }
    timer_ = this->create_wall_timer(200ms, std::bind(&PipelineFSM::runFSM, this));
    
    RCLCPP_INFO(this->get_logger(), " State updated. FSM timer restarted.");

    // Force immediate processing to avoid delay
    this->runFSM();
}

void PipelineFSM::runFSM()
{
    RCLCPP_INFO(this->get_logger(), "FSM TICK - Current State: %d", static_cast<int>(current_state_));
    switch (current_state_) {
        case State::IDLE:
            RCLCPP_INFO(this->get_logger(), " Starting pipeline from IDLE");
            transitionTo(State::NAV_TO_PICK_PRE);
            break;

        case State::NAV_TO_PICK_PRE:
            RCLCPP_INFO(this->get_logger(), "🗺️ Navigating to pick_pre_amr");
            navigateTo("pick_pre_amr");
            transitionTo(State::WAIT_NAV_PICK_PRE);
            break;

        case State::WAIT_NAV_PICK_PRE:
            if (!navigation_complete_) {
                RCLCPP_INFO(this->get_logger(), " Waiting for navigation to complete...");
                return;
            }
            if (!navigation_succeeded_) {
                RCLCPP_ERROR(this->get_logger(), " Failed to reach pick_pre_amr. Retrying...");
                transitionTo(State::NAV_TO_PICK_PRE);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Reached pick_pre_amr");
            transitionTo(State::SLIDER_EXTEND);
            break;

        case State::SLIDER_EXTEND:
            RCLCPP_INFO(this->get_logger(), " Extending slider");
            sendSliderTrajectory();
            break;

        case State::SPAWN_BOX:
            RCLCPP_INFO(this->get_logger(), " Spawning box");
            callTriggerService("/spawn_box");
            transitionTo(State::WAIT_AFTER_SPAWN);
            break;

        case State::WAIT_AFTER_SPAWN:
            RCLCPP_INFO(this->get_logger(), " Waiting 1s after spawn before moving arm...");
            timer_ = this->create_wall_timer(std::chrono::milliseconds(DELAY_1000MS),
                [this]() {
                    RCLCPP_INFO(this->get_logger(), " 1s after spawn — sending arm to 'safe_pick_arm'");
                    this->sendArmTrajectory("safe_pick_arm");
                    this->transitionTo(State::MOVE_ARM_SAFE_PICK);
                    this->timer_->cancel();
                });
            break;

        case State::MOVE_ARM_SAFE_PICK:
            RCLCPP_INFO(this->get_logger(), " Arm is moving to 'safe_pick_arm' (waiting for result...)");
            break;

        case State::START_VISION:
            RCLCPP_INFO(this->get_logger(), " Arm reached safe pose — starting vision detection");
            callStartDetection();
            transitionTo(State::WAIT_VISION_COMPLETE);
            break;

        case State::WAIT_VISION_COMPLETE:
            RCLCPP_INFO(this->get_logger(), " Waiting 2s for vision to compute...");
            timer_ = this->create_wall_timer(std::chrono::milliseconds(DELAY_2000MS),
                [this]() {
                    RCLCPP_INFO(this->get_logger(), " Vision wait done — calling /start_picking");
                    this->callStartPicking();
                    this->timer_->cancel();
                });
            break;

        case State::START_PICKING:
            RCLCPP_INFO(this->get_logger(), " Waiting for /start_picking to complete...");
            break;

        case State::ATTACH_OBJECT:
            RCLCPP_INFO(this->get_logger(), " Attaching object...");
            callAttachDetach(true);
            transitionTo(State::MOVE_ARM_HOME);
            break;

        case State::MOVE_ARM_HOME:
            RCLCPP_INFO(this->get_logger(), " Sending arm home (if not already active)...");
            if (!arm_goal_active_) {
                sendArmTrajectory("safe_pick_arm");
            }
            break;

        case State::NAV_TO_MIDDLE:
            RCLCPP_INFO(this->get_logger(), " Navigating to middle_amr");
            // navigateTo("middle_amr");
            transitionTo(State::NAV_TO_LIFT_0);
            break;

        case State::NAV_TO_LIFT_0:
            RCLCPP_INFO(this->get_logger(), " Navigating to lift_0_amr");
            navigateTo("lift_0_amr");
            transitionTo(State::WAIT_NAV_LIFT_0);
            break;

        case State::WAIT_NAV_LIFT_0:
            if (!navigation_complete_) {
                RCLCPP_INFO(this->get_logger(), " Waiting for navigation to lift_0...");
                return;
            }
            if (!navigation_succeeded_) {
                RCLCPP_ERROR(this->get_logger(), " Failed to reach lift_0_amr");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Reached lift_0_amr");
            transitionTo(State::CALL_ELEVATOR);
            break;

        case State::CALL_ELEVATOR:
            RCLCPP_INFO(this->get_logger(), " Calling elevator (SetBool true)");
            callSetBoolService("/elevator_cmd", true);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(DELAY_2000MS),
                [this]() {
                    this->transitionTo(State::SWITCH_FLOOR);
                    this->timer_->cancel();
                });
            break;

        case State::SWITCH_FLOOR:
            RCLCPP_INFO(this->get_logger(), "Switching to Floor 1");
            publishBoolTopic("/use_floor_1", true);
            delay_timer_ = this->create_wall_timer(std::chrono::milliseconds(DELAY_1000MS),
                [this]() {
                    RCLCPP_INFO(this->get_logger(), " Floor switch delay expired — navigating to drop_pre_amr");
                    this->transitionTo(State::NAV_TO_DROP_PRE_AMR);
                    if (this->delay_timer_) this->delay_timer_->cancel();
                });
            break;

        case State::NAV_TO_DROP_PRE_AMR:
            RCLCPP_INFO(this->get_logger(), " Navigating to drop_pre_amr");
            navigateTo("drop_pre_amr");
            transitionTo(State::WAIT_NAV_DROP_PRE_AMR);
            break;

        case State::WAIT_NAV_DROP_PRE_AMR:
            if (!navigation_complete_) {
                RCLCPP_INFO(this->get_logger(), " Waiting for navigation to drop_pre...");
                return;
            }
            if (!navigation_succeeded_) {
                RCLCPP_ERROR(this->get_logger(), " Failed to reach drop_pre_amr");
                return;
            }
            RCLCPP_INFO(this->get_logger(), " Reached drop_pre_amr");
            transitionTo(State::MOVE_ARM_DROP_PRE);
            break;

        case State::MOVE_ARM_DROP_PRE:
            RCLCPP_INFO(this->get_logger(), " Sending arm to 'drop_pre'");
            sendArmTrajectory("drop_pre");
            break;

        case State::MOVE_ARM_DROP:
            RCLCPP_INFO(this->get_logger(), " Sending arm to 'drop'");
            sendArmTrajectory("drop");
            break;

        case State::DETACH_OBJECT:
            RCLCPP_INFO(this->get_logger(), " Detaching object...");
            callAttachDetach(false);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(DELAY_500MS),
                [this]() {
                    this->transitionTo(State::MOVE_ARM_HOME_FINAL);
                    this->timer_->cancel();
                });
            break;

        case State::MOVE_ARM_HOME_FINAL:
            RCLCPP_INFO(this->get_logger(), " Final arm home movement (if not already active)...");
            if (!arm_goal_active_) {
                sendArmTrajectory("drop_pre");
            }
            break;

        case State::DONE:
            RCLCPP_INFO(this->get_logger(), " PIPELINE COMPLETED SUCCESSFULLY");
            //  End the analytics session
            if (analytics_session_id_ >= 0) {
                std::string end_time = getCurrentTimestamp();
                analytics_db_.endSession(analytics_session_id_, end_time);
                RCLCPP_INFO(this->get_logger(), "AnalyticsDB session ended.");
            }
            if (timer_) timer_->cancel();
            break;

        default:
            RCLCPP_WARN(this->get_logger(), " Unknown state: %d", static_cast<int>(current_state_));
            break;
    }
}

// IMPLEMENTATION OF HELPER FUNCTION
std::string PipelineFSM::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%F %T", &tm);
    return std::string(buf);
}