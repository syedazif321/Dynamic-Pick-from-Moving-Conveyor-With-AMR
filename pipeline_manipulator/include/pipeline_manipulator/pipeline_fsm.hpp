#ifndef PIPELINE_MANIPULATOR_PIPELINE_FSM_HPP
#define PIPELINE_MANIPULATOR_PIPELINE_FSM_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <msg_gazebo/srv/attach_detach.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <sqlite3.h>   
#include <atomic>
#include <thread>
#include <mutex>

using namespace std::chrono_literals;

// ----------------- AnalyticsDB Class Definition -----------------
class AnalyticsDB {
public:
    explicit AnalyticsDB(rclcpp::Logger logger, const std::string &db_path = "pipeline_analytics.db")
        : logger_(logger), db_path_(db_path), db_(nullptr) {
        open();
    }
    ~AnalyticsDB() {
        close();
    }

    bool open() {
        std::lock_guard<std::mutex> lk(m_);
        if (db_) return true;
        if (sqlite3_open(db_path_.c_str(), &db_) != SQLITE_OK) {
            RCLCPP_ERROR(logger_, "AnalyticsDB: cannot open DB %s: %s", db_path_.c_str(), sqlite3_errmsg(db_));
            sqlite3_close(db_);
            db_ = nullptr;
            return false;
        }
        exec("PRAGMA foreign_keys = ON;");

        // Table for Simulation Sessions
        exec("CREATE TABLE IF NOT EXISTS simulation_sessions ("
             "session_id INTEGER PRIMARY KEY AUTOINCREMENT,"
             "session_name TEXT NOT NULL,"
             "start_time TEXT NOT NULL,"
             "end_time TEXT);");

        // Table for Robot Goal Locations (from Targets.json)
        exec("CREATE TABLE IF NOT EXISTS robot_goals ("
             "goal_id INTEGER PRIMARY KEY AUTOINCREMENT,"
             "session_id INTEGER NOT NULL,"
             "goal_name TEXT NOT NULL,"
             "goal_type TEXT CHECK(goal_type IN ('amr_pose', 'arm_position')) NOT NULL,"
             "x REAL, y REAL, z REAL,"
             "ox REAL, oy REAL, oz REAL, ow REAL,"
             "FOREIGN KEY(session_id) REFERENCES simulation_sessions(session_id));");

        // Table for Box Analytics (spawned model_0)
        exec("CREATE TABLE IF NOT EXISTS box_analytics ("
             "box_id INTEGER PRIMARY KEY AUTOINCREMENT,"
             "session_id INTEGER NOT NULL,"
             "box_model TEXT NOT NULL," // e.g., "model_0"
             "color TEXT,"
             "size TEXT,"
             "spawn_time TEXT,"
             "detection_time TEXT,"
             "place_time TEXT,"
             "cycle_duration REAL,"
             "height REAL," // Assumed 0.1m unless specified otherwise
             "FOREIGN KEY(session_id) REFERENCES simulation_sessions(session_id));");

        // Table for Lift Usage Logs
        exec("CREATE TABLE IF NOT EXISTS lift_logs ("
             "log_id INTEGER PRIMARY KEY AUTOINCREMENT,"
             "session_id INTEGER NOT NULL,"
             "timestamp TEXT NOT NULL,"
             "action TEXT CHECK(action IN ('CALL', 'SWITCH_FLOOR')) NOT NULL,"
             "floor_target INTEGER,"
             "success BOOLEAN,"
             "FOREIGN KEY(session_id) REFERENCES simulation_sessions(session_id));");

        RCLCPP_INFO(logger_, "AnalyticsDB opened: %s", db_path_.c_str());
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> lk(m_);
        if (db_) {
            sqlite3_close(db_);
            db_ = nullptr;
            RCLCPP_INFO(logger_, "AnalyticsDB closed");
        }
    }

    int startSession(const std::string &session_name, const std::string &start_time) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return -1;
        std::string sql = "INSERT INTO simulation_sessions (session_name, start_time) VALUES ('" + escape(session_name) + "', '" + escape(start_time) + "');";
        if (!exec(sql)) return -1;
        int id = static_cast<int>(sqlite3_last_insert_rowid(db_));
        RCLCPP_INFO(logger_, "AnalyticsDB: started session %d (%s) @ %s", id, session_name.c_str(), start_time.c_str());
        return id;
    }

    bool endSession(int session_id, const std::string &end_time) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return false;
        std::string sql = "UPDATE simulation_sessions SET end_time='" + escape(end_time) + "' WHERE session_id=" + std::to_string(session_id) + ";";
        return exec(sql);
    }

    // Log ALL robot goals from Targets.json
    bool logAllGoals(int session_id, const nlohmann::json& targets_json) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_ || !targets_json.is_object()) return false;

        // Log AMR Poses
        if (targets_json.contains("amr_poses")) {
            for (auto& [name, pose_data] : targets_json["amr_poses"].items()) {
                double x = pose_data["position"][0];
                double y = pose_data["position"][1];
                double z = pose_data["position"][2];
                double ox = pose_data["orientation"][0];
                double oy = pose_data["orientation"][1];
                double oz = pose_data["orientation"][2];
                double ow = pose_data["orientation"][3];

                std::string sql = "INSERT INTO robot_goals (session_id, goal_name, goal_type, x, y, z, ox, oy, oz, ow) "
                                  "VALUES (" + std::to_string(session_id) + ", '" + escape(name) + "', 'amr_pose', "
                                  + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ", "
                                  + std::to_string(ox) + ", " + std::to_string(oy) + ", " + std::to_string(oz) + ", " + std::to_string(ow) + ");";
                if (!exec(sql)) {
                    RCLCPP_WARN(logger_, "Failed to log AMR goal: %s", name.c_str());
                }
            }
        }

        // Log Arm Positions
        if (targets_json.contains("arm_positions")) {
            for (auto& [name, joint_values] : targets_json["arm_positions"].items()) {
                std::string sql = "INSERT INTO robot_goals (session_id, goal_name, goal_type) "
                                  "VALUES (" + std::to_string(session_id) + ", '" + escape(name) + "', 'arm_position');";
                if (!exec(sql)) {
                    RCLCPP_WARN(logger_, "Failed to log ARM goal: %s", name.c_str());
                }
            }
        }

        RCLCPP_INFO(logger_, "AnalyticsDB: Logged %zu robot goals.",
                    (targets_json.contains("amr_poses") ? targets_json["amr_poses"].size() : 0) +
                    (targets_json.contains("arm_positions") ? targets_json["arm_positions"].size() : 0));
        return true;
    }

    // Log when a box is spawned (assume fixed height of 0.1m)
    bool logBoxSpawn(int session_id, const std::string& box_model, const std::string& color, double height) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return false;
        std::string timestamp = getCurrentTimestamp();
        std::string sql = "INSERT INTO box_analytics (session_id, box_model, color, height, spawn_time) "
                          "VALUES (" + std::to_string(session_id) + ", '" + escape(box_model) + "', '" + escape(color) + "', " + 
                          std::to_string(height) + ", '" + escape(timestamp) + "');";
        return exec(sql);
    }

    // Log vision detection time
    bool logBoxDetection(int session_id, const std::string& box_model, const std::string& color, const std::string& size) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return false;
        std::string timestamp = getCurrentTimestamp();
        std::string sql = "UPDATE box_analytics SET detection_time='" + escape(timestamp) + "', color='" + escape(color) + "', size='" + escape(size) + "' "
                          "WHERE box_model='" + escape(box_model) + "' AND spawn_time IS NOT NULL AND detection_time IS NULL;";
        return exec(sql);
    }

    // Log when box is placed (detached)
    bool logBoxPlace(int session_id, const std::string& box_model) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return false;
        std::string timestamp = getCurrentTimestamp();
        std::string sql_upd = "UPDATE box_analytics SET place_time='" + escape(timestamp) + "' "
                              "WHERE box_model='" + escape(box_model) + "' AND spawn_time IS NOT NULL AND place_time IS NULL;";
        if (!exec(sql_upd)) return false;
        // Calculate cycle duration (seconds)
        std::string sql_calc = "UPDATE box_analytics SET cycle_duration=(julianday(place_time)-julianday(spawn_time))*86400.0 "
                               "WHERE box_model='" + escape(box_model) + "' AND spawn_time IS NOT NULL AND place_time IS NOT NULL;";
        return exec(sql_calc);
    }

    // Log elevator call and floor switch
    bool logLiftUsage(int session_id, const std::string& action, int floor_target, bool success) {
        std::lock_guard<std::mutex> lk(m_);
        if (!db_) return false;
        std::string timestamp = getCurrentTimestamp();
        std::string sql = "INSERT INTO lift_logs (session_id, timestamp, action, floor_target, success) "
                          "VALUES (" + std::to_string(session_id) + ", '" + escape(timestamp) + "', '" + escape(action) + "', " + 
                          std::to_string(floor_target) + ", " + (success ? "1" : "0") + ");";
        return exec(sql);
    }

private:
    bool exec(const std::string &sql) {
        if (!db_) return false;
        char *errmsg = nullptr;
        if (sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &errmsg) != SQLITE_OK) {
            RCLCPP_ERROR(logger_, "AnalyticsDB SQL error: %s", errmsg ? errmsg : "(unknown)");
            if (errmsg) sqlite3_free(errmsg);
            return false;
        }
        return true;
    }

    std::string escape(const std::string &s) const {
        std::string out;
        for (char c : s) {
            if (c == '\'') out += "''";
            else out += c;
        }
        return out;
    }

    std::string getCurrentTimestamp() const {
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

    rclcpp::Logger logger_;
    std::string db_path_;
    sqlite3 *db_;
    std::mutex m_;
};

// ----------------- PipelineFSM Class -----------------
class PipelineFSM : public rclcpp::Node
{
public:
    enum class State {
        IDLE,
        NAV_TO_PICK_PRE,
        WAIT_NAV_PICK_PRE,
        SLIDER_EXTEND,
        SPAWN_BOX,
        WAIT_AFTER_SPAWN,
        MOVE_ARM_SAFE_PICK,
        START_VISION,
        WAIT_VISION_COMPLETE,
        START_PICKING,
        ATTACH_OBJECT,
        MOVE_ARM_HOME,
        NAV_TO_MIDDLE,
        NAV_TO_LIFT_0,
        WAIT_NAV_LIFT_0,
        CALL_ELEVATOR,
        SWITCH_FLOOR,
        NAV_TO_DROP_PRE_AMR,
        WAIT_NAV_DROP_PRE_AMR,
        MOVE_ARM_DROP_PRE,
        MOVE_ARM_DROP,
        DETACH_OBJECT,
        MOVE_ARM_HOME_FINAL,
        DONE
    };

    PipelineFSM();

private:
    void runFSM();
    void transitionTo(State next); 
    void loadTargets();
    void navigateTo(const std::string& target_name);
    void sendSliderTrajectory();
    void callTriggerService(const std::string& service_name);
    void callStartDetection();
    void callStartPicking();
    void callAttachDetach(bool attach);
    void callSetBoolService(const std::string& service_name, bool data);
    void publishBoolTopic(const std::string& topic_name, bool data);
    void sendArmTrajectory(const std::string& joint_target_name);

    State current_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr delay_timer_;

    // Action clients
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr slider_client_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;

    // Service clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr spawn_box_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_picking_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_detection_client_;
    rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedPtr attach_detach_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr elevator_client_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr floor_pub_;

    bool navigation_goal_sent_ = false;
    bool navigation_complete_ = false;
    bool navigation_succeeded_ = false;
    bool slider_goal_active_ = false;
    bool arm_goal_active_ = false;

    // Loaded targets
    nlohmann::json targets_json_;

    // Delays (in ms)
    const int DELAY_500MS = 500;
    const int DELAY_1000MS = 1000;
    const int DELAY_2000MS = 2000;
    const int DELAY_3000MS = 3000;
    const int DELAY_200MS = 200;

    AnalyticsDB analytics_db_;             
    int analytics_session_id_{-1};        
    std::string getCurrentTimestamp() const; 
};

#endif // PIPELINE_MANIPULATOR_PIPELINE_FSM_HPP