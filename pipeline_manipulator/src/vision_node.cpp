// File: /home/azif/projetcs/Dynamic-Pick-from-Moving-Conveyor-With-AMR/pipeline_manipulator/src/vision_node.cpp

#include "msg_gazebo/msg/box_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp> 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp> 
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <optional>
#include <random>
#include <iomanip>
#include <sstream>
#include <deque>
#include <map> 
#include <utility> // For std::pair

using namespace std::chrono_literals;

struct PoseHistory
{
    rclcpp::Time timestamp;
    tf2::Vector3 position;
    tf2::Quaternion orientation;
};

class PoseEstimator
{
public:
    PoseEstimator(int history_size = 5) : history_size_(history_size) {}

    void update_pose(const geometry_msgs::msg::PoseStamped& pose_msg)
    {
        PoseHistory current;
        current.timestamp = pose_msg.header.stamp;
        tf2::fromMsg(pose_msg.pose.position, current.position);
        tf2::fromMsg(pose_msg.pose.orientation, current.orientation);
        current.orientation.normalize();

        history_.push_back(current);
        if (history_.size() > history_size_) {
            history_.pop_front();
        }

        if (history_.size() >= 2)
        {
            const auto& start = history_.front();
            const auto& end = history_.back();

            double dt = (end.timestamp - start.timestamp).seconds();
            if (dt > 1e-6)
            {
                // Linear Velocity
                tf2::Vector3 linear_vel_tf = (end.position - start.position) / dt;
                linear_vel_.x = linear_vel_tf.x();
                linear_vel_.y = linear_vel_tf.y();
                linear_vel_.z = linear_vel_tf.z();

                // Angular Velocity (Simplified RPY difference)
                tf2::Quaternion rot_diff = start.orientation.inverse() * end.orientation;
                tf2::Matrix3x3 rot_mat(rot_diff);
                double r, p, y;
                rot_mat.getRPY(r, p, y);

                angular_vel_.x = r / dt;
                angular_vel_.y = p / dt;
                angular_vel_.z = y / dt;
            }
        }
    }

    geometry_msgs::msg::TwistStamped get_velocity(const std::string& frame_id, const rclcpp::Time& timestamp) const
    {
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = timestamp;
        twist.header.frame_id = frame_id;
        twist.twist.linear = linear_vel_;
        twist.twist.angular = angular_vel_;
        return twist;
    }
private:
    std::deque<PoseHistory> history_;
    size_t history_size_;
    geometry_msgs::msg::Vector3 linear_vel_ = geometry_msgs::msg::Vector3();
    geometry_msgs::msg::Vector3 angular_vel_ = geometry_msgs::msg::Vector3();
};
// --- End PoseEstimator ---

class DynamicDetector : public rclcpp::Node
{
public:
    DynamicDetector()
        : Node("dynamic_detector_node"),
        fx_(declare_parameter("fx", 554.3827)),
        fy_(declare_parameter("fy", 554.3827)),
        cx_(declare_parameter("cx", 320.5)),
        cy_(declare_parameter("cy", 240.5)),
        

        marker_id_(declare_parameter("marker_id", 0)), 
        marker_size_m_(declare_parameter("marker_size_m", 0.0789)), 

        camera_frame_(declare_parameter("camera_frame", "camera_rgb_optical_frame")),
        base_frame_(declare_parameter("base_frame", "base_link")),
        world_frame_(declare_parameter("world_frame", "odom")), 
        
        // Orientation Offsets (KEEP)
        roll_offset_deg_(declare_parameter("roll_offset_deg", 180.0)),
        pitch_offset_deg_(declare_parameter("pitch_offset_deg", 0.0)),
        yaw_offset_deg_(declare_parameter("yaw_offset_deg", 90.0)),
        estimator_(/* history size */ 5)
        
    {
        // Publishers
        box_state_pub_ = create_publisher<msg_gazebo::msg::BoxState>("/box_state_dynamic", 10);
        info_pub_ = create_publisher<std_msgs::msg::String>("/detected_box_info", 10);
        
        z_offset_m_ = declare_parameter("z_offset_m", -0.01); 
        
        // Velocity-to-offset map: Key: AMR Linear X Velocity (m/s), Value: {X Offset (m), Y Offset (m)}
        vel_xy_offset_map_ = {
            {-0.10, {-0.0, -0.34}},
            {-0.15, {-0.0, -0.22}},
            {-0.20, {0.0, -0.38}},
            {-0.25, {0.0, -0.25}}, 
            {-0.30, {0.0, -0.26}},
        };
        current_amr_vel_x_ = declare_parameter("current_amr_vel_x", 0.0);
        
        // Subscriptions
        rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera_rgb/rgb_camera/image_raw");
        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera_depth/depth_camera/depth/image_raw");
        
        // Subscriber for AMR velocity
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&DynamicDetector::cmdVelCb, this, std::placeholders::_1));

        tf_buffer_ = std::shared_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(get_clock())); 
        tf_listener_ = std::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tf_buffer_)); 
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);
        sync_->registerCallback(std::bind(&DynamicDetector::imageCb, this, std::placeholders::_1, std::placeholders::_2));

        aruco_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        aruco_parameters_ = cv::aruco::DetectorParameters::create();

        RCLCPP_INFO(get_logger(), "DynamicDetector ready. Using ArUco DICT_4X4_50 and marker ID: %d", marker_id_);
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    
    // /cmd_vel callback to track AMR speed
    void cmdVelCb(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
    {
        current_amr_vel_x_ = msg->linear.x;
        RCLCPP_DEBUG(get_logger(), "AMR Speed updated: %.2f m/s", current_amr_vel_x_);
    }

    // Function to get the required X and Y offsets based on AMR velocity
    std::pair<double, double> getPredictionOffsets(double amr_vel_x)
    {
        for (const auto& pair : vel_xy_offset_map_) {
            // Check for close match, useful if cmd_vel isn't exactly the map key
            if (std::abs(pair.first - amr_vel_x) < 1e-4) {
                // Return {X_offset, Y_offset}
                return pair.second;
            }
        }
        
        // Fallback: If the speed isn't in the map, use {0.0, 0.0} offset (safe default)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
            "AMR speed (%.2f) not found in offset map. Using {0.0, 0.0} offsets.", amr_vel_x);
        return {0.0, 0.0};
    }

    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                 const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        // Convert images SAFELY
        cv::Mat rgb;
        try {
            rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "CV_BRIDGE ERROR: %s", e.what());
            return;
        }

        if (rgb.empty()) {
            RCLCPP_WARN(get_logger(), "Empty image.");
            return;
        }
        
        cv::Mat gray;
        cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);

        // --- 1. ARUCO MARKER DETECTION ---
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
        cv::aruco::detectMarkers(gray, aruco_dictionary_, marker_corners, marker_ids, aruco_parameters_, rejected_candidates);

        std::optional<geometry_msgs::msg::PoseStamped> best_pose_camera;

        if (!marker_ids.empty()) {
            
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
            cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F); 
            
            cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_m_, camera_matrix, dist_coeffs, rvecs, tvecs);

            for (size_t i = 0; i < marker_ids.size(); ++i) {
                if (marker_ids[i] == marker_id_) {
                    
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvecs[i], rotation_matrix);

                    tf2::Matrix3x3 R_tf(
                        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
                    );
                    tf2::Quaternion q_raw;
                    R_tf.getRotation(q_raw);
                    q_raw.normalize();

                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = rgb_msg->header;
                    pose.header.frame_id = camera_frame_;
                    pose.pose.position.x = tvecs[i][0];
                    pose.pose.position.y = tvecs[i][1];
                    pose.pose.position.z = tvecs[i][2];

                    pose.pose.position.z += z_offset_m_; 
                    pose.pose.orientation = tf2::toMsg(q_raw);
                    
                    best_pose_camera = pose;
                    
                    cv::aruco::drawAxis(rgb, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_size_m_ * 0.5);

                    break;
                }
            }
        }

        cv::aruco::drawDetectedMarkers(rgb, marker_corners, marker_ids, cv::Scalar(0, 255, 0));

        if (best_pose_camera) {
        
            geometry_msgs::msg::TransformStamped T_cw; // Camera in World (World is the target for the pose)
            
            try {
                // T_cw: Transform from camera_frame to world_frame at the time of image capture
                T_cw = tf_buffer_->lookupTransform(world_frame_, camera_frame_, rgb_msg->header.stamp, 100ms);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
                cv::imshow("Detection", rgb);
                cv::waitKey(1);
                return;
            }
            
            // 2. Apply Orientation Offset
            tf2::Quaternion q_orig;
            tf2::fromMsg(best_pose_camera->pose.orientation, q_orig);

            double roll_rad = roll_offset_deg_ * M_PI / 180.0;
            double pitch_rad = pitch_offset_deg_ * M_PI / 180.0;
            double yaw_rad = yaw_offset_deg_ * M_PI / 180.0;

            tf2::Quaternion q_offset;
            q_offset.setRPY(roll_rad, pitch_rad, yaw_rad);

            tf2::Quaternion q_final = q_orig * q_offset;
            q_final.normalize();
            best_pose_camera->pose.orientation = tf2::toMsg(q_final);
            
            // 3. Transform Pose to World Frame
            geometry_msgs::msg::PoseStamped pose_world_raw;
            tf2::doTransform(*best_pose_camera, pose_world_raw, T_cw);
            
            // 4. Estimate Velocity 
            estimator_.update_pose(pose_world_raw); 
            geometry_msgs::msg::TwistStamped vel_world = 
                estimator_.get_velocity(world_frame_, pose_world_raw.header.stamp);

            // 5. Apply Dynamic Prediction Offsets 
            geometry_msgs::msg::PoseStamped pose_world = pose_world_raw; 

            std::pair<double, double> offsets = getPredictionOffsets(current_amr_vel_x_);
            double x_offset = offsets.first;
            double y_offset = offsets.second;
            
            pose_world.pose.position.x += x_offset; 
            pose_world.pose.position.y += y_offset; 

            pose_world.header.frame_id = world_frame_;
            pose_world.header.stamp = this->now();
            
            // 6. Publish Custom Message
            auto box_state_msg = msg_gazebo::msg::BoxState();
            box_state_msg.header = pose_world.header;
            box_state_msg.pose_world = pose_world;      
            box_state_msg.velocity_world = vel_world;   
            box_state_pub_->publish(box_state_msg);
            
            // 7. Broadcast TF for Visualization 
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = pose_world.header;
            tf_msg.child_frame_id = "detected_box"; 
            tf_msg.transform.translation.x = pose_world.pose.position.x;
            tf_msg.transform.translation.y = pose_world.pose.position.y;
            tf_msg.transform.translation.z = pose_world.pose.position.z;
            tf_msg.transform.rotation = pose_world.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);

            // 8. Publish Info (Debug) - Using ANSI Escape Codes for RED
            const std::string RED = "\033[31m";
            const std::string RESET = "\033[0m";

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3)
                << "Box Detected (WORLD) | AMR Vel X: " << current_amr_vel_x_ << " m/s\n"
                << "  " << RED << "*Applied X Offset: " << x_offset << " [m]" << RESET << "\n" 
                << "  " << RED << "*Applied Y Offset: " << y_offset << " [m]" << RESET << "\n" 
                << "  Raw Pos (X, Y, Z): " << pose_world_raw.pose.position.x << ", " 
                << pose_world_raw.pose.position.y << ", " << pose_world_raw.pose.position.z << " [m]\n"
                << "  Target Pos (X, Y, Z): " << pose_world.pose.position.x << ", " 
                << pose_world.pose.position.y << ", " << pose_world.pose.position.z << " [m]\n"
                << "  Vel (VX, VY, VZ): " << vel_world.twist.linear.x << ", "
                << vel_world.twist.linear.y << ", " << vel_world.twist.linear.z << " [m/s]";
            
            auto info_msg = std_msgs::msg::String();
            info_msg.data = oss.str();
            info_pub_->publish(info_msg);
            
            // Use RCLCPP_INFO_STREAM to ensure the output is visible by default
            RCLCPP_INFO_STREAM(get_logger(), "\n" << oss.str());
        }

        // 9. Visualization
        cv::imshow("Detection", rgb);
        cv::waitKey(1);
    }

    // --- Member Variables ---
    double fx_, fy_, cx_, cy_;
    
    // ArUco Parameters
    int marker_id_;
    double marker_size_m_;
    double z_offset_m_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_;
    
    std::string camera_frame_, base_frame_, world_frame_;
    double roll_offset_deg_, pitch_offset_deg_, yaw_offset_deg_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_, depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; 
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<msg_gazebo::msg::BoxState>::SharedPtr box_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
    
    PoseEstimator estimator_;
    
    std::map<double, std::pair<double, double>> vel_xy_offset_map_;
    double current_amr_vel_x_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicDetector>());
    rclcpp::shutdown();
    return 0;
}