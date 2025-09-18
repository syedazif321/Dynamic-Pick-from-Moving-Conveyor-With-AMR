#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
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

class BoxDetector : public rclcpp::Node
{
public:
    BoxDetector()
        : Node("box_detector_node"),
        fx_(declare_parameter("fx", 554.3827)),
        fy_(declare_parameter("fy", 554.3827)),
        cx_(declare_parameter("cx", 320.5)),
        cy_(declare_parameter("cy", 240.5)),
        min_area_px_(declare_parameter("min_area_px", 500.0)),
        square_tolerance_(declare_parameter("square_tolerance", 0.12)),
        camera_normal_cos_thresh_(declare_parameter("camera_normal_cos_thresh", 0.3)),
        plane_inlier_thresh_(declare_parameter("plane_inlier_thresh", 0.006)),
        ransac_iters_(declare_parameter("ransac_iters", 120)),
        max_sample_points_(declare_parameter("max_sample_points", 4000)),
        camera_frame_(declare_parameter("camera_frame", "camera_rgb_optical_frame")),
        base_frame_(declare_parameter("base_frame", "base_link")),
          // >>>>>>>>>>>>>>>>>>>> ADD RPY OFFSET PARAMETERS <<<<<<<<<<<<<<<<<<<<<
        roll_offset_deg_(declare_parameter("roll_offset_deg", 180.0)),
        pitch_offset_deg_(declare_parameter("pitch_offset_deg", 0.0)),
        yaw_offset_deg_(declare_parameter("yaw_offset_deg", 90.0)),
          // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        detection_active_(false),
        has_detected_(false),
        cycle_id_(0)
    {
        // Publishers
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/detected_box_pose", 10);
        info_pub_ = create_publisher<std_msgs::msg::String>("/detected_box_info", 10);

        rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera_rgb/rgb_camera/image_raw");

        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera_depth/depth_camera/depth/image_raw");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);

        // Services
        start_srv_ = create_service<std_srvs::srv::Trigger>(
            "/start_detection",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                std::lock_guard<std::mutex> lk(mutex_);
                if (detection_active_) {
                    resp->success = false;
                    resp->message = "Detection already running.";
                    return;
                }
                cycle_id_++;
                detection_active_ = true;
                has_detected_ = false;
                frozen_pose_.reset();
                sync_->registerCallback(std::bind(&BoxDetector::imageCb, this, std::placeholders::_1, std::placeholders::_2));
                resp->success = true;
                resp->message = "Detection started.";
                RCLCPP_INFO(get_logger(), "Detection started (cycle %lu).", static_cast<unsigned long>(cycle_id_));
            });

        stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "/stop_detection",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
                std::lock_guard<std::mutex> lk(mutex_);
                if (!detection_active_) {
                    resp->success = false;
                    resp->message = "No active detection.";
                    return;
                }
                detection_active_ = false;
                sync_.reset();
                sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);
                resp->success = true;
                resp->message = "Detection stopped.";
                RCLCPP_INFO(get_logger(), "Detection stopped (cycle %lu).", static_cast<unsigned long>(cycle_id_));
            });

        RCLCPP_INFO(get_logger(), "BoxDetector ready. Call /start_detection to begin.");
        RCLCPP_INFO(get_logger(), "Tune TF orientation live with:");
        RCLCPP_INFO(get_logger(), "  ros2 param set /box_detector_node roll_offset_deg  VALUE");
        RCLCPP_INFO(get_logger(), "  ros2 param set /box_detector_node pitch_offset_deg VALUE");
        RCLCPP_INFO(get_logger(), "  ros2 param set /box_detector_node yaw_offset_deg   VALUE");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    struct PlaneModel {
        cv::Vec3f n;
        float d;
    };

    static bool planeFrom3(const cv::Vec3f& a, const cv::Vec3f& b, const cv::Vec3f& c, PlaneModel& out)
    {
        cv::Vec3f n = (b - a).cross(c - a);
        float nn = std::sqrt(n.dot(n));
        if (nn < 1e-6f) return false;
        n = n * (1.0f / nn);
        float d = -n.dot(a);
        out.n = n; out.d = d;
        return true;
    }

    static inline float pointPlaneDist(const cv::Vec3f& p, const PlaneModel& pl)
    { return std::fabs(pl.n.dot(p) + pl.d); }

    cv::Mat createBoxMask(const cv::Mat& hsv) {
        cv::Mat red1, red2, blue, out;
        cv::inRange(hsv, cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255), red1);
        cv::inRange(hsv, cv::Scalar(160, 100, 50), cv::Scalar(180, 255, 255), red2);
        cv::inRange(hsv, cv::Scalar(100, 100, 50), cv::Scalar(140, 255, 255), blue);
        cv::bitwise_or(red1, red2, out);
        cv::bitwise_or(out, blue, out);
        return out;
    }

    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                 const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        // >>>>>>>>>>>>>>>>>>>> DEBUG: LOG INCOMING ENCODINGS <<<<<<<<<<<<<<<<<<<<<
        RCLCPP_DEBUG(get_logger(), "RGB encoding: %s | Depth encoding: %s",
                     rgb_msg->encoding.c_str(), depth_msg->encoding.c_str());

        if (!detection_active_.load()) return;

        // If already detected, just re-broadcast TF
        if (has_detected_.load() && frozen_pose_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = this->now();
            tf_msg.header.frame_id = base_frame_;
            tf_msg.child_frame_id = "detected_box";

            tf_msg.transform.translation.x = frozen_pose_->pose.position.x;
            tf_msg.transform.translation.y = frozen_pose_->pose.position.y;
            tf_msg.transform.translation.z = frozen_pose_->pose.position.z;

            tf2::Quaternion q_orig;
            tf2::fromMsg(frozen_pose_->pose.orientation, q_orig);

            double roll_rad = roll_offset_deg_ * M_PI / 180.0;
            double pitch_rad = pitch_offset_deg_ * M_PI / 180.0;
            double yaw_rad = yaw_offset_deg_ * M_PI / 180.0;

            tf2::Quaternion q_offset;
            q_offset.setRPY(roll_rad, pitch_rad, yaw_rad);

            tf2::Quaternion q_final = q_orig * q_offset;
            q_final.normalize();

            tf_msg.transform.rotation = tf2::toMsg(q_final);

            tf_broadcaster_->sendTransform(tf_msg);

            auto republished_pose = *frozen_pose_;
            republished_pose.header.stamp = tf_msg.header.stamp;
            republished_pose.pose.orientation = tf_msg.transform.rotation;
            pose_pub_->publish(republished_pose);

            // Convert images SAFELY
            cv::Mat rgb, depth32;
            try {
                rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
                depth32 = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(get_logger(), " CV_BRIDGE ERROR in FROZEN branch: %s", e.what());
                return;
            }

            cv::imshow("Detection", rgb);
            if (!depth32.empty()) {
                cv::Mat depth_vis;
                depth32.convertTo(depth_vis, CV_8UC1, 255.0 / 5.0);
                cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
                cv::imshow("Depth View", depth_vis);
            }
            cv::waitKey(1);
            return;
        }

        // Convert images SAFELY — MAIN DETECTION BRANCH
        cv::Mat rgb, depth32;
        try {
            rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
            depth32 = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "CV_BRIDGE ERROR in MAIN branch: %s", e.what());
            return;
        }

        if (rgb.empty() || depth32.empty()) {
            RCLCPP_WARN(get_logger(), "Empty image or depth.");
            return;
        }

        cv::Mat hsv, mask;
        cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
        mask = createBoxMask(hsv);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty()) {
            RCLCPP_DEBUG(get_logger(), "No contours found.");
            return;
        }

        geometry_msgs::msg::TransformStamped T_cb;
        try {
            T_cb = tf_buffer_->lookupTransform(base_frame_, camera_frame_, rclcpp::Time(0), tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
            return;
        }
        tf2::Transform tf_cb;
        tf2::fromMsg(T_cb.transform, tf_cb);

        tf2::Vector3 cam_dir_in_cam(0, 0, 1);
        tf2::Vector3 cam_dir_in_base = tf_cb * cam_dir_in_cam;
        cam_dir_in_base.normalize();

        static thread_local std::mt19937 rng{std::random_device{}()};

        std::optional<geometry_msgs::msg::PoseStamped> best_pose;
        std::optional<std::string> best_info_str;
        double best_dist = 1e9;

        for (const auto& cnt : contours) {
            double area_px = cv::contourArea(cnt);
            if (area_px < min_area_px_) continue;

            cv::Rect bb = cv::boundingRect(cnt);
            std::vector<cv::Point> pxs;
            for (int y = bb.y; y < bb.y + bb.height; ++y) {
                const uint8_t* mrow = mask.ptr<uint8_t>(y);
                for (int x = bb.x; x < bb.x + bb.width; ++x) {
                    if (mrow[x]) pxs.emplace_back(x, y);
                }
            }
            if (pxs.size() < 400) continue;

            std::vector<cv::Vec3f> pts_base;
            std::vector<cv::Point> pxs_valid;
            pts_base.reserve(pxs.size());
            pxs_valid.reserve(pxs.size());

            for (const auto& p : pxs) {
                if (p.y >= depth32.rows || p.x >= depth32.cols || p.y < 0 || p.x < 0) continue;
                float z = depth32.at<float>(p.y, p.x);
                if (!std::isfinite(z) || z < 0.2f || z > 5.0f) continue;

                float X_cam = (p.x - cx_) * z / fx_;
                float Y_cam = (p.y - cy_) * z / fy_;
                tf2::Vector3 point_in_cam(X_cam, Y_cam, z);
                tf2::Vector3 point_in_base = tf_cb * point_in_cam;

                pts_base.emplace_back(
                    static_cast<float>(point_in_base.x()),
                    static_cast<float>(point_in_base.y()),
                    static_cast<float>(point_in_base.z())
                );
                pxs_valid.emplace_back(p);
            }

            if (pts_base.size() < 400) continue;

            PlaneModel best_plane;
            size_t best_inliers = 0;
            std::vector<int> best_idxs;
            std::uniform_int_distribution<int> uni(0, static_cast<int>(pts_base.size()) - 1);

            for (int it = 0; it < ransac_iters_; ++it) {
                int i1 = uni(rng), i2 = uni(rng), i3 = uni(rng);
                if (i1 == i2 || i1 == i3 || i2 == i3) { --it; continue; }

                PlaneModel pl;
                if (!planeFrom3(pts_base[i1], pts_base[i2], pts_base[i3], pl)) continue;

                tf2::Vector3 n_vec(pl.n[0], pl.n[1], pl.n[2]);
                double cos_cam = n_vec.dot(cam_dir_in_base);
                if (cos_cam < camera_normal_cos_thresh_) continue;

                std::vector<int> inliers;
                inliers.reserve(pts_base.size());
                for (int idx = 0; idx < static_cast<int>(pts_base.size()); ++idx) {
                    if (pointPlaneDist(pts_base[idx], pl) <= static_cast<float>(plane_inlier_thresh_)) {
                        inliers.push_back(idx);
                    }
                }
                if (inliers.size() > best_inliers) {
                    best_inliers = inliers.size();
                    best_plane = pl;
                    best_idxs = inliers;
                }
            }

            if (best_inliers < 200) continue;

            std::vector<cv::Vec3f> inliers3;
            inliers3.reserve(best_idxs.size());
            for (int idx : best_idxs) inliers3.push_back(pts_base[idx]);

            cv::Mat data(static_cast<int>(inliers3.size()), 3, CV_32F);
            for (size_t i = 0; i < inliers3.size(); ++i) {
                data.at<float>(static_cast<int>(i), 0) = inliers3[i][0];
                data.at<float>(static_cast<int>(i), 1) = inliers3[i][1];
                data.at<float>(static_cast<int>(i), 2) = inliers3[i][2];
            }
            cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);

            auto nrm = [](cv::Vec3f v) {
                float n = std::sqrt(v.dot(v));
                return (n > 1e-9f) ? v * (1.f / n) : v;
            };

            cv::Vec3f e0p = nrm(pca.eigenvectors.row(0));
            cv::Vec3f e1p = nrm(pca.eigenvectors.row(1));
            cv::Vec3f e2 = nrm(pca.eigenvectors.row(2));

            tf2::Vector3 e2_tf(e2[0], e2[1], e2[2]);
            double dot_cam = e2_tf.dot(cam_dir_in_base);
            if (dot_cam > 0) {
                e2 = -e2;
            }

            e0p = nrm(e0p - e2 * (e0p.dot(e2)));
            e1p = nrm(e2.cross(e0p));

            cv::Vec3f C = pca.mean;

            std::vector<cv::Point2f> uv;
            uv.reserve(inliers3.size());
            for (const auto& P : inliers3) {
                cv::Vec3f d = P - C;
                uv.emplace_back(d.dot(e0p), d.dot(e1p));
            }

            std::vector<int> hull_idx;
            cv::convexHull(uv, hull_idx, true, false);
            std::vector<cv::Point2f> hull;
            hull.reserve(hull_idx.size());
            for (int i : hull_idx) hull.push_back(uv[i]);

            if (hull.size() < 4) continue;

            double peri = cv::arcLength(hull, true);
            std::vector<cv::Point2f> poly;
            cv::approxPolyDP(hull, poly, 0.02 * peri, true);
            if (poly.size() != 4) continue;

            auto angle_ok = [](const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
                cv::Point2f v1 = a - b, v2 = c - b;
                double n1 = std::hypot(v1.x, v1.y), n2 = std::hypot(v2.x, v2.y);
                if (n1 < 1e-6 || n2 < 1e-6) return false;
                double cosang = std::fabs(v1.x * v2.x + v1.y * v2.y) / (n1 * n2);
                return cosang < 0.20;
            };

            bool right = angle_ok(poly[3], poly[0], poly[1]) &&
                         angle_ok(poly[0], poly[1], poly[2]) &&
                         angle_ok(poly[1], poly[2], poly[3]) &&
                         angle_ok(poly[2], poly[3], poly[0]);
            if (!right) continue;

            auto seglen = [](const cv::Point2f& a, const cv::Point2f& b) {
                return std::hypot(a.x - b.x, a.y - b.y);
            };
            double s0 = seglen(poly[0], poly[1]);
            double s1 = seglen(poly[1], poly[2]);
            double s2 = seglen(poly[2], poly[3]);
            double s3 = seglen(poly[3], poly[0]);
            double L = 0.5 * (s0 + s2);
            double W = 0.5 * (s1 + s3);
            double max_side = std::max(L, W);
            if (max_side < 1e-6) continue;
            if (std::fabs(L - W) / max_side > square_tolerance_) continue;

            float thickness = 0.0f;
            if (!pxs_valid.empty()) {
                std::vector<float> z_vals;
                z_vals.reserve(pxs_valid.size());
                for (const auto& p : pxs_valid) {
                    float z = depth32.at<float>(p.y, p.x);
                    if (std::isfinite(z)) z_vals.push_back(z);
                }
                if (!z_vals.empty()) {
                    std::sort(z_vals.begin(), z_vals.end());
                    float z_front = z_vals.front();
                    float z_back = z_vals.back();
                    thickness = std::max(0.0f, z_back - z_front);
                }
            }

            cv::Point2f e_uv;
            if (L >= W) {
                e_uv = poly[1] - poly[0];
            } else {
                e_uv = poly[2] - poly[1];
            }

            double e_norm = std::hypot(e_uv.x, e_uv.y);
            if (e_norm < 1e-9) continue;
            e_uv.x /= e_norm; e_uv.y /= e_norm;

            cv::Vec3f e0 = nrm(e0p * static_cast<float>(e_uv.x) + e1p * static_cast<float>(e_uv.y));
            cv::Vec3f e1 = nrm(e2.cross(e0));

            tf2::Matrix3x3 R(
                e0[0], e1[0], e2[0],
                e0[1], e1[1], e2[1],
                e0[2], e1[2], e2[2]
            );

            if (R.determinant() < 0) {
                e1 = -e1;
                R = tf2::Matrix3x3(e0[0], e1[0], e2[0],
                                  e0[1], e1[1], e2[1],
                                  e0[2], e1[2], e2[2]);
            }

            tf2::Quaternion q;
            R.getRotation(q);
            q.normalize();

            geometry_msgs::msg::PoseStamped pose;
            pose.header = rgb_msg->header;
            pose.header.frame_id = base_frame_;
            pose.pose.position.x = C[0];
            pose.pose.position.y = C[1];
            pose.pose.position.z = C[2];
            pose.pose.orientation = tf2::toMsg(q);

            std::string color = "unknown";
            cv::Rect roi = cv::boundingRect(cnt);
            cv::Scalar mean_hsv = cv::mean(hsv(roi), mask(roi));
            if (mean_hsv[0] < 20 || mean_hsv[0] > 170) {
                color = "red";
            } else if (mean_hsv[0] > 90 && mean_hsv[0] < 135) {
                color = "blue";
            }

            double roll, pitch, yaw;
            tf2::Matrix3x3(tf2::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                           pose.pose.orientation.z, pose.pose.orientation.w))
                .getRPY(roll, pitch, yaw);

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3)
                << "Box Detected (RAW):\n"
                << "  Color: " << color << "\n"
                << "  Size (L x W): " << L << "m x " << W << "m\n"
                << "  Estimated Thickness: " << thickness << "m\n"
                << "  Position (X, Y, Z): " << C[0] << ", " << C[1] << ", " << C[2] << " [m]\n"
                << "  Orientation (R, P, Y): "
                << roll * 180 / M_PI << ", "
                << pitch * 180 / M_PI << ", "
                << yaw * 180 / M_PI << " [deg]\n"
                << "  Area (pixels): " << area_px << "\n"
                << "  Distance: " << std::sqrt(C[0]*C[0] + C[1]*C[1] + C[2]*C[2]) << " [m]\n"
                << "  Timestamp: " << rgb_msg->header.stamp.sec << "."
                << std::setfill('0') << std::setw(9) << rgb_msg->header.stamp.nanosec;

            double dist = std::sqrt(C[0]*C[0] + C[1]*C[1] + C[2]*C[2]);
            if (dist < best_dist) {
                best_dist = dist;
                best_pose = pose;
                best_info_str = oss.str();
            }

            cv::polylines(rgb, {cnt}, true, cv::Scalar(0, 255, 0), 2);
        }

        if (best_pose && best_info_str) {
            pose_pub_->publish(*best_pose);

            auto info_msg = std_msgs::msg::String();
            info_msg.data = *best_info_str;
            info_pub_->publish(info_msg);

            RCLCPP_INFO_STREAM(get_logger(), "\n" << *best_info_str);

            {
                std::lock_guard<std::mutex> lk(mutex_);
                frozen_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*best_pose);
                has_detected_ = true;
            }

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = best_pose->header;
            tf_msg.child_frame_id = "detected_box";
            tf_msg.transform.translation.x = best_pose->pose.position.x;
            tf_msg.transform.translation.y = best_pose->pose.position.y;
            tf_msg.transform.translation.z = best_pose->pose.position.z;
            tf_msg.transform.rotation = best_pose->pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);

            RCLCPP_INFO(get_logger(), "✅ Box pose detected and frozen. Tune orientation with roll/pitch/yaw_offset_deg parameters.");
        }

        cv::imshow("Detection", rgb);
        if (!depth32.empty()) {
            cv::Mat depth_vis;
            depth32.convertTo(depth_vis, CV_8UC1, 255.0 / 5.0);
            cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
            cv::imshow("Depth View", depth_vis);
        }
        cv::waitKey(1);
    }

    double fx_, fy_, cx_, cy_;
    double min_area_px_;
    double square_tolerance_;
    double camera_normal_cos_thresh_;
    double plane_inlier_thresh_;
    int ransac_iters_;
    int max_sample_points_;
    std::string camera_frame_;
    std::string base_frame_;

    double roll_offset_deg_;
    double pitch_offset_deg_;
    double yaw_offset_deg_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_, depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;

    std::atomic<bool> detection_active_;
    std::atomic<bool> has_detected_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> frozen_pose_;
    uint64_t cycle_id_;
    std::mutex mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxDetector>());
    rclcpp::shutdown();
    return 0;
}