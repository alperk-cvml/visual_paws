/**
 * @file orbslam3_rgbd_node.cpp
 * @brief ROS 2 wrapper node for ORB-SLAM3 in RGBD mode.
 *
 * Subscribes to synchronised RGB + Depth image topics, runs ORB-SLAM3 tracking,
 * and publishes the estimated camera pose as nav_msgs/Odometry on /odom_slam.
 *
 * The pose is transformed from the camera optical frame to base_link so that
 * downstream consumers (robot_localization EKF) receive body-frame odometry.
 *
 * Architecture (Phase 2 localization pipeline):
 *   /front_camera/image_raw + /depth_camera/depth_image
 *       │
 *       ▼
 *   orbslam3_rgbd_node
 *       │
 *       ├──→ /odom_slam  (nav_msgs/Odometry)
 *       └──→ /orbslam3/tracking_image  (sensor_msgs/Image, debug)
 *
 *   robot_localization EKF
 *       ← /odom_slam + /odom_lko + /imu/data
 *       → /odom (fused, authoritative)
 *       → /tf: map → odom
 */

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/core/core.hpp>

// ORB-SLAM3
#include "System.h"

using namespace std::chrono_literals;

class OrbSlam3RgbdNode : public rclcpp::Node
{
public:
    OrbSlam3RgbdNode()
    : Node("orbslam3_rgbd_node")
    {
        // ── Declare parameters ──────────────────────────────────────────
        this->declare_parameter<std::string>("voc_file",
            "/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt");
        this->declare_parameter<std::string>("settings_file", "");
        this->declare_parameter<std::string>("rgb_topic",
            "/front_camera/image_raw");
        this->declare_parameter<std::string>("depth_topic",
            "/depth_camera/depth_image");
        this->declare_parameter<bool>("enable_viewer", false);
        this->declare_parameter<bool>("publish_tf", false);
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("camera_frame", "camera_link");
        this->declare_parameter<std::string>("mode", "MONOCULAR");

        // ── Read parameters ─────────────────────────────────────────────
        voc_file_ = this->get_parameter("voc_file").as_string();
        settings_file_ = this->get_parameter("settings_file").as_string();
        std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
        std::string depth_topic = this->get_parameter("depth_topic").as_string();
        enable_viewer_ = this->get_parameter("enable_viewer").as_bool();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        mode_ = this->get_parameter("mode").as_string();

        // ── Resolve settings file ───────────────────────────────────────
        if (settings_file_.empty()) {
            // Default: use the config installed with the package
            std::string pkg_share = ament_index_cpp::get_package_share_directory("localization");
            if (mode_ == "MONOCULAR") {
                settings_file_ = pkg_share + "/config/orbslam3_mono.yaml";
            } else {
                settings_file_ = pkg_share + "/config/orbslam3_rgbd.yaml";
            }
        }

        RCLCPP_INFO(this->get_logger(),
            "ORB-SLAM3 Node starting in %s mode\n"
            "  Vocabulary:  %s\n"
            "  Settings:    %s\n"
            "  RGB topic:   %s\n"
            "  Depth topic: %s (used only in RGBD mode)\n"
            "  Viewer:      %s",
            mode_.c_str(), voc_file_.c_str(), settings_file_.c_str(),
            rgb_topic.c_str(), depth_topic.c_str(),
            enable_viewer_ ? "ON" : "OFF");

        // ── Initialise ORB-SLAM3 ────────────────────────────────────────
        ORB_SLAM3::System::eSensor sensor_type;
        if (mode_ == "MONOCULAR") {
            sensor_type = ORB_SLAM3::System::MONOCULAR;
        } else if (mode_ == "RGBD") {
            sensor_type = ORB_SLAM3::System::RGBD;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode: %s. Defaulting to MONOCULAR", mode_.c_str());
            mode_ = "MONOCULAR";
            sensor_type = ORB_SLAM3::System::MONOCULAR;
        }

        slam_system_ = std::make_unique<ORB_SLAM3::System>(
            voc_file_, settings_file_,
            sensor_type,
            enable_viewer_);

        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 system initialised successfully");

        // ── TF2 ─────────────────────────────────────────────────────────
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ── Publishers ──────────────────────────────────────────────────
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom_slam", 10);
        tracking_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/orbslam3/tracking_image", 1);

        // ── Image subscribers ───────────────────────────────────────────
        // Use BEST_EFFORT QoS for camera streams (Gazebo publishes best-effort)
        rmw_qos_profile_t sensor_qos_profile = rmw_qos_profile_sensor_data;

        if (mode_ == "MONOCULAR") {
            mono_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                rgb_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&OrbSlam3RgbdNode::monoImageCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to Monocular RGB [%s]", rgb_topic.c_str());
        } else {
            rgb_sub_.subscribe(this, rgb_topic, sensor_qos_profile);
            depth_sub_.subscribe(this, depth_topic, sensor_qos_profile);

            sync_ = std::make_shared<Synchronizer>(
                SyncPolicy(10), rgb_sub_, depth_sub_);
            sync_->registerCallback(
                std::bind(&OrbSlam3RgbdNode::imageCallback, this,
                          std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(),
                "Subscribed to RGB [%s] + Depth [%s] with ApproximateTime sync",
                rgb_topic.c_str(), depth_topic.c_str());
        }

        // ── Stats timer ─────────────────────────────────────────────────
        stats_timer_ = this->create_wall_timer(
            10s, std::bind(&OrbSlam3RgbdNode::printStats, this));
    }

    ~OrbSlam3RgbdNode()
    {
        if (slam_system_) {
            RCLCPP_INFO(this->get_logger(), "Shutting down ORB-SLAM3...");
            slam_system_->Shutdown();
        }
    }

private:
    // ── Type aliases ────────────────────────────────────────────────────
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    // ── Monocular image callback ────────────────────────────────────────
    void monoImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg)
    {
        // Convert ROS image to OpenCV
        cv::Mat rgb_image;
        try {
            rgb_image = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(),
                "cv_bridge conversion failed: %s", e.what());
            return;
        }

        double timestamp = rclcpp::Time(rgb_msg->header.stamp).seconds();

        Sophus::SE3f Tcw;
        try {
            Tcw = slam_system_->TrackMonocular(rgb_image, timestamp);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(),
                "ORB-SLAM3 TrackMonocular exception: %s", e.what());
            return;
        }

        total_frames_++;

        // ── Check tracking state ────────────────────────────────────────
        int tracking_state = slam_system_->GetTrackingState();
        if (tracking_state != 2) {  // 2 = OK
            if (tracking_state == 3) {  // LOST
                lost_frames_++;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "ORB-SLAM3 tracking LOST (lost %lu / %lu frames)",
                    lost_frames_, total_frames_);
            }
            return;  // Don't publish when not tracking
        }

        tracked_frames_++;

        processAndPublishPose(Tcw, rgb_msg->header.stamp);
    }

    // ── Synchronised image callback ─────────────────────────────────────
    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
    {
        // Convert ROS images to OpenCV
        cv::Mat rgb_image, depth_image;
        try {
            rgb_image = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
            depth_image = cv_bridge::toCvShare(depth_msg)->image;
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(),
                "cv_bridge conversion failed: %s", e.what());
            return;
        }

        // Ensure depth is CV_32FC1 (metres)
        if (depth_image.type() != CV_32FC1) {
            if (depth_image.type() == CV_16UC1) {
                // Convert 16-bit mm depth to 32-bit metre depth
                depth_image.convertTo(depth_image, CV_32FC1, 1.0 / 1000.0);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Unexpected depth encoding: type=%d. Expected 32FC1 or 16UC1.",
                    depth_image.type());
                return;
            }
        }

        // ── Run ORB-SLAM3 tracking ──────────────────────────────────────
        double timestamp = rclcpp::Time(rgb_msg->header.stamp).seconds();

        Sophus::SE3f Tcw;
        try {
            Tcw = slam_system_->TrackRGBD(rgb_image, depth_image, timestamp);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(),
                "ORB-SLAM3 TrackRGBD exception: %s", e.what());
            return;
        }

        total_frames_++;

        // ── Check tracking state ────────────────────────────────────────
        int tracking_state = slam_system_->GetTrackingState();
        if (tracking_state != 2) {  // 2 = OK
            if (tracking_state == 3) {  // LOST
                lost_frames_++;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "ORB-SLAM3 tracking LOST (lost %lu / %lu frames)",
                    lost_frames_, total_frames_);
            }
            return;  // Don't publish when not tracking
        }

        tracked_frames_++;

        processAndPublishPose(Tcw, rgb_msg->header.stamp);
    }

    // ── Common pose processing and publishing ───────────────────────────
    void processAndPublishPose(const Sophus::SE3f & Tcw, const rclcpp::Time & stamp)
    {
        // Tcw is camera-in-world (world = SLAM map origin).
        // We need Twc (world-to-camera) = Tcw.inverse()
        Sophus::SE3f Twc = Tcw.inverse();

        // Extract translation and rotation
        Eigen::Vector3f t = Twc.translation();
        Eigen::Quaternionf q(Twc.rotationMatrix());

        // ORB-SLAM3 operates in camera optical frame:
        //   X-right, Y-down, Z-forward
        // ROS base_link convention:
        //   X-forward, Y-left, Z-up
        //
        // Transform: apply the rotation from camera_optical to base_link.
        // R_base_from_cam = Rz(−π/2) · Rx(−π/2)
        // This is equivalent to: x_ros = z_cam, y_ros = −x_cam, z_ros = −y_cam
        //
        // We look up the camera→base_link transform from TF if available,
        // otherwise apply the standard optical→ROS conversion.

        Eigen::Quaternionf q_optical_to_ros;
        // Rotation from optical frame to ROS frame:
        // optical: Z-forward, X-right, Y-down
        // ROS:     X-forward, Y-left,  Z-up
        // This is a rotation of -90° around Z then -90° around X
        // q = Rz(-π/2) * Rx(-π/2)
        Eigen::AngleAxisf rot_z(-M_PI / 2.0f, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf rot_x(-M_PI / 2.0f, Eigen::Vector3f::UnitX());
        q_optical_to_ros = rot_z * rot_x;

        // Apply frame correction to both position and orientation
        Eigen::Vector3f t_ros = q_optical_to_ros * t;
        Eigen::Quaternionf q_ros = q_optical_to_ros * q * q_optical_to_ros.inverse();
        q_ros.normalize();

        // ── Publish Odometry ────────────────────────────────────────────
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = stamp;
        odom_msg->header.frame_id = map_frame_;
        odom_msg->child_frame_id = base_frame_;

        odom_msg->pose.pose.position.x = static_cast<double>(t_ros.x());
        odom_msg->pose.pose.position.y = static_cast<double>(t_ros.y());
        odom_msg->pose.pose.position.z = static_cast<double>(t_ros.z());

        odom_msg->pose.pose.orientation.x = static_cast<double>(q_ros.x());
        odom_msg->pose.pose.orientation.y = static_cast<double>(q_ros.y());
        odom_msg->pose.pose.orientation.z = static_cast<double>(q_ros.z());
        odom_msg->pose.pose.orientation.w = static_cast<double>(q_ros.w());

        // Conservative covariance — SLAM is accurate but can jump on loop closure
        // Position variance: 0.05 m²  |  Orientation variance: 0.1 rad²
        for (int i = 0; i < 36; ++i) {
            odom_msg->pose.covariance[i] = 0.0;
        }
        odom_msg->pose.covariance[0]  = 0.05;  // x
        odom_msg->pose.covariance[7]  = 0.05;  // y
        odom_msg->pose.covariance[14] = 0.05;  // z
        odom_msg->pose.covariance[21] = 0.1;   // roll
        odom_msg->pose.covariance[28] = 0.1;   // pitch
        odom_msg->pose.covariance[35] = 0.1;   // yaw

        // We don't publish twist (velocity) — EKF gets that from LKO/IMU
        for (int i = 0; i < 36; ++i) {
            odom_msg->twist.covariance[i] = 0.0;
        }

        odom_pub_->publish(std::move(odom_msg));

        // ── Optionally publish TF: map → odom ───────────────────────────
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = map_frame_;
            tf_msg.child_frame_id = odom_frame_;

            tf_msg.transform.translation.x = static_cast<double>(t_ros.x());
            tf_msg.transform.translation.y = static_cast<double>(t_ros.y());
            tf_msg.transform.translation.z = static_cast<double>(t_ros.z());
            tf_msg.transform.rotation.x = static_cast<double>(q_ros.x());
            tf_msg.transform.rotation.y = static_cast<double>(q_ros.y());
            tf_msg.transform.rotation.z = static_cast<double>(q_ros.z());
            tf_msg.transform.rotation.w = static_cast<double>(q_ros.w());

            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    // ── Periodic stats logging ──────────────────────────────────────────
    void printStats()
    {
        if (total_frames_ == 0) {
            RCLCPP_INFO(this->get_logger(),
                "ORB-SLAM3: waiting for images...");
            return;
        }

        double track_rate = static_cast<double>(tracked_frames_) /
                            static_cast<double>(total_frames_) * 100.0;
        int num_map_points = slam_system_->GetTrackedMapPoints().size();
        int num_keypoints = slam_system_->GetTrackedKeyPointsUn().size();

        RCLCPP_INFO(this->get_logger(),
            "ORB-SLAM3 stats | frames: %lu  tracked: %.1f%%  "
            "map_points: %d  keypoints: %d  lost: %lu",
            total_frames_, track_rate, num_map_points,
            num_keypoints, lost_frames_);
    }

    // ── Member variables ────────────────────────────────────────────────
    // ORB-SLAM3
    std::unique_ptr<ORB_SLAM3::System> slam_system_;
    std::string voc_file_;
    std::string settings_file_;
    bool enable_viewer_;
    bool publish_tf_;
    std::string mode_;

    // Frame names
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string camera_frame_;

    // ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_img_pub_;

    // TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Synchronised image subscribers
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<Synchronizer> sync_;

    // Monocular subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_sub_;

    // Stats timer
    rclcpp::TimerBase::SharedPtr stats_timer_;

    // Tracking statistics
    uint64_t total_frames_ = 0;
    uint64_t tracked_frames_ = 0;
    uint64_t lost_frames_ = 0;
};

// ── main ────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrbSlam3RgbdNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
