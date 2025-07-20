#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

class LifecycleArucoTracker : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit LifecycleArucoTracker(const rclcpp::NodeOptions & options);

private:
    // Lifecycle transition callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &);

    void loadParameters();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_pub;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr _precision_land_pub;

    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> _detector_params;
    cv::Mat _camera_matrix;
    cv::Mat _dist_coeffs;

    int _param_dictionary {};
    double _param_marker_size {};
};
