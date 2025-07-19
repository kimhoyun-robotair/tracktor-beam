#include "LifecycleArucoTracker.hpp"
#include <sstream>

LifecycleArucoTracker::LifecycleArucoTracker(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("aruco_tracker_node", options)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleArucoTracker::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring Aruco Tracker");

    loadParameters();

    _dictionary      = cv::aruco::getPredefinedDictionary(_param_dictionary);
    _detector_params = cv::aruco::DetectorParameters::create();

    auto qos = rclcpp::QoS(1).best_effort();

    _image_sub = create_subscription<sensor_msgs::msg::Image>(
        "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image", qos,
        std::bind(&LifecycleArucoTracker::image_callback, this, std::placeholders::_1)
    );

    _camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info", qos,
        std::bind(&LifecycleArucoTracker::camera_info_callback, this, std::placeholders::_1)
    );

    _image_pub       = create_publisher<sensor_msgs::msg::Image>("/image_proc", qos);
    _target_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", qos);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleArucoTracker::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Activating Aruco Tracker");
    LifecycleNode::on_activate(state);
    _image_pub->on_activate();
    _target_pose_pub->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleArucoTracker::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Deactivating Aruco Tracker");
    LifecycleNode::on_deactivate(state);
    _image_pub->on_deactivate();
    _target_pose_pub->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleArucoTracker::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up Aruco Tracker");
    _image_sub.reset();
    _camera_info_sub.reset();
    _image_pub.reset();
    _target_pose_pub.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleArucoTracker::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down Aruco Tracker");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LifecycleArucoTracker::loadParameters()
{
    declare_parameter<int>("dictionary", cv::aruco::DICT_APRILTAG_36h10);
    declare_parameter<double>("marker_size", 0.5);

    get_parameter("dictionary", _param_dictionary);
    get_parameter("marker_size", _param_marker_size);
}

void LifecycleArucoTracker::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(
            cv_ptr->image,
            _dictionary,
            corners,
            ids,
            _detector_params
        );
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

        if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {
            if (!ids.empty()) {
                std::vector<std::vector<cv::Point2f>> undistortedCorners;
                for (const auto& corner : corners) {
                    std::vector<cv::Point2f> ud;
                    cv::undistortPoints(
                        corner, ud, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix
                    );
                    undistortedCorners.push_back(ud);
                }

                size_t i = 0;
                float half_size = _param_marker_size / 2.0f;
                std::vector<cv::Point3f> objectPoints = {
                    {-half_size,  half_size, 0},
                    { half_size,  half_size, 0},
                    { half_size, -half_size, 0},
                    {-half_size, -half_size, 0}
                };

                cv::Vec3d rvec, tvec;
                cv::solvePnP(
                    objectPoints,
                    undistortedCorners[i],
                    _camera_matrix,
                    cv::noArray(),
                    rvec,
                    tvec
                );
                cv::drawFrameAxes(
                    cv_ptr->image,
                    _camera_matrix,
                    cv::noArray(),
                    rvec,
                    tvec,
                    _param_marker_size
                );

                cv::Mat rot_mat;
                cv::Rodrigues(rvec, rot_mat);
                cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();

                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp    = msg->header.stamp;
                pose_msg.header.frame_id = msg->header.frame_id;
                pose_msg.pose.position.x = tvec[0];
                pose_msg.pose.position.y = tvec[1];
                pose_msg.pose.position.z = tvec[2];
                pose_msg.pose.orientation.x = quat.x;
                pose_msg.pose.orientation.y = quat.y;
                pose_msg.pose.orientation.z = quat.z;
                pose_msg.pose.orientation.w = quat.w;

                _target_pose_pub->publish(pose_msg);

                annotate_image(cv_ptr, tvec);
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Missing camera calibration");
        }

        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image    = cv_ptr->image;
        _image_pub->publish(*out_msg.toImageMsg());

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void LifecycleArucoTracker::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    _camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    _dist_coeffs   = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();

    RCLCPP_INFO(
        get_logger(),
        "Camera matrix updated: fx=%f, fy=%f, cx=%f, cy=%f",
        _camera_matrix.at<double>(0, 0),
        _camera_matrix.at<double>(1, 1),
        _camera_matrix.at<double>(0, 2),
        _camera_matrix.at<double>(1, 2)
    );

    if (_camera_matrix.at<double>(0, 0) != 0) {
        _camera_info_sub.reset();
    } else {
        RCLCPP_ERROR(get_logger(), "Focal length is zero after update!");
    }
}

void LifecycleArucoTracker::annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2)
       << "X: " << target[0]
       << " Y: " << target[1]
       << " Z: " << target[2];
    std::string text = ss.str();

    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    int thickness = 2;
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    baseline += thickness;
    cv::Point org(image->image.cols - textSize.width - 10,
                   image->image.rows - 10);
    cv::putText(
        image->image, text, org,
        fontFace, fontScale,
        cv::Scalar(0, 255, 255), thickness, cv::LINE_AA
    );
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto lifecycle_aruco_node = std::make_shared<LifecycleArucoTracker>(rclcpp::NodeOptions());
    executor.add_node(lifecycle_aruco_node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
