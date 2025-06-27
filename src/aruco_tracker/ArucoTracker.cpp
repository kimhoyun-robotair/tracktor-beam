#include "ArucoTracker.hpp"
#include <sstream>

ArucoTrackerNode::ArucoTrackerNode()
    : Node("aruco_tracker_node")
{
    loadParameters();

    // [MODIFIED] Initialize AprilTag detector (removed ArUcoDetector usage)
    _dictionary      = cv::aruco::getPredefinedDictionary(_param_dictionary);  // AprilTag 36h10
    _detector_params = cv::aruco::DetectorParameters::create();

    auto qos = rclcpp::QoS(1).best_effort();

    _image_sub = create_subscription<sensor_msgs::msg::Image>(
        "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image", qos,
        std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1)
    );

    _camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info", qos,
        std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1)
    );

    // Publishers
    _image_pub       = create_publisher<sensor_msgs::msg::Image>("/image_proc", qos);
    _target_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", qos);
}

void ArucoTrackerNode::loadParameters()
{
    // [MODIFIED] Only dictionary and marker_size parameters
    declare_parameter<int>("dictionary", cv::aruco::DICT_APRILTAG_36h10);
    declare_parameter<double>("marker_size", 0.5);

    get_parameter("dictionary", _param_dictionary);
    get_parameter("marker_size", _param_marker_size);
}

void ArucoTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Detect AprilTag markers
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
                // Undistort detected corners
                std::vector<std::vector<cv::Point2f>> undistortedCorners;
                for (const auto& corner : corners) {
                    std::vector<cv::Point2f> ud;
                    cv::undistortPoints(
                        corner, ud, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix
                    );
                    undistortedCorners.push_back(ud);
                }

                // Process first detected tag only
                size_t i = 0;
                float half_size = _param_marker_size / 2.0f;
                std::vector<cv::Point3f> objectPoints = {
                    {-half_size,  half_size, 0},
                    { half_size,  half_size, 0},
                    { half_size, -half_size, 0},
                    {-half_size, -half_size, 0}
                };

                // Estimate pose
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

                // Convert rotation to quaternion
                cv::Mat rot_mat;
                cv::Rodrigues(rvec, rot_mat);
                cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();

                // Publish pose
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp    = msg->header.stamp;
                pose_msg.header.frame_id = msg->header.frame_id;  // preserve input frame
                pose_msg.pose.position.x = tvec[0];
                pose_msg.pose.position.y = tvec[1];
                pose_msg.pose.position.z = tvec[2];
                pose_msg.pose.orientation.x = quat.x;
                pose_msg.pose.orientation.y = quat.y;
                pose_msg.pose.orientation.z = quat.z;
                pose_msg.pose.orientation.w = quat.w;

                _target_pose_pub->publish(pose_msg);

                // Annotate image with pose text
                annotate_image(cv_ptr, tvec);
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Missing camera calibration");
        }

        // Always publish annotated image
        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image    = cv_ptr->image;
        _image_pub->publish(*out_msg.toImageMsg());

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void ArucoTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
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

    // Unsubscribe after first valid info
    if (_camera_matrix.at<double>(0, 0) != 0) {
        _camera_info_sub.reset();
    } else {
        RCLCPP_ERROR(get_logger(), "Focal length is zero after update!");
    }
}

void ArucoTrackerNode::annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target)
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
    rclcpp::spin(std::make_shared<ArucoTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
