#pragma once

// include px4_msgs headers
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

// include px4-ros2-interface-lib headers
// #include <px4_ros2/odometry/local_position.hpp>
// #include <px4_ros2/odometry/attitude.hpp>
// #include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

// include ROS2, geometry_msgs, tf2 headers
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// include others
#include <vector>
#include <chrono>
#include <limits>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

class PrecisionLand : public rclcpp::Node
{
public:
	explicit PrecisionLand();

	void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
	void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
	void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);

	void publishVehicleCommand(
		uint16_t command,
		float param1 = 0.0f,
		float param2 = 0.0f,
		float param3 = 0.0f,
		float param4 = 0.0f,
		float param5 = 0.0f,
		float param6 = 0.0f,
		float param7 = 0.0f
	);

	void arm();
	void disarm();
	void controlLoop();
	void engageOffboardControlMode();
	void publishOffBoardControlMode();
	void publishTrajectorySetpointPos(float x, float y, float z, float yaw_deg = 0.0f);
	void publishTrajectorySetpointVel(float vx, float vy, float vz, float yaw_deg = 0.0f);

	static float quaternionToYawRad(const Eigen::Quaterniond& q);

private:
/*--------------- For px4-ros2-interface-library ----------------*/
	// px4_ros2 is not used in this code for test
	// but it can be uncommented if needed
	// px4_ros2::Context _context;
	// std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	// std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	// std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

/*--------------- For ROS2 Initialization ----------------*/
	rclcpp::QoS _qos_profile;
	// ros2 subscriptions
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr _vehicle_attitude_sub;

	px4_msgs::msg::VehicleLocalPosition _current_local_position;
	px4_msgs::msg::VehicleAttitude      _current_attitude;

    // ros2 publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_ctrl_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_cmd_pub;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;

    // ros2 timer
    rclcpp::TimerBase::SharedPtr _ctrl_timer;
	void loadParameters();


/*--------------- For Aruco Marker (AprilTag) Detection ----------------*/
	// For Aruco Marker or Aruco Tag Detecion
	struct ArucoTag {
		// Initialize position with NaN values directly in the struct
		Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
		Eigen::Quaterniond orientation;
		rclcpp::Time timestamp;

		bool valid() { return timestamp.nanoseconds() > 0; };
	};

	ArucoTag getTagWorld(const ArucoTag& tag);

	Eigen::Vector2f calculateVelocitySetpointXY();
	bool checkTargetTimeout();
	bool positionReached(const Eigen::Vector3f& target) const;

	ArucoTag _tag;

/*--------------- For Precision Landing States ----------------*/
	enum class State {
		Idle,
		Search, 	// Searches for target using a search pattern
		Approach, 	// Positioning over landing target while maintaining altitude
		Descend, 	// Stay over landing target while descending
		Finished
	};

	void switchToState(State state);
	std::string stateName(State state);

	// Data
	State _state = State::Search;
	bool _search_started = false;
	bool _arm_started = false;
	bool _offboard_engaged = false;
	rclcpp::Time _arm_time;
	float _approach_altitude = {};

	// Land detection
	bool _land_detected = false;
	bool _target_lost_prev = true;

	// Waypoints for Search pattern
	std::vector<Eigen::Vector3f> _search_waypoints;
	// Search pattern generation
	void generateSearchWaypoints();
	// Search pattern index
	int _search_waypoint_index = 0;

	// Parameters
	float _param_descent_vel = {};
	float _param_vel_p_gain = {};
	float _param_vel_i_gain = {};
	float _param_max_velocity = {};
	float _param_target_timeout = {};
	float _param_delta_position = {};
	float _param_delta_velocity = {};

	float _vel_x_integral {};
	float _vel_y_integral {};
};
