#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/bool.hpp>

#include <vector>
#include <chrono>
#include <limits>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

class LifecyclePrecisionLand : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit LifecyclePrecisionLand(const rclcpp::NodeOptions & options);

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

    rclcpp::QoS _qos_profile;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr _vehicle_attitude_sub;

    px4_msgs::msg::VehicleLocalPosition _current_local_position;
    px4_msgs::msg::VehicleAttitude      _current_attitude;

    rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_ctrl_pub;
    rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_cmd_pub;
    rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr _precision_land_pub;    

    rclcpp::TimerBase::SharedPtr _ctrl_timer;
    void loadParameters();

    struct ArucoTag {
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

    enum class State {
        Idle,
        Search,
        Approach,
        Descend,
        Finished
    };

    void switchToState(State state);
    std::string stateName(State state);

    State _state = State::Search;
    bool _search_started = false;
    bool _arm_started = false;
    bool _offboard_engaged = false;
    rclcpp::Time _arm_time;
    float _approach_altitude = {};

    bool _land_detected = false;
    bool _target_lost_prev = true;

    std::vector<Eigen::Vector3f> _search_waypoints;
    void generateSearchWaypoints();
    int _search_waypoint_index = 0;

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
