#include "LifecyclePrecisionLandOffboard.hpp"
#include <px4_ros2/utils/geometry.hpp>
#include <algorithm>
#include <numbers>

LifecyclePrecisionLand::LifecyclePrecisionLand(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("precision_land", options),
      _qos_profile(rclcpp::KeepLast(1))
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecyclePrecisionLand::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring Precision Land");

    _qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    _offboard_ctrl_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", _qos_profile);
    _vehicle_cmd_pub   = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", _qos_profile);
    _trajectory_setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", _qos_profile);
    _precision_land_pub = create_publisher<std_msgs::msg::Bool>(
        "precision_land_offboard_lifecycle/end", rclcpp::QoS(1).transient_local());

    _target_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", rclcpp::QoS(1).best_effort(),
        std::bind(&LifecyclePrecisionLand::targetPoseCallback, this, std::placeholders::_1));

    _vehicle_land_detected_sub = create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", _qos_profile,
        std::bind(&LifecyclePrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));

    _vehicle_local_position_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", _qos_profile,
        std::bind(&LifecyclePrecisionLand::vehicleLocalPositionCallback, this, std::placeholders::_1));
    
    _vehicle_attitude_sub = create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", _qos_profile,
        std::bind(&LifecyclePrecisionLand::vehicleAttitudeCallback, this, std::placeholders::_1));

    loadParameters();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecyclePrecisionLand::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Activating Precision Land");
    LifecycleNode::on_activate(state);
    _offboard_ctrl_pub->on_activate();
    _vehicle_cmd_pub->on_activate();
    _precision_land_pub->on_activate();
    _trajectory_setpoint_pub->on_activate();
    _ctrl_timer = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&LifecyclePrecisionLand::controlLoop, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecyclePrecisionLand::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Deactivating Precision Land");
    LifecycleNode::on_activate(state);
    _offboard_ctrl_pub->on_deactivate();
    _vehicle_cmd_pub->on_deactivate();
    _precision_land_pub->on_deactivate();
    _trajectory_setpoint_pub->on_deactivate();
    _ctrl_timer->cancel();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecyclePrecisionLand::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up Precision Land");
    _target_pose_sub.reset();
    _vehicle_land_detected_sub.reset();
    _vehicle_local_position_sub.reset();
    _vehicle_attitude_sub.reset();
    _offboard_ctrl_pub.reset();
    _vehicle_cmd_pub.reset();
    _precision_land_pub.reset();
    _trajectory_setpoint_pub.reset();
    _ctrl_timer.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecyclePrecisionLand::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down Precision Land");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LifecyclePrecisionLand::loadParameters()
{
    declare_parameter<float>("descent_vel", 1.0);
    declare_parameter<float>("vel_p_gain", 1.5);
    declare_parameter<float>("vel_i_gain", 0.0);
    declare_parameter<float>("max_velocity", 3.0);
    declare_parameter<float>("target_timeout", 3.0);
    declare_parameter<float>("delta_position", 0.25);
    declare_parameter<float>("delta_velocity", 0.25);

    get_parameter("descent_vel", _param_descent_vel);
    get_parameter("vel_p_gain", _param_vel_p_gain);
    get_parameter("vel_i_gain", _param_vel_i_gain);
    get_parameter("max_velocity", _param_max_velocity);
    get_parameter("target_timeout", _param_target_timeout);
    get_parameter("delta_position", _param_delta_position);
    get_parameter("delta_velocity", _param_delta_velocity);

    RCLCPP_INFO(get_logger(), "descent_vel: %f", _param_descent_vel);
    RCLCPP_INFO(get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
}

void LifecyclePrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
    _land_detected = msg->landed;
}

void LifecyclePrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (_search_started) {
        auto tag = ArucoTag {
            .position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
            .orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
            .timestamp = now(),
        };

        _tag = getTagWorld(tag);
    }
}

void LifecyclePrecisionLand::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    _current_local_position = *msg;
    if (!_search_started){
        generateSearchWaypoints();
        _search_started = true;
        RCLCPP_INFO(get_logger(), "Search waypoints generated at z=%.2f",
                    _current_local_position.z);
    }
}

void LifecyclePrecisionLand::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    _current_attitude = *msg;
}

void LifecyclePrecisionLand::publishVehicleCommand(
    uint16_t command,
    float param1,
    float param2,
    float param3,
    float param4,
    float param5,
    float param6,
    float param7)
{
  px4_msgs::msg::VehicleCommand msg;
  msg.command          = command;
  msg.param1           = param1;
  msg.param2           = param2;
  msg.param3           = param3;
  msg.param4           = param4;
  msg.param5           = param5;
  msg.param6           = param6;
  msg.param7           = param7;

  msg.target_system    = 1;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;
  msg.from_external    = true;

  msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);

  _vehicle_cmd_pub->publish(msg);
}

void LifecyclePrecisionLand::arm()
{
    RCLCPP_INFO(get_logger(), "Arming vehicle");
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void LifecyclePrecisionLand::disarm()
{
    RCLCPP_INFO(get_logger(), "Disarming vehicle");
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void LifecyclePrecisionLand::engageOffboardControlMode(){
    RCLCPP_INFO(get_logger(), "Engage Offboard Control Mode");
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void LifecyclePrecisionLand::publishOffBoardControlMode()
{
  px4_msgs::msg::OffboardControlMode msg;
  msg.position     = true;
  msg.velocity     = true;
  msg.acceleration = false;
  msg.attitude     = false;
  msg.body_rate    = false;
  msg.timestamp    = static_cast<uint64_t>(this->now().nanoseconds() / 1000);

  _offboard_ctrl_pub->publish(msg);
}

void LifecyclePrecisionLand::publishTrajectorySetpointPos(
    float x, float y, float z,
    float yaw_deg)
{
  px4_msgs::msg::TrajectorySetpoint msg;

  msg.position[0] = x;
  msg.position[1] = y;
  msg.position[2] = z;

  float nanv = std::numeric_limits<float>::quiet_NaN();
  msg.velocity[0]     = nanv;
  msg.velocity[1]     = nanv;
  msg.velocity[2]     = nanv;
  msg.acceleration[0] = nanv;
  msg.acceleration[1] = nanv;
  msg.acceleration[2] = nanv;

  constexpr float PI = 3.14159265358979323846f;
  float yaw_rad = yaw_deg * (PI / 180.0f);
  msg.yaw = std::clamp(yaw_rad, -PI, PI);

  msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);

  _trajectory_setpoint_pub->publish(msg);
}

void LifecyclePrecisionLand::publishTrajectorySetpointVel(
    float vx, float vy, float vz,
    float yaw_deg)
{
  px4_msgs::msg::TrajectorySetpoint msg;
  float nanv = std::numeric_limits<float>::quiet_NaN();
  msg.position[0] = nanv;
  msg.position[1] = nanv;
  msg.position[2] = nanv;

  msg.velocity[0]     = vx;
  msg.velocity[1]     = vy;
  msg.velocity[2]     = vz;

  msg.acceleration[0] = nanv;
  msg.acceleration[1] = nanv;
  msg.acceleration[2] = nanv;

  constexpr float PI = 3.14159265358979323846f;
  float yaw_rad = yaw_deg * (PI / 180.0f);
  msg.yaw = std::clamp(yaw_rad, -PI, PI);

  msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);

  _trajectory_setpoint_pub->publish(msg);
}

LifecyclePrecisionLand::ArucoTag LifecyclePrecisionLand::getTagWorld(const ArucoTag& tag)
{
    Eigen::Matrix3d R;
    R << 0, -1, 0,
         1, 0, 0,
         0, 0, 1;
    Eigen::Quaterniond quat_NED(R);

    Eigen::Vector3d vehicle_position(
        static_cast<double>(_current_local_position.x),
        static_cast<double>(_current_local_position.y),
        static_cast<double>(_current_local_position.z)
    );

    Eigen::Quaterniond vehicle_orientation(
        static_cast<double>(_current_attitude.q[0]),
        static_cast<double>(_current_attitude.q[1]),
        static_cast<double>(_current_attitude.q[2]),
        static_cast<double>(_current_attitude.q[3])
    );

    Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
    Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * quat_NED;
    Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
    Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

    ArucoTag world_tag = {
        .position = tag_world_transform.translation(),
        .orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
        .timestamp = tag.timestamp,
    };

    return world_tag;
}

Eigen::Vector2f LifecyclePrecisionLand::calculateVelocitySetpointXY()
{
    float p_gain = _param_vel_p_gain;
    float i_gain = _param_vel_i_gain;

    float delta_pos_x = _current_local_position.x - _tag.position.x();
    float delta_pos_y = _current_local_position.y - _tag.position.y();

    _vel_x_integral += delta_pos_x;
    _vel_y_integral += delta_pos_y;
    float max_integral = _param_max_velocity;
    _vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
    _vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

    float Xp = delta_pos_x * p_gain;
    float Xi = _vel_x_integral * i_gain;
    float Yp = delta_pos_y * p_gain;
    float Yi = _vel_y_integral * i_gain;

    float vx = -1.f * (Xp + Xi);
    float vy = -1.f * (Yp + Yi);

    vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
    vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

    return Eigen::Vector2f(vx, vy);
}

void LifecyclePrecisionLand::generateSearchWaypoints()
{
    double start_x = 0.0;
    double start_y = 0.0;
    double current_z = _current_local_position.z;
    auto min_z = -1.0;

    double max_radius = 2.0;
    double layer_spacing = 0.5;
    int points_per_layer = 16;
    std::vector<Eigen::Vector3f> waypoints;

    int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ? 1 : (static_cast<int>((
                 min_z - current_z) / layer_spacing) / 2);

    for (int layer = 0; layer < num_layers; ++layer) {
        std::vector<Eigen::Vector3f> layer_waypoints;

        double radius = 0.0;

        for (int point = 0; point < points_per_layer + 1; ++point) {
            double angle = 2.0 * M_PI * point / points_per_layer;
            double x = start_x + radius * cos(angle);
            double y = start_y + radius * sin(angle);
            double z = current_z;

            layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
            radius += max_radius / points_per_layer;
        }

        waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

        current_z += layer_spacing;

        std::reverse(layer_waypoints.begin(), layer_waypoints.end());

        for (auto& waypoint : layer_waypoints) {
            waypoint.z() = current_z;
        }

        waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

        current_z += layer_spacing;
    }

    _search_waypoints = waypoints;
}

bool LifecyclePrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
    Eigen::Vector3f position{
        _current_local_position.x,
        _current_local_position.y,
        _current_local_position.z
    };
    Eigen::Vector3f velocity{
        _current_local_position.vx,
        _current_local_position.vy,
        _current_local_position.vz
    };

    const Eigen::Vector3f delta_pos = target - position;

    return (delta_pos.norm() < _param_delta_position)
        && (velocity.norm()  < _param_delta_velocity);
}

std::string LifecyclePrecisionLand::stateName(State state)
{
    switch (state) {
    case State::Idle:
        return "Idle";
    case State::Search:
        return "Search";
    case State::Approach:
        return "Approach";
    case State::Descend:
        return "Descend";
    case State::Finished:
        return "Finished";
    default:
        return "Unknown";
    }
}

void LifecyclePrecisionLand::switchToState(State state)
{
    RCLCPP_INFO(get_logger(), "Switching to %s", stateName(state).c_str());
    _state = state;
}

bool LifecyclePrecisionLand::checkTargetTimeout()
{
    if (!_tag.valid()) {
        return true;
    }

    if (now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
        return true;
    }

    return false;
}

float LifecyclePrecisionLand::quaternionToYawRad(const Eigen::Quaterniond &q)
{
  tf2::Quaternion tf_q(
    q.x(),
    q.y(),
    q.z(),
    q.w()
  );

  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  return static_cast<float>(yaw);
}

void LifecyclePrecisionLand::controlLoop(){
    if (!_arm_started) {
        RCLCPP_INFO(get_logger(), "Arming vehicle");
        arm();
        _arm_started = true;
        _arm_time = this->now();
    }

    constexpr double OFFBOARD_DELAY_S = 1.0;
    if (_arm_started && !_offboard_engaged) {
        auto dt = this->now() - _arm_time;
        if (dt.seconds() > OFFBOARD_DELAY_S) {
            RCLCPP_INFO(get_logger(), "Engaging Offboard Control Mode");
            engageOffboardControlMode();
            _offboard_engaged = true;
        }
    }

    if (_offboard_engaged) {
        publishOffBoardControlMode();
    }

    bool target_lost = checkTargetTimeout();

    if (target_lost && !_target_lost_prev) {
        RCLCPP_INFO(get_logger(), "Target lost: State %s", stateName(_state).c_str());
    } else if (!target_lost && _target_lost_prev) {
        RCLCPP_INFO(get_logger(), "Target acquired");
    }

    _target_lost_prev = target_lost;

    switch (_state) {
    case State::Idle: {
        break;
    }

    case State::Search: {
        if (!std::isnan(_tag.position.x())) {
            _approach_altitude = _current_local_position.z;
            switchToState(State::Approach);
            break;
        }

        auto waypoint_position = _search_waypoints[_search_waypoint_index];

        publishTrajectorySetpointPos(
            waypoint_position.x(),
            waypoint_position.y(),
            waypoint_position.z(),
            0.0f
        );

        if (positionReached(waypoint_position)) {
            _search_waypoint_index++;

            if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
                _search_waypoint_index = 0;
            }
        }

        break;
    }

    case State::Approach: {
        if (target_lost) {
            RCLCPP_INFO(get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
            switchToState(State::Idle);
            return;
        }

        auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(), _approach_altitude);

        publishTrajectorySetpointPos(
            target_position.x(),
            target_position.y(),
            target_position.z(),
            0.0f
        );

        if (positionReached(target_position)) {
            switchToState(State::Descend);
        }

        break;
    }

    case State::Descend: {
        if (target_lost) {
            if (_current_local_position.z >= (-0.8)){
                std_msgs::msg::Bool msg;
                msg.data = true;
                _precision_land_pub->publish(msg);
                switchToState(State::Finished);
                return;
            } else {
                RCLCPP_INFO(get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
                switchToState(State::Idle);
                return;
            }
        }

        Eigen::Vector2f vel = calculateVelocitySetpointXY();
        float yaw_rad = quaternionToYawRad(_tag.orientation);
        publishTrajectorySetpointVel(
            vel.x(),
            vel.y(),
            _param_descent_vel,
            yaw_rad
        );

        if (_land_detected) {
            switchToState(State::Finished);
        }

        break;
    }

    case State::Finished: {
        disarm();
        RCLCPP_INFO(get_logger(), "Landing finished, disarming vehicle");
        _ctrl_timer->cancel();
        break;
    }
    }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto lifecycle_precision_land_node = std::make_shared<LifecyclePrecisionLand>(rclcpp::NodeOptions());
  executor.add_node(lifecycle_precision_land_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
