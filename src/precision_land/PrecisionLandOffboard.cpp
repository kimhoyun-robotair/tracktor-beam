#include "PrecisionLandOffboard.hpp"
#include <px4_ros2/utils/geometry.hpp>
#include <algorithm>
#include <numbers>

/*--------------- For Node Initialization ----------------*/
PrecisionLand::PrecisionLand()
	: Node("precision_land")//, _context(*this)
	, _qos_profile(rclcpp::KeepLast(1))
{
	// px4_ros2 Initialization
	// _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(_context);
	// _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(_context);
	// _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(_context);


	_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    /* 3) 퍼블리셔 */
    _offboard_ctrl_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", _qos_profile);
    _vehicle_cmd_pub   = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", _qos_profile);
	_trajectory_setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
		"/fmu/in/trajectory_setpoint", _qos_profile);

    /* 4) 구독 */
    _target_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", rclcpp::QoS(1).best_effort(),
        std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));

    _vehicle_land_detected_sub = create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", _qos_profile,
        std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));

	_vehicle_local_position_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position", _qos_profile,
		std::bind(&PrecisionLand::vehicleLocalPositionCallback, this, std::placeholders::_1));
	
	_vehicle_attitude_sub = create_subscription<px4_msgs::msg::VehicleAttitude>(
		"/fmu/out/vehicle_attitude", _qos_profile,
		std::bind(&PrecisionLand::vehicleAttitudeCallback, this, std::placeholders::_1));

	loadParameters();

    _ctrl_timer = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PrecisionLand::controlLoop, this));

    RCLCPP_INFO(get_logger(), "PrecisionLand node started (Offboard mode)");
}

/*--------------- For Parameters Initialization ----------------*/
void PrecisionLand::loadParameters()
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

/*--------------- For PX4-Msgs Callback ----------------*/
void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if (_search_started) {
		auto tag = ArucoTag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = now(),
		};

		// Save tag position/orientation in NED world frame
		_tag = getTagWorld(tag);
	}

}

void PrecisionLand::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
	_current_local_position = *msg;
	if (!_search_started){
		generateSearchWaypoints();
		_search_started = true;
        RCLCPP_INFO(get_logger(), "Search waypoints generated at z=%.2f",
                    _current_local_position.z);
	}
}

void PrecisionLand::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
	_current_attitude = *msg;
}

/*--------------- For PX4 VehicleCommand Utils ----------------*/
void PrecisionLand::publishVehicleCommand(
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

void PrecisionLand::arm()
{
	// arm the vehicle
	RCLCPP_INFO(get_logger(), "Arming vehicle");
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void PrecisionLand::disarm()
{
	// Disarm the vehicle
	RCLCPP_INFO(get_logger(), "Disarming vehicle");
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void PrecisionLand::engageOffboardControlMode(){
	RCLCPP_INFO(get_logger(), "Engage Offboard Control Mode");
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void PrecisionLand::publishOffBoardControlMode()
{
  px4_msgs::msg::OffboardControlMode msg;
  msg.position     = true;   // position setpoints 사용
  msg.velocity     = true;
  msg.acceleration = false;
  msg.attitude     = false;
  msg.body_rate    = false;
  // Python 의 get_clock().now().nanoseconds / 1000 에 대응
  msg.timestamp    = static_cast<uint64_t>(this->now().nanoseconds() / 1000);

  _offboard_ctrl_pub->publish(msg);
}

void PrecisionLand::publishTrajectorySetpointPos(
    float x, float y, float z,
    float yaw_deg)
{
  px4_msgs::msg::TrajectorySetpoint msg;

  // 1) position 설정
  msg.position[0] = x;
  msg.position[1] = y;
  msg.position[2] = z;

  // 2) velocity, acceleration를 NaN으로 설정하여 제어 대상에서 제외
  float nanv = std::numeric_limits<float>::quiet_NaN();
  msg.velocity[0]     = nanv;
  msg.velocity[1]     = nanv;
  msg.velocity[2]     = nanv;
  msg.acceleration[0] = nanv;
  msg.acceleration[1] = nanv;
  msg.acceleration[2] = nanv;

  // 3) yaw를 degree→rad 변환 후 -π..+π 범위로 클램프
  constexpr float PI = 3.14159265358979323846f;
  float yaw_rad = yaw_deg * (PI / 180.0f);
  msg.yaw = std::clamp(yaw_rad, -PI, PI);

  // 4) timestamp (마이크로초 단위)
  msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);

  // 5) 발행
  _trajectory_setpoint_pub->publish(msg);
}

void PrecisionLand::publishTrajectorySetpointVel(
    float vx, float vy, float vz,
    float yaw_deg)
{
  px4_msgs::msg::TrajectorySetpoint msg;
  float nanv = std::numeric_limits<float>::quiet_NaN();
  // 1) position 설정
  msg.position[0] = nanv;
  msg.position[1] = nanv;
  msg.position[2] = nanv;

  // 2) velocity, acceleration를 NaN으로 설정하여 제어 대상에서 제외
  msg.velocity[0]     = vx;
  msg.velocity[1]     = vy;
  msg.velocity[2]     = vz;

  msg.acceleration[0] = nanv;
  msg.acceleration[1] = nanv;
  msg.acceleration[2] = nanv;

  // 3) yaw를 degree→rad 변환 후 -π..+π 범위로 클램프
  constexpr float PI = 3.14159265358979323846f;
  float yaw_rad = yaw_deg * (PI / 180.0f);
  msg.yaw = std::clamp(yaw_rad, -PI, PI);

  // 4) timestamp (마이크로초 단위)
  msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);

  // 5) 발행
  _trajectory_setpoint_pub->publish(msg);
}

/*--------------- For AprilTag Utils ----------------*/
PrecisionLand::ArucoTag PrecisionLand::getTagWorld(const ArucoTag& tag)
{
	// Convert from optical to NED
	// Optical: X right, Y down, Z away from lens
	// NED: X forward, Y right, Z away from viewer
	Eigen::Matrix3d R;
	R << 0, -1, 0,
	1, 0, 0,
	0, 0, 1;
	Eigen::Quaterniond quat_NED(R);

	// 차량의 NED 위치를 Eigen::Vector3d 로 변환
	Eigen::Vector3d vehicle_position(
	static_cast<double>(_current_local_position.x),
	static_cast<double>(_current_local_position.y),
	static_cast<double>(_current_local_position.z)
	);

	// 차량의 쿼터니언(종합): q 배열은 (w, x, y, z) 순서입니다
	Eigen::Quaterniond vehicle_orientation(
	static_cast<double>(_current_attitude.q[0]),  // w
	static_cast<double>(_current_attitude.q[1]),  // x
	static_cast<double>(_current_attitude.q[2]),  // y
	static_cast<double>(_current_attitude.q[3])   // z
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

Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY()
{
	float p_gain = _param_vel_p_gain;
	float i_gain = _param_vel_i_gain;

	// P component
	float delta_pos_x = _current_local_position.x - _tag.position.x();
	float delta_pos_y = _current_local_position.y - _tag.position.y();

	// I component
	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;
	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

	float Xp = delta_pos_x * p_gain;
	float Xi = _vel_x_integral * i_gain;
	float Yp = delta_pos_y * p_gain;
	float Yi = _vel_y_integral * i_gain;

	// Sum P and I gains
	float vx = -1.f * (Xp + Xi);
	float vy = -1.f * (Yp + Yi);

	// 0.1m/s min vel and 3m/s max vel
	vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

	return Eigen::Vector2f(vx, vy);
}

/*--------------- For Landing Utils ----------------*/
void PrecisionLand::generateSearchWaypoints()
{
	// Generate spiral search waypoints
	// The search waypoints are generated in the NED frame
	// Parameters for the search pattern
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _current_local_position.z;
	auto min_z = -1.0;

	double max_radius = 2.0;
	double layer_spacing = 0.5;
	int points_per_layer = 16;
	std::vector<Eigen::Vector3f> waypoints;

	// Generate waypoints
	// Calculate the number of layers needed
	int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ? 1 : (static_cast<int>((
				 min_z - current_z) / layer_spacing) / 2);

	// Generate waypoints
	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;

		// Spiral out to max radius
		double radius = 0.0;

		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;

			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer;
		}

		// Push the spiral out waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the inward spiral
		current_z += layer_spacing;

		// Reverse the layer waypoints for spiral in
		std::reverse(layer_waypoints.begin(), layer_waypoints.end());

		// Adjust the z-coordinate for the inward spiral
		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = current_z;
		}

		// Push the reversed waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the next outward spiral
		current_z += layer_spacing;
	}

	_search_waypoints = waypoints;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
    // 메시지 필드를 Eigen::Vector3f 로 변환
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

    // 목표 위치와의 차이 계산
    const Eigen::Vector3f delta_pos = target - position;

    // 거리와 속도 기준 비교
    return (delta_pos.norm() < _param_delta_position)
        && (velocity.norm()  < _param_delta_velocity);
}

std::string PrecisionLand::stateName(State state)
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

void PrecisionLand::switchToState(State state)
{
	RCLCPP_INFO(get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

bool PrecisionLand::checkTargetTimeout()
{
	if (!_tag.valid()) {
		return true;
	}

	if (now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}

	return false;
}

float PrecisionLand::quaternionToYawRad(const Eigen::Quaterniond &q)
{
  // Eigen → TF2 쿼터니언 복사
  tf2::Quaternion tf_q(
    q.x(),  // x
    q.y(),  // y
    q.z(),  // z
    q.w()   // w
  );

  // RPY 추출 (roll, pitch, yaw)
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  return static_cast<float>(yaw);
}

/*--------------- For Control Timer Callback ----------------*/
void PrecisionLand::controlLoop(){
    // Offboard Mode Activation
    // 1) Arm (최초 1회)
    if (!_arm_started) {
        RCLCPP_INFO(get_logger(), "Arming vehicle");
        arm();
        _arm_started = true;
        _arm_time = this->now();  // ARM 요청 시각 저장
    }

    // 2) Offboard 모드 엔게이지는 ARM 후 1초 뒤로 지연
    constexpr double OFFBOARD_DELAY_S = 1.0; // 1초 딜레이
    if (_arm_started && !_offboard_engaged) {
        auto dt = this->now() - _arm_time;
        if (dt.seconds() > OFFBOARD_DELAY_S) {
            RCLCPP_INFO(get_logger(), "Engaging Offboard Control Mode");
            engageOffboardControlMode();
            _offboard_engaged = true;
        }
    }

    // 3) 이후에는 heartbeat
    if (_offboard_engaged) {
        publishOffBoardControlMode();
    }

    /* 1) 상태기 업데이트 */
    //constexpr float dt = 0.05f;   // 50 ms

    bool target_lost = checkTargetTimeout();

    if (target_lost && !_target_lost_prev) {
        RCLCPP_INFO(get_logger(), "Target lost: State %s", stateName(_state).c_str());

    } else if (!target_lost && _target_lost_prev) {
        RCLCPP_INFO(get_logger(), "Target acquired");
    }

    _target_lost_prev = target_lost;

    // State machine
    switch (_state) {
    case State::Idle: {
        // No-op -- just spin
        break;
    }

    case State::Search: {

        if (!std::isnan(_tag.position.x())) {
            _approach_altitude = _current_local_position.z;
            switchToState(State::Approach);
            break;
        }

		auto waypoint_position = _search_waypoints[_search_waypoint_index];

		// Publish the waypoint as a trajectory setpoint
		publishTrajectorySetpointPos(
			waypoint_position.x(),
			waypoint_position.y(),
			waypoint_position.z(),
			0.0f // yaw_deg, set as needed
		);

		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;

            // If we have searched all waypoints, start over
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

        // Approach using position setpoints
        auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(), _approach_altitude);

		// Publish the target position as a trajectory setpoint
		publishTrajectorySetpointPos(
			target_position.x(),
			target_position.y(),
			target_position.z(),
			0.0f // yaw_deg, set as needed
		);

		if (positionReached(target_position)) {
			switchToState(State::Descend);
		}

        break;
    }

    case State::Descend: {

        if (target_lost) {
            RCLCPP_INFO(get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
            switchToState(State::Idle);
            return;
        }

		// Descend using velocity setpoints and P velocity controller for XY
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
        // Finished landing, stop the vehicle
        disarm();
		RCLCPP_INFO(get_logger(), "Landing finished, disarming vehicle");
		_ctrl_timer->cancel();
        break;
    }
    } // end switch/case
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrecisionLand>());
  rclcpp::shutdown();
  return 0;
}
