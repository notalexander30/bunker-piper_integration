#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "piper_x_aruco_wall_approach/srv/search_marker.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr std::size_t kJointCount = 6;

std::vector<double> degrees_to_radians(const std::vector<double> & degrees)
{
  std::vector<double> radians;
  radians.reserve(degrees.size());
  for (const auto value : degrees) {
    radians.push_back(value * M_PI / 180.0);
  }
  return radians;
}

bool valid_delta(const std::vector<double> & values)
{
  return values.size() == kJointCount && std::all_of(values.begin(), values.end(), [](const double value) {
    return std::isfinite(value);
  });
}

std::string normalise_direction(std::string direction)
{
  std::transform(direction.begin(), direction.end(), direction.begin(), [](const unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  direction.erase(std::remove_if(direction.begin(), direction.end(), [](const char ch) {
    return ch == ' ' || ch == '_' || ch == '-';
  }), direction.end());
  if (direction.empty() || direction == "auto" || direction == "reactive") {
    return "auto";
  }
  if (direction == "current" || direction == "check" || direction == "none") {
    return "current";
  }
  if (direction == "left") {
    return "left";
  }
  if (direction == "right") {
    return "right";
  }
  if (direction == "up") {
    return "up";
  }
  if (direction == "down") {
    return "down";
  }
  if (direction == "upleft") {
    return "up_left";
  }
  if (direction == "upright") {
    return "up_right";
  }
  if (direction == "downleft") {
    return "down_left";
  }
  if (direction == "downright") {
    return "down_right";
  }
  if (direction == "center" || direction == "centre") {
    return "center";
  }
  return direction;
}
}  // namespace

class SearchMarkerNode : public rclcpp::Node
{
public:
  explicit SearchMarkerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("search_marker_node", options)
  {
    aruco_pose_topic_ = declare_parameter<std::string>("aruco_pose_topic", "/aruco_single/pose");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "joint_states");
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    move_group_namespace_ = declare_parameter<std::string>("move_group_namespace", "");
    marker_id_ = declare_parameter<int>("marker_id", 6);
    marker_timeout_s_ = declare_parameter<double>("marker_timeout_s", 1.0);
    joint_state_timeout_s_ = declare_parameter<double>("joint_state_timeout_s", 1.0);
    settle_time_s_ = declare_parameter<double>("settle_time_s", 0.5);
    detection_window_frames_ = declare_parameter<int>("detection_window_frames", 5);
    required_detections_ = declare_parameter<int>("required_detections", 3);
    max_steps_ = declare_parameter<int>("max_steps", 100);
    velocity_scaling_ = declare_parameter<double>("velocity_scaling", 0.45);
    acceleration_scaling_ = declare_parameter<double>("acceleration_scaling", 0.35);
    planning_time_ = declare_parameter<double>("planning_time", 10.0);
    planning_attempts_ = declare_parameter<int>("planning_attempts", 10);
    max_single_joint_step_deg_ = declare_parameter<double>("max_single_joint_step_deg", 8.0);
    min_feedback_motion_deg_ = declare_parameter<double>("min_feedback_motion_deg", 1.0);
    physical_motion_timeout_s_ = declare_parameter<double>("physical_motion_timeout_s", 2.0);
    // The integrated PiPER stack uses the AgileX driver directly, not a
    // ros2_control controller manager. Physical execution is still verified
    // after every MoveIt step from fresh /feedback/joint_states motion.
    require_physical_hardware_ = declare_parameter<bool>("require_physical_hardware", false);
    controller_manager_service_ = declare_parameter<std::string>("controller_manager_service", "");
    center_step_scale_ = declare_parameter<double>("center_step_scale", 1.0);
    joint1_near_sweep_rad_ = declare_parameter<double>("joint1_near_sweep_rad", 1.6);
    joint4_reset_rad_ = declare_parameter<double>("joint4_reset_rad", 0.0);
    auto_horizontal_offset_rad_ = declare_parameter<double>("auto_horizontal_offset_rad", 2.0);
    auto_vertical_offset_rad_ = declare_parameter<double>("auto_vertical_offset_rad", 0.525);
    vertical_lift_joint2_rad_ = declare_parameter<double>("vertical_lift_joint2_rad", 0.05);
    vertical_lift_joint3_rad_ = declare_parameter<double>("vertical_lift_joint3_rad", 0.08);
    auto_sequence_ = declare_parameter<std::vector<std::string>>(
      "auto_sequence",
      std::vector<std::string>{
        "up", "left", "right", "right", "left",
        "up", "right", "left", "left", "right"});

    load_direction_deltas();
    normalise_parameters();

    data_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions data_subscription_options;
    data_subscription_options.callback_group = data_callback_group_;

    marker_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      aruco_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_marker_received_ = now();
        last_marker_header_stamp_s_ = stamp_to_seconds(*msg);
        marker_sequence_++;
        marker_detection_count_ = std::min(
          marker_detection_count_ + 1, detection_window_frames_);
        marker_confirmed_ = marker_detection_count_ >= required_detections_;
      },
      data_subscription_options);

    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_joint_received_ = now();
        last_joint_state_ = *msg;
      },
      data_subscription_options);

    if (controller_manager_service_.empty()) {
      controller_manager_service_ = move_group_namespace_.empty()
        ? "/controller_manager/list_hardware_components"
        : move_group_namespace_ + "/controller_manager/list_hardware_components";
    }
    hardware_client_ = create_client<controller_manager_msgs::srv::ListHardwareComponents>(
      controller_manager_service_);

    service_ = create_service<piper_x_aruco_wall_approach::srv::SearchMarker>(
      "/search_marker",
      [this](
        const std::shared_ptr<piper_x_aruco_wall_approach::srv::SearchMarker::Request> request,
        std::shared_ptr<piper_x_aruco_wall_approach::srv::SearchMarker::Response> response) {
        handle_search(request, response);
      },
      rmw_qos_profile_services_default,
      service_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Reactive search service ready: marker_id=%d, settle_time_s=%.2f, detection=%d/%d, controller_manager=%s",
      marker_id_, settle_time_s_, required_detections_, detection_window_frames_,
      controller_manager_service_.c_str());
  }

  void initialise_moveit(const rclcpp::Node::SharedPtr & node)
  {
    ensure_default_kinematics_parameters();
    moveit::planning_interface::MoveGroupInterface::Options options(
      planning_group_, "robot_description", move_group_namespace_);
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node,
      options);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    RCLCPP_INFO(
      get_logger(), "MoveIt ready for reactive marker search: group=%s, namespace=%s",
      planning_group_.c_str(), move_group_namespace_.c_str());
  }

private:
  template<typename ParameterT>
  void declare_if_missing(const std::string & name, const ParameterT & value)
  {
    if (!has_parameter(name)) {
      declare_parameter<ParameterT>(name, value);
    }
  }

  void ensure_default_kinematics_parameters()
  {
    const std::string prefix = "robot_description_kinematics." + planning_group_ + ".";
    declare_if_missing(prefix + "kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    declare_if_missing(prefix + "kinematics_solver_search_resolution", 0.005);
    declare_if_missing(prefix + "kinematics_solver_timeout", 0.05);
    declare_if_missing(prefix + "kinematics_solver_attempts", 3);
  }

  static std::optional<double> stamp_to_seconds(const geometry_msgs::msg::PoseStamped & msg)
  {
    const auto & stamp = msg.header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return std::nullopt;
    }
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  void load_direction_deltas()
  {
    direction_deltas_["left"] = degrees_to_radians(declare_parameter<std::vector<double>>(
      "direction_deltas_deg.left", std::vector<double>{5.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    direction_deltas_["right"] = degrees_to_radians(declare_parameter<std::vector<double>>(
      "direction_deltas_deg.right", std::vector<double>{-5.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    direction_deltas_["up"] = degrees_to_radians(declare_parameter<std::vector<double>>(
      "direction_deltas_deg.up", std::vector<double>{0.0, 0.0, 0.0, 5.0, 0.0, 0.0}));
    direction_deltas_["down"] = degrees_to_radians(declare_parameter<std::vector<double>>(
      "direction_deltas_deg.down", std::vector<double>{0.0, 0.0, 0.0, -5.0, 0.0, 0.0}));
    direction_deltas_["up_left"] = degrees_to_radians(declare_parameter<std::vector<double>>(
      "direction_deltas_deg.up_left", std::vector<double>{5.0, 0.0, 0.0, 5.0, 0.0, 0.0}));
    direction_deltas_["up_right"] = degrees_to_radians(declare_parameter<std::vector<double>>(
      "direction_deltas_deg.up_right", std::vector<double>{-5.0, 0.0, 0.0, 5.0, 0.0, 0.0}));
    direction_deltas_["down_left"] = degrees_to_radians(declare_parameter<std::vector<double>>(
      "direction_deltas_deg.down_left", std::vector<double>{5.0, 0.0, 0.0, -5.0, 0.0, 0.0}));
    direction_deltas_["down_right"] = degrees_to_radians(declare_parameter<std::vector<double>>(
      "direction_deltas_deg.down_right", std::vector<double>{-5.0, 0.0, 0.0, -5.0, 0.0, 0.0}));
  }

  void normalise_parameters()
  {
    if (!std::isfinite(marker_timeout_s_) || marker_timeout_s_ <= 0.0) {
      marker_timeout_s_ = 1.0;
    }
    if (!std::isfinite(joint_state_timeout_s_) || joint_state_timeout_s_ <= 0.0) {
      joint_state_timeout_s_ = 1.0;
    }
    if (!std::isfinite(settle_time_s_) || settle_time_s_ < 0.0) {
      settle_time_s_ = 0.5;
    }
    detection_window_frames_ = std::max(1, detection_window_frames_);
    required_detections_ = std::clamp(required_detections_, 1, detection_window_frames_);
    max_steps_ = std::max(0, max_steps_);
    if (!std::isfinite(max_single_joint_step_deg_) || max_single_joint_step_deg_ <= 0.0) {
      max_single_joint_step_deg_ = 8.0;
    }
    max_single_joint_step_rad_ = max_single_joint_step_deg_ * M_PI / 180.0;
    if (!std::isfinite(min_feedback_motion_deg_) || min_feedback_motion_deg_ <= 0.0) {
      min_feedback_motion_deg_ = 1.0;
    }
    min_feedback_motion_rad_ = min_feedback_motion_deg_ * M_PI / 180.0;
    if (!std::isfinite(physical_motion_timeout_s_) || physical_motion_timeout_s_ <= 0.0) {
      physical_motion_timeout_s_ = 2.0;
    }
    if (!std::isfinite(center_step_scale_) || center_step_scale_ <= 0.0) {
      center_step_scale_ = 1.0;
    }
    if (!std::isfinite(auto_horizontal_offset_rad_) || auto_horizontal_offset_rad_ <= 0.0) {
      auto_horizontal_offset_rad_ = 2.0;
    }
    if (!std::isfinite(auto_vertical_offset_rad_) || auto_vertical_offset_rad_ <= 0.0) {
      auto_vertical_offset_rad_ = 0.525;
    }
    if (!std::isfinite(vertical_lift_joint2_rad_) || vertical_lift_joint2_rad_ <= 0.0) {
      vertical_lift_joint2_rad_ = 0.05;
    }
    if (!std::isfinite(vertical_lift_joint3_rad_) || vertical_lift_joint3_rad_ <= 0.0) {
      vertical_lift_joint3_rad_ = 0.08;
    }

    for (auto & [direction, delta] : direction_deltas_) {
      if (!valid_delta(delta)) {
        RCLCPP_WARN(get_logger(), "Invalid delta for direction '%s'; disabling it", direction.c_str());
        delta.assign(kJointCount, 0.0);
        continue;
      }
      for (auto & value : delta) {
        value = std::clamp(value, -max_single_joint_step_rad_, max_single_joint_step_rad_);
      }
    }
  }

  bool marker_fresh_locked(const rclcpp::Time & current_time) const
  {
    if (!last_marker_received_) {
      return false;
    }
    const double monotonic_age_s = (current_time - *last_marker_received_).seconds();
    if (monotonic_age_s > marker_timeout_s_) {
      return false;
    }
    if (!last_marker_header_stamp_s_) {
      return true;
    }
    const double wall_age_s = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - *last_marker_header_stamp_s_;
    return wall_age_s <= marker_timeout_s_;
  }

  bool marker_fresh() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return marker_fresh_locked(now());
  }

  bool continuously_confirmed_marker() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return marker_confirmed_ && marker_fresh_locked(now());
  }

  bool confirm_marker(std::string & message)
  {
    if (continuously_confirmed_marker()) {
      message = "continuous ArUco detector already confirmed marker " + std::to_string(marker_id_);
      return true;
    }
    int detections = 0;
    uint64_t seen_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      seen_sequence = marker_sequence_;
    }

    for (int frame = 0; frame < detection_window_frames_; ++frame) {
      const auto deadline = now() + rclcpp::Duration::from_seconds(marker_timeout_s_);
      bool got_new_marker = false;
      while (rclcpp::ok() && now() < deadline) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (marker_sequence_ != seen_sequence && marker_fresh_locked(now())) {
            seen_sequence = marker_sequence_;
            got_new_marker = true;
            break;
          }
        }
        rclcpp::sleep_for(50ms);
      }
      if (got_new_marker) {
        detections++;
      }
    }

    message = "marker detections " + std::to_string(detections) + "/" +
      std::to_string(detection_window_frames_);
    return detections >= required_detections_;
  }

  std::optional<std::vector<double>> current_joint_values(std::string & message)
  {
    if (!move_group_) {
      message = "MoveIt interface is not ready";
      return std::nullopt;
    }
    const auto joint_names = move_group_->getJointNames();
    if (joint_names.size() != kJointCount) {
      message = "MoveIt returned " + std::to_string(joint_names.size()) +
        " joint names, expected " + std::to_string(kJointCount);
      return std::nullopt;
    }

    sensor_msgs::msg::JointState joint_state;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!last_joint_received_) {
        message = "No joint state received on " + joint_state_topic_;
        return std::nullopt;
      }
      const double age_s = (now() - *last_joint_received_).seconds();
      if (age_s > joint_state_timeout_s_) {
        message = "Joint state on " + joint_state_topic_ + " is stale";
        return std::nullopt;
      }
      joint_state = *last_joint_state_;
    }

    if (joint_state.name.empty() || joint_state.position.size() < joint_state.name.size()) {
      message = "Joint state on " + joint_state_topic_ + " has invalid name/position fields";
      return std::nullopt;
    }

    std::vector<double> values;
    values.reserve(joint_names.size());
    for (const auto & joint_name : joint_names) {
      auto iter = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
      if (iter == joint_state.name.end()) {
        message = "Joint state on " + joint_state_topic_ + " does not contain " + joint_name;
        return std::nullopt;
      }
      const auto index = static_cast<std::size_t>(std::distance(joint_state.name.begin(), iter));
      values.push_back(joint_state.position[index]);
    }

    if (!valid_delta(values)) {
      message = "Joint state on " + joint_state_topic_ + " contains invalid joint values";
      return std::nullopt;
    }
    return values;
  }

  static double max_abs_difference(
    const std::vector<double> & lhs,
    const std::vector<double> & rhs)
  {
    if (lhs.size() != rhs.size()) {
      return std::numeric_limits<double>::infinity();
    }
    double max_difference = 0.0;
    for (std::size_t i = 0; i < lhs.size(); ++i) {
      max_difference = std::max(max_difference, std::abs(lhs[i] - rhs[i]));
    }
    return max_difference;
  }

  template<typename PlanT>
  static auto joint_trajectory_from_plan(const PlanT & plan, int)
    -> decltype((plan.trajectory.joint_trajectory))
  {
    return plan.trajectory.joint_trajectory;
  }

  template<typename PlanT>
  static auto joint_trajectory_from_plan(const PlanT & plan, long)
    -> decltype((plan.trajectory_.joint_trajectory))
  {
    return plan.trajectory_.joint_trajectory;
  }

  std::optional<std::vector<double>> planned_final_joint_values(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::vector<std::string> & expected_joint_names) const
  {
    const auto & trajectory = joint_trajectory_from_plan(plan, 0);
    if (trajectory.points.empty()) {
      return std::nullopt;
    }

    const auto & final_point = trajectory.points.back();
    if (final_point.positions.size() != trajectory.joint_names.size()) {
      return std::nullopt;
    }

    std::vector<double> values;
    values.reserve(expected_joint_names.size());
    for (const auto & joint_name : expected_joint_names) {
      const auto iter = std::find(
        trajectory.joint_names.begin(),
        trajectory.joint_names.end(),
        joint_name);
      if (iter == trajectory.joint_names.end()) {
        return std::nullopt;
      }
      const auto index = static_cast<std::size_t>(
        std::distance(trajectory.joint_names.begin(), iter));
      values.push_back(final_point.positions[index]);
    }
    return values;
  }

  double planned_trajectory_duration_s(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan) const
  {
    const auto & trajectory = joint_trajectory_from_plan(plan, 0);
    if (trajectory.points.empty()) {
      return 0.0;
    }
    const auto & duration = trajectory.points.back().time_from_start;
    return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
  }

  bool wait_for_physical_feedback_motion(
    const std::vector<double> & before,
    std::string & message)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(physical_motion_timeout_s_);
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      rclcpp::sleep_for(100ms);
      std::string joint_message;
      const auto after = current_joint_values(joint_message);
      if (!after) {
        message = joint_message;
        continue;
      }
      if (max_abs_difference(before, *after) >= min_feedback_motion_rad_) {
        return true;
      }
    }

    message =
      "MoveIt reported execution success, but real joint feedback did not change by at least " +
      std::to_string(min_feedback_motion_deg_) +
      " deg. The AGX arm may be disabled, the control gate may be closed, or MoveIt may only be driving the ros2_control FakeSystem.";
    return false;
  }

  bool monitor_motion_for_marker(
    const std::vector<double> & before,
    const std::vector<double> & expected_final,
    const std::string & label,
    const double plan_duration_s,
    std::string & stage,
    std::string & message)
  {
    const double timeout_s = std::max(
      physical_motion_timeout_s_, plan_duration_s + physical_motion_timeout_s_);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    bool saw_physical_motion = false;
    std::string joint_message;

    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      if (continuously_confirmed_marker()) {
        move_group_->stop();
        stage = "marker_acquired_during_motion";
        message = "continuous ArUco detection acquired marker during search target '" +
          label + "'";
        return true;
      }

      const auto current = current_joint_values(joint_message);
      if (current) {
        saw_physical_motion =
          saw_physical_motion || max_abs_difference(before, *current) >= min_feedback_motion_rad_;
        if (max_abs_difference(expected_final, *current) <= min_feedback_motion_rad_) {
          stage = "search_motion_complete";
          message = "completed reactive search target '" + label + "'";
          return true;
        }
      }

      rclcpp::sleep_for(50ms);
    }

    if (continuously_confirmed_marker()) {
      move_group_->stop();
      stage = "marker_acquired_during_motion";
      message = "continuous ArUco detection acquired marker during search target '" +
        label + "'";
      return true;
    }

    if (saw_physical_motion) {
      stage = "search_motion_complete";
      message = "reactive search target '" + label +
        "' moved physically; final joint feedback did not settle before timeout";
      return true;
    }

    stage = "physical_motion";
    message =
      "MoveIt accepted the search trajectory, but real joint feedback did not change by at least " +
      std::to_string(min_feedback_motion_deg_) +
      " deg while monitoring for marker detection. The AGX arm may be disabled, the control gate "
      "may be closed, or MoveIt may only be driving the ros2_control FakeSystem.";
    return false;
  }

  bool execute_joint_delta(
    const std::vector<double> & delta,
    const bool execute,
    const std::string & direction,
    std::string & stage,
    std::string & message)
  {
    if (!valid_delta(delta)) {
      stage = "search_step_config";
      message = "invalid search delta for direction '" + direction + "'";
      return false;
    }
    if (!move_group_) {
      stage = "moveit_unavailable";
      message = "MoveIt interface is not ready";
      return false;
    }
    std::string joint_message;
    auto current = current_joint_values(joint_message);
    if (!current) {
      stage = "moveit_state";
      message = joint_message;
      return false;
    }
    const auto joint_names = move_group_->getJointNames();
    std::vector<double> target = *current;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      target[i] += delta[i];
    }
    const bool ok = execute_joint_target(target, execute, direction, *current, joint_names, stage, message);
    if (ok && stage == "search_motion_complete") {
      for (std::size_t i = 0; i < kJointCount; ++i) {
        cumulative_offset_[i] += delta[i];
      }
      message = "completed reactive search direction '" + direction + "'";
    }
    return ok;
  }

  bool execute_joint_target(
    const std::vector<double> & target,
    const bool execute,
    const std::string & label,
    const std::vector<double> & current,
    const std::vector<std::string> & joint_names,
    std::string & stage,
    std::string & message)
  {
    if (!valid_delta(target)) {
      stage = "search_step_config";
      message = "invalid search target for '" + label + "'";
      return false;
    }
    if (!move_group_) {
      stage = "moveit_unavailable";
      message = "MoveIt interface is not ready";
      return false;
    }
    move_group_->setStartStateToCurrentState();
    move_group_->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      stage = "search_plan";
      message = "MoveIt planning failed for reactive search target '" + label + "'";
      return false;
    }
    const auto planned_final = planned_final_joint_values(plan, joint_names);
    if (planned_final && max_abs_difference(current, *planned_final) < min_feedback_motion_rad_) {
      stage = "search_motion_skipped";
      message =
        "reactive search target '" + label +
        "' was skipped because the planned physical motion is below " +
        std::to_string(min_feedback_motion_deg_) +
        " deg, likely because the joint is at or near its limit";
      return true;
    }
    if (!execute) {
      stage = "plan_only";
      message = "reactive search target '" + label + "' planned; execute=false so no motion was commanded";
      return true;
    }
    const auto execute_result = move_group_->execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
      stage = "search_execution";
      message = "MoveIt failed to execute reactive search target '" + label + "'";
      return false;
    }

    if (continuously_confirmed_marker()) {
      stage = "marker_acquired_during_motion";
      message = "continuous ArUco detection acquired marker during search target '" +
        label + "'";
      return true;
    }

    if (!wait_for_physical_feedback_motion(current, message)) {
      stage = "physical_motion";
      return false;
    }

    stage = "search_motion_complete";
    message = "completed reactive search target '" + label + "'";
    return true;
  }

  bool verify_physical_hardware(std::string & stage, std::string & message)
  {
    if (!require_physical_hardware_) {
      return true;
    }
    if (!hardware_client_) {
      stage = "physical_hardware";
      message = "controller manager hardware client is not initialized";
      return false;
    }
    if (!hardware_client_->wait_for_service(500ms)) {
      stage = "physical_hardware";
      message = "controller manager service unavailable: " + controller_manager_service_;
      return false;
    }

    auto request =
      std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>();
    auto future = hardware_client_->async_send_request(request);
    if (future.wait_for(1s) != std::future_status::ready) {
      stage = "physical_hardware";
      message = "timed out checking hardware through " + controller_manager_service_;
      return false;
    }

    const auto response = future.get();
    if (!response) {
      stage = "physical_hardware";
      message = controller_manager_service_ + " returned no hardware response";
      return false;
    }

    bool saw_active_hardware = false;
    for (const auto & component : response->component) {
      if (
        component.name == "FakeSystem" ||
        component.class_type.find("mock_components") != std::string::npos ||
        component.class_type.find("GenericSystem") != std::string::npos)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Using AgileX MoveIt demo FakeSystem as a trajectory sampler into the AGX driver control topic");
        saw_active_hardware = true;
        continue;
      }
      if (component.state.label == "active") {
        saw_active_hardware = true;
      }
    }
    if (response->component.empty()) {
      stage = "physical_hardware";
      message = "no hardware components reported by " + controller_manager_service_;
      return false;
    }
    if (!saw_active_hardware) {
      stage = "physical_hardware";
      message = "no active hardware components reported by " + controller_manager_service_;
      return false;
    }
    return true;
  }

  bool execute_absolute_step(
    const std::vector<double> & target,
    const bool execute,
    const std::string & label,
    int & steps_used,
    std::string & found_at_pose,
    std::string & stage,
    std::string & message)
  {
    if (!move_group_) {
      stage = "moveit_unavailable";
      message = "MoveIt interface is not ready";
      return false;
    }
    std::string joint_message;
    auto current = current_joint_values(joint_message);
    if (!current) {
      stage = "moveit_state";
      message = joint_message;
      return false;
    }
    const auto joint_names = move_group_->getJointNames();
    if (!execute_joint_target(target, execute, label, *current, joint_names, stage, message)) {
      return false;
    }
    steps_used++;
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(settle_time_s_)));

    std::string confirmation_message;
    if (marker_fresh() && confirm_marker(confirmation_message)) {
      found_at_pose = "reactive_" + label;
      stage = "complete";
      message = "marker acquired after reactive search target '" + label + "'";
      return true;
    }
    return false;
  }

  std::vector<double> center_delta(std::string & message) const
  {
    (void)message;
    std::vector<double> delta(kJointCount, 0.0);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      delta[i] = std::clamp(
        -cumulative_offset_[i] * center_step_scale_,
        -max_single_joint_step_rad_,
        max_single_joint_step_rad_);
    }
    return delta;
  }

  bool perform_step(
    const std::string & requested_direction,
    const bool execute,
    int & steps_used,
    std::string & found_at_pose,
    std::string & stage,
    std::string & message)
  {
    const auto direction = normalise_direction(requested_direction);
    if (direction == "current") {
      stage = "current_view";
      message = "checked current camera view without motion";
    } else {
      std::vector<double> delta;
      if (direction == "center") {
        delta = center_delta(message);
      } else {
        const auto iter = direction_deltas_.find(direction);
        if (iter == direction_deltas_.end()) {
          stage = "search_direction";
          message = "unsupported reactive search direction '" + requested_direction + "'";
          return false;
        }
        delta = iter->second;
      }
      if (!execute_joint_delta(delta, execute, direction, stage, message)) {
        return false;
      }
      steps_used++;
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settle_time_s_)));
    }

    std::string confirmation_message;
    if (marker_fresh() && confirm_marker(confirmation_message)) {
      found_at_pose = direction == "current" ? "current_view" : "reactive_" + direction;
      stage = "complete";
      message = direction == "current" ?
        "marker already visible in current camera view" :
        "marker acquired after reactive search direction '" + direction + "'";
      return true;
    }
    return false;
  }

  std::optional<std::pair<double, double>> joint_bounds(
    const std::size_t joint_index,
    std::string & message) const
  {
    if (!move_group_) {
      message = "MoveIt interface is not ready";
      return std::nullopt;
    }
    const auto joint_names = move_group_->getJointNames();
    if (joint_index >= joint_names.size()) {
      message = "joint index " + std::to_string(joint_index) + " is outside MoveIt joint list";
      return std::nullopt;
    }
    const auto robot_model = move_group_->getRobotModel();
    if (!robot_model) {
      message = "MoveIt robot model is not available";
      return std::nullopt;
    }
    const auto & bounds = robot_model->getVariableBounds(joint_names[joint_index]);
    if (!bounds.position_bounded_) {
      message = "MoveIt joint " + joint_names[joint_index] + " has no bounded position limits";
      return std::nullopt;
    }
    return std::make_pair(bounds.min_position_, bounds.max_position_);
  }

  bool maybe_check_marker_after_no_motion(
    const std::string & label,
    std::string & found_at_pose,
    std::string & stage,
    std::string & message)
  {
    std::string confirmation_message;
    if (marker_fresh() && confirm_marker(confirmation_message)) {
      found_at_pose = "reactive_" + label;
      stage = "complete";
      message = "marker acquired at reactive search target '" + label + "'";
      return true;
    }
    return false;
  }

  bool move_target_and_allow_skips(
    const std::vector<double> & target,
    const std::string & label,
    int & steps_used,
    std::string & found_at_pose,
    std::string & stage,
    std::string & message,
    bool & motion_ok)
  {
    motion_ok = true;
    const bool found = execute_absolute_step(
      target, true, label, steps_used, found_at_pose, stage, message);
    if (found) {
      return true;
    }
    if (stage == "search_motion_complete" || stage == "search_motion_skipped") {
      return maybe_check_marker_after_no_motion(label, found_at_pose, stage, message);
    }
    motion_ok = false;
    return false;
  }

  bool run_joint_limit_sweep(
    int & steps_used,
    std::string & found_at_pose,
    std::string & stage,
    std::string & message)
  {
    std::string joint_message;
    auto current = current_joint_values(joint_message);
    if (!current) {
      stage = "moveit_state";
      message = joint_message;
      return false;
    }

    const auto joint1_bounds = joint_bounds(0, message);
    if (!joint1_bounds) {
      stage = "moveit_bounds";
      return false;
    }
    const auto joint4_bounds = joint_bounds(3, message);
    if (!joint4_bounds) {
      stage = "moveit_bounds";
      return false;
    }

    const auto up_iter = direction_deltas_.find("up");
    const auto left_iter = direction_deltas_.find("left");
    const auto right_iter = direction_deltas_.find("right");
    if (up_iter == direction_deltas_.end() || left_iter == direction_deltas_.end() ||
      right_iter == direction_deltas_.end())
    {
      stage = "search_direction";
      message = "full search requires configured up, left, and right deltas";
      return false;
    }

    const double up_step = up_iter->second[3];
    if (!std::isfinite(up_step) || std::abs(up_step) < 1e-6) {
      stage = "search_step_config";
      message = "full search requires a non-zero joint4 up delta";
      return false;
    }
    const double up_limit = up_step > 0.0 ? joint4_bounds->second : joint4_bounds->first;
    const double near_positive = std::clamp(joint1_near_sweep_rad_, joint1_bounds->first, joint1_bounds->second);
    const double near_negative = std::clamp(-joint1_near_sweep_rad_, joint1_bounds->first, joint1_bounds->second);
    const double look_left_offset = left_iter->second[0];
    const double look_right_offset = right_iter->second[0];
    const double limit_margin = std::max(0.5 * M_PI / 180.0, min_feedback_motion_rad_);

    std::vector<double> sectors;
    sectors.push_back(std::clamp((*current)[0], joint1_bounds->first, joint1_bounds->second));
    sectors.push_back(near_positive);
    sectors.push_back(joint1_bounds->second);
    sectors.push_back(near_negative);
    sectors.push_back(joint1_bounds->first);

    auto append_unique_sector = [](std::vector<double> & values, const double value) {
      constexpr double kDuplicateTolerance = 0.02;
      if (std::none_of(values.begin(), values.end(), [value](const double existing) {
          return std::abs(existing - value) < kDuplicateTolerance;
        }))
      {
        values.push_back(value);
      }
    };
    std::vector<double> unique_sectors;
    for (const auto sector : sectors) {
      append_unique_sector(unique_sectors, sector);
    }

    for (std::size_t sector_index = 0; sector_index < unique_sectors.size(); ++sector_index) {
      const double sector_joint1 = unique_sectors[sector_index];
      current = current_joint_values(joint_message);
      if (!current) {
        stage = "moveit_state";
        message = joint_message;
        return false;
      }

      std::vector<double> target = *current;
      target[3] = std::clamp(joint4_reset_rad_, joint4_bounds->first, joint4_bounds->second);
      if (sector_index > 0) {
        bool motion_ok = true;
        if (move_target_and_allow_skips(
            target, "j4_reset_before_sector_" + std::to_string(sector_index),
            steps_used, found_at_pose, stage, message, motion_ok))
        {
          return true;
        }
        if (!motion_ok) {
          return false;
        }
      }

      current = current_joint_values(joint_message);
      if (!current) {
        stage = "moveit_state";
        message = joint_message;
        return false;
      }
      target = *current;
      target[0] = sector_joint1;
      target[3] = std::clamp(joint4_reset_rad_, joint4_bounds->first, joint4_bounds->second);
      bool motion_ok = true;
      if (move_target_and_allow_skips(
          target, "j1_sector_" + std::to_string(sector_index),
          steps_used, found_at_pose, stage, message, motion_ok))
      {
        return true;
      }
      if (!motion_ok) {
        return false;
      }

      while (rclcpp::ok()) {
        current = current_joint_values(joint_message);
        if (!current) {
          stage = "moveit_state";
          message = joint_message;
          return false;
        }
        const double remaining = up_limit - (*current)[3];
        if (std::abs(remaining) <= limit_margin || remaining * up_step <= 0.0) {
          break;
        }

        target = *current;
        target[0] = sector_joint1;
        if (std::abs(remaining) <= std::abs(up_step)) {
          target[3] = up_limit;
        } else {
          target[3] = (*current)[3] + up_step;
        }
        target[3] = std::clamp(target[3], joint4_bounds->first, joint4_bounds->second);
        if (move_target_and_allow_skips(
            target, "sector_" + std::to_string(sector_index) + "_j4_up",
            steps_used, found_at_pose, stage, message, motion_ok))
        {
          return true;
        }
        if (!motion_ok) {
          return false;
        }

        for (const auto & look : {
            std::make_pair(std::string("left"), look_left_offset),
            std::make_pair(std::string("right"), look_right_offset),
            std::make_pair(std::string("right_far"), 2.0 * look_right_offset),
            std::make_pair(std::string("left_far"), 2.0 * look_left_offset)})
        {
          current = current_joint_values(joint_message);
          if (!current) {
            stage = "moveit_state";
            message = joint_message;
            return false;
          }
          target = *current;
          target[0] = std::clamp(sector_joint1 + look.second, joint1_bounds->first, joint1_bounds->second);
          if (move_target_and_allow_skips(
              target,
              "sector_" + std::to_string(sector_index) + "_look_" + look.first,
              steps_used, found_at_pose, stage, message, motion_ok))
          {
            return true;
          }
          if (!motion_ok) {
            return false;
          }
        }

        current = current_joint_values(joint_message);
        if (!current) {
          stage = "moveit_state";
          message = joint_message;
          return false;
        }
        target = *current;
        target[0] = sector_joint1;
        if (move_target_and_allow_skips(
            target, "sector_" + std::to_string(sector_index) + "_recenter_j1",
            steps_used, found_at_pose, stage, message, motion_ok))
        {
          return true;
        }
        if (!motion_ok) {
          return false;
        }
      }
    }

    current = current_joint_values(joint_message);
    if (!current) {
      stage = "moveit_state";
      message = joint_message;
      return false;
    }
    std::vector<double> target = *current;
    target[0] = std::clamp(0.0, joint1_bounds->first, joint1_bounds->second);
    target[3] = std::clamp(joint4_reset_rad_, joint4_bounds->first, joint4_bounds->second);
    bool motion_ok = true;
    if (move_target_and_allow_skips(
        target, "search_failed_return_j1_j4_zero",
        steps_used, found_at_pose, stage, message, motion_ok))
    {
      return true;
    }
    if (!motion_ok) {
      return false;
    }

    stage = "search_complete";
    message =
      "marker_not_found after full joint-limit sweep; returned joint1 and joint4 to zero";
    return false;
  }

  bool run_direction_sequence(
    int & steps_used,
    std::string & found_at_pose,
    std::string & stage,
    std::string & message)
  {
    std::string joint_message;
    const auto initial = current_joint_values(joint_message);
    if (!initial) {
      stage = "moveit_state";
      message = joint_message;
      return false;
    }

    const auto joint1_bounds = joint_bounds(0, message);
    const auto joint2_bounds = joint_bounds(1, message);
    const auto joint3_bounds = joint_bounds(2, message);
    const auto joint4_bounds = joint_bounds(3, message);
    if (!joint1_bounds || !joint2_bounds || !joint3_bounds || !joint4_bounds) {
      stage = "moveit_bounds";
      return false;
    }
    const double horizontal = std::abs(auto_horizontal_offset_rad_);
    // Preserve the previously validated lift directions: positive J2 and
    // negative J3. Keep both increments small; J4 compensates their net pitch
    // change.
    const double lift_j2 = std::abs(vertical_lift_joint2_rad_);
    const double lift_j3 = -std::abs(vertical_lift_joint3_rad_);
    const double original_j2 = std::clamp((*initial)[1], joint2_bounds->first, joint2_bounds->second);
    const double original_j3 = std::clamp((*initial)[2], joint3_bounds->first, joint3_bounds->second);
    const double original_j4 = std::clamp((*initial)[3], joint4_bounds->first, joint4_bounds->second);

    std::vector<double> sectors{
      joint1_bounds->second,
      std::clamp(M_PI / 2.0, joint1_bounds->first, joint1_bounds->second),
      std::clamp(0.0, joint1_bounds->first, joint1_bounds->second),
      std::clamp(-M_PI / 2.0, joint1_bounds->first, joint1_bounds->second),
      joint1_bounds->first};

    while (rclcpp::ok() && (max_steps_ == 0 || steps_used < max_steps_)) {
      for (std::size_t sector_index = 0; sector_index < sectors.size(); ++sector_index) {
        const double sector = sectors[sector_index];
        double level_j2 = original_j2;
        double level_j3 = original_j3;

        std::vector<double> sector_start = *initial;
        sector_start[0] = sector;
        sector_start[1] = original_j2;
        sector_start[2] = original_j3;
        sector_start[3] = original_j4;
        bool motion_ok = true;
        if (move_target_and_allow_skips(
            sector_start, "sector_" + std::to_string(sector_index) + "_start_original_height",
            steps_used, found_at_pose, stage, message, motion_ok))
        {
          return true;
        }
        if (!motion_ok) {
          return false;
        }

        while (rclcpp::ok() && (max_steps_ == 0 || steps_used < max_steps_)) {
        const double level_j4 = std::clamp(
          original_j4 - ((level_j2 - original_j2) + (level_j3 - original_j3)),
          joint4_bounds->first, joint4_bounds->second);
        const double look_up = std::clamp(
          level_j4 - auto_vertical_offset_rad_, joint4_bounds->first, joint4_bounds->second);
        const std::vector<std::pair<std::string, std::vector<double>>> views = {
          {"sector_" + std::to_string(sector_index) + "_right", {sector - horizontal, level_j2, level_j3, level_j4, (*initial)[4], (*initial)[5]}},
          {"sector_" + std::to_string(sector_index) + "_left", {sector + horizontal, level_j2, level_j3, level_j4, (*initial)[4], (*initial)[5]}},
          {"sector_" + std::to_string(sector_index) + "_up", {sector, level_j2, level_j3, look_up, (*initial)[4], (*initial)[5]}},
          {"sector_" + std::to_string(sector_index) + "_up_left", {sector + horizontal, level_j2, level_j3, look_up, (*initial)[4], (*initial)[5]}},
          {"sector_" + std::to_string(sector_index) + "_j4_down", {sector, level_j2, level_j3, level_j4, (*initial)[4], (*initial)[5]}}};
          for (const auto & [label, unclamped] : views) {
            std::vector<double> target = unclamped;
            target[0] = std::clamp(target[0], joint1_bounds->first, joint1_bounds->second);
            target[1] = std::clamp(target[1], joint2_bounds->first, joint2_bounds->second);
            target[2] = std::clamp(target[2], joint3_bounds->first, joint3_bounds->second);
            target[3] = std::clamp(target[3], joint4_bounds->first, joint4_bounds->second);
            bool motion_ok = true;
            if (move_target_and_allow_skips(target, label, steps_used, found_at_pose, stage, message, motion_ok)) {
              return true;
            }
            if (!motion_ok) {
              return false;
            }
            if (max_steps_ > 0 && steps_used >= max_steps_) {
              break;
            }
          }
          const double next_j2 = level_j2 + lift_j2;
          const double next_j3 = level_j3 + lift_j3;
          if (next_j2 > joint2_bounds->second - min_feedback_motion_rad_ ||
            next_j3 < joint3_bounds->first + min_feedback_motion_rad_)
          {
            break;
          }
          level_j2 = next_j2;
          level_j3 = next_j3;
          std::vector<double> lift_target = *initial;
          lift_target[0] = sector;
          lift_target[1] = level_j2;
          lift_target[2] = level_j3;
          lift_target[3] = std::clamp(
            original_j4 - ((level_j2 - original_j2) + (level_j3 - original_j3)),
            joint4_bounds->first, joint4_bounds->second);
          if (move_target_and_allow_skips(lift_target, "sector_" + std::to_string(sector_index) + "_lift", steps_used, found_at_pose, stage, message, motion_ok)) {
            return true;
          }
          if (!motion_ok) {
            return false;
          }
        }
      }
    }
    stage = "search_complete";
    message = max_steps_ == 0 ? "search stopped" : "marker_not_found after configured search limit";
    return false;
  }

  void handle_search(
    const std::shared_ptr<piper_x_aruco_wall_approach::srv::SearchMarker::Request> request,
    std::shared_ptr<piper_x_aruco_wall_approach::srv::SearchMarker::Response> response)
  {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    response->marker_id = marker_id_;
    response->marker_found = false;
    response->success = false;
    response->found_at_pose = "";
    response->poses_checked = 0;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      marker_detection_count_ = 0;
      marker_confirmed_ = false;
    }

    std::string found_at_pose;
    std::string stage;
    std::string message;
    int steps_used = 0;

    if (perform_step("current", false, steps_used, found_at_pose, stage, message)) {
      response->success = true;
      response->marker_found = true;
      response->found_at_pose = found_at_pose;
      response->stage = stage;
      response->message = message;
      response->poses_checked = steps_used;
      return;
    }

    const auto direction = normalise_direction(request->direction);

    if (!request->execute) {
      response->stage = "marker_not_found";
      response->message = "marker is not visible in current view and execute=false prevents reactive search motion";
      response->poses_checked = 0;
      return;
    }

    if (!verify_physical_hardware(response->stage, response->message)) {
      response->poses_checked = 0;
      return;
    }

    if (direction != "auto") {
      const bool found = perform_step(
        direction, true, steps_used, found_at_pose, response->stage, response->message);
      response->poses_checked = steps_used;
      if (found) {
        response->success = true;
        response->marker_found = true;
        response->found_at_pose = found_at_pose;
        return;
      }
      if (
        response->stage == "search_motion_complete" ||
        response->stage == "search_motion_skipped" ||
        response->stage == "current_view" ||
        response->stage == "plan_only")
      {
        response->success = true;
        response->stage = "step_complete";
        response->message = "reactive search step completed; marker_not_found";
      }
      return;
    }

    const bool found = run_direction_sequence(
      steps_used, found_at_pose, response->stage, response->message);
    response->poses_checked = steps_used;
    if (found) {
      response->success = true;
      response->marker_found = true;
      response->found_at_pose = found_at_pose;
    }
  }

  std::string aruco_pose_topic_;
  std::string joint_state_topic_;
  std::string planning_group_;
  std::string move_group_namespace_;
  int marker_id_{};
  double marker_timeout_s_{};
  double joint_state_timeout_s_{};
  double settle_time_s_{};
  int detection_window_frames_{};
  int required_detections_{};
  int max_steps_{};
  double velocity_scaling_{};
  double acceleration_scaling_{};
  double planning_time_{};
  int planning_attempts_{};
  double max_single_joint_step_deg_{};
  double max_single_joint_step_rad_{};
  double min_feedback_motion_deg_{};
  double min_feedback_motion_rad_{};
  double physical_motion_timeout_s_{};
  bool require_physical_hardware_{};
  std::string controller_manager_service_;
  double center_step_scale_{};
  double joint1_near_sweep_rad_{};
  double joint4_reset_rad_{};
  double auto_horizontal_offset_rad_{};
  double auto_vertical_offset_rad_{};
  double vertical_lift_joint2_rad_{};
  double vertical_lift_joint3_rad_{};
  std::vector<std::string> auto_sequence_;
  std::unordered_map<std::string, std::vector<double>> direction_deltas_;
  std::vector<double> cumulative_offset_ = std::vector<double>(kJointCount, 0.0);

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::CallbackGroup::SharedPtr data_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr hardware_client_;
  rclcpp::Service<piper_x_aruco_wall_approach::srv::SearchMarker>::SharedPtr service_;

  mutable std::mutex data_mutex_;
  std::mutex operation_mutex_;
  std::optional<rclcpp::Time> last_marker_received_;
  std::optional<double> last_marker_header_stamp_s_;
  uint64_t marker_sequence_{0};
  int marker_detection_count_{0};
  bool marker_confirmed_{false};
  std::optional<rclcpp::Time> last_joint_received_;
  std::optional<sensor_msgs::msg::JointState> last_joint_state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SearchMarkerNode>();
  node->initialise_moveit(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
