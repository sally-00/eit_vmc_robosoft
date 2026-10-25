#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <string>

namespace franka_eit_vmc {
using V3 = Eigen::Vector3d;
using V7 = Eigen::Matrix<double, 7, 1>;
using J67 = Eigen::Matrix<double, 6, 7>;
enum class Task { Ready, Joystick, Ball };
enum class JoystickMechanism { Legacy, InwardArc };
enum class Mode { Hold, Ready, Approach, Preload, Interact, Retract, Done, Stopping, Fault };
enum class Fault {
  None,
  Period,
  Nonfinite,
  Workspace,
  JointLimit,
  JointSpeed,
  CartesianSpeed,
  Joint7,
  SensorStale,
  SensorPosition,
  ContactLoss,
  ApproachTimeout,
  ReadyTimeout,
  Tracking,
  Heartbeat,
  Robot,
  RetractTimeout,
  JoystickFeedback
};
enum class Request { None, Ready, Start, Stop };
struct Config {
  Task task{Task::Joystick};
  V3 ee_to_pad{0., 0., 0.14};
  Eigen::Matrix3d ee_R_pad{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d ready_pad_rotation{
      (Eigen::AngleAxisd(3.141592653589793, V3::UnitX())).toRotationMatrix()};
  V3 ready_pad_position{0.45, 0., 0.35};
  V3 workspace_min{0.15, -0.18, 0.05}, workspace_max{0.75, 0.18, 0.70};
  V7 q_min{(V7() << -2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508).finished()};
  V7 q_max{(V7() << 2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508).finished()};
  V7 posture_reference{V7::Zero()};
  bool use_saved_posture{false};
  bool update_ball_attachment{false};
  double ready_speed{0.015}, ready_angular_speed{0.12}, ready_timeout{90.};
  double approach_speed{0.004}, approach_distance{0.020}, approach_timeout{8.};
  double approach_contact_confirm_s{0.25};
  double approach_contact_speed{0.002}, approach_contact_travel_m{0.001};
  double retract_distance{0.015}, preload{3.}, preload_time{0.55};
  double hold_k{600.}, hold_d{35.}, ready_k{600.}, ready_d{35.};
  double hold_kr{30.}, hold_dr{3.};
  double ready_kr{90.}, ready_dr{5.}, ready_moment_limit{4.};
  double approach_kr{90.}, approach_dr{5.}, approach_moment_limit{4.};
  double coupling_k{700.}, coupling_d{30.}, force_xy_limit{3.};
  double normal_k{800.}, normal_d{60.}, normal_force_limit{7.};
  double orientation_k{3.}, orientation_d{1.}, moment_limit{1.};
  double null_k{2.}, null_d{1.}, torque_rate{50.}, joint_margin{0.06};
  double joint_speed_limit{0.6}, point_speed_limit{0.10}, angular_speed_limit{0.5};
  double q7_tolerance{0.35};
  double sensor_timeout{0.30}, heartbeat_timeout{0.30}, max_period{0.010},
         idle_max_period{0.020};
  double active_radius{0.036};
  double joystick_length{0.092}, joystick_angle{0.20943951023931953}, joystick_duration{9.1};
  // Only InwardArc uses these geometry/guide settings and live shaft feedback.
  JoystickMechanism joystick_mechanism{JoystickMechanism::Legacy};
  double joystick_inward_arc_m{.010};
  V3 joystick_pivot_m{.45, 0., .035};
  double joystick_alignment_half_span_m{.080}, joystick_slider_radius_m{.036};
  double joystick_alignment_spring_k{234.375}, joystick_alignment_spring_d{78.125};
  double joystick_feedback_timeout{.20};
  double angle_speed{0.28}, angle_acceleration{2.0};
  double initial_ball_x{0.45}, roll_distance{0.020}, ball_duration{8.};
  double virtual_mass{0.6}, virtual_damping{6.}, goal_k{80.}, goal_force_limit{2.},
      progress_speed{0.0125};
  void validate() const;
};
struct Sample {
  V7 q{V7::Zero()}, dq{V7::Zero()}, coriolis{V7::Zero()}, previous_torque{V7::Zero()};
  V3 ee_position{V3::Zero()}, external_force{V3::Zero()}, external_moment{V3::Zero()};
  Eigen::Matrix3d ee_rotation{Eigen::Matrix3d::Identity()};
  J67 jacobian{J67::Zero()};
};
struct Contact {
  uint64_t sequence{0};
  double acquired_steady_s{-1.};
  bool contact{false}, contact_candidate{false}, position_valid{false}, physical_model{false},
      measurement_valid{false};
  double signal_strength{0.}, contact_threshold{0.}, pattern_distance{0.},
      pattern_support_limit{0.}, predicted_radius_mm{0.};
  double x{0.}, y{0.};
};
struct ShaftFeedback {
  uint64_t sequence{0};
  double acquired_steady_s{-1.}, angle{0.}, rate{0.};
  bool valid{false}, clipped{false};
};
struct Output {
  Mode mode{Mode::Hold};
  Fault fault{Fault::None};
  V7 torque{V7::Zero()};
  V3 pad_position{V3::Zero()}, point{V3::Zero()}, reference{V3::Zero()}, attachment{V3::Zero()};
  V3 coupling_force{V3::Zero()};
  V3 requested_force{V3::Zero()}, requested_moment{V3::Zero()};
  double progress{0.}, progress_velocity{0.}, goal{0.}, angle_command{0.}, trial_time{0.};
  bool use_eit{true}, finished{false}, retract_incomplete{false};
  JoystickMechanism joystick_mechanism{JoystickMechanism::Legacy};
  double inward_arc_m{0.}, shaft_angle{0.}, shaft_rate{0.}, shaft_age_s{0.}, force_scale{1.};
  bool shaft_fresh{false}, shaft_clipped{false}, slider_at_stop{false};
  V3 alignment_force{V3::Zero()}, alignment_centre{V3::Zero()}, alignment_target{V3::Zero()};
  Eigen::Vector2d slider{Eigen::Vector2d::Zero()};
  uint64_t sensor_sequence{0};
  bool sensor_contact{false}, sensor_contact_candidate{false}, sensor_fresh{false}, sensor_measurement_valid{false},
      sensor_position_valid{false}, sensor_physical_model{false};
  double sensor_age_s{0.}, sensor_signal_strength{0.}, sensor_contact_threshold{0.};
  double sensor_pattern_distance{0.}, sensor_pattern_support_limit{0.},
      sensor_predicted_radius_mm{0.};
};
Eigen::Matrix3d skew(const V3& x);
J67 point_jacobian(const J67& ee_jacobian, const V3& world_offset);
V3 orientation_error(const Eigen::Matrix3d& actual, const Eigen::Matrix3d& desired);
class Core {
 public:
  explicit Core(Config config);
  void activate(const Sample& s);
  bool set_condition(bool use_eit);
  bool set_joystick_mechanism(JoystickMechanism mechanism, double inward_arc_m);
  void robot_fault() { fail(Fault::Robot); }
  Output step(const Sample& s, const Contact& contact, double now, double dt, bool heartbeat,
              Request request, const ShaftFeedback& shaft = ShaftFeedback{});
  Mode mode() const { return mode_; }
  Fault fault() const { return fault_; }
  const Config& config() const { return c_; }

 private:
  Config c_;
  Mode mode_{Mode::Hold};
  Fault fault_{Fault::None};
  bool use_eit_{true};
  V7 posture_{V7::Zero()};
  V3 hold_{V3::Zero()}, reference_{V3::Zero()}, attachment_{V3::Zero()}, h0_{V3::Zero()};
  Eigen::Matrix3d hold_rotation_{Eigen::Matrix3d::Identity()};
  V3 phase_start_position_{V3::Zero()}, initial_face_{V3::Zero()};
  Eigen::Quaterniond phase_start_rotation_{Eigen::Quaterniond::Identity()};
  double phase_time_{0.}, trial_time_{0.}, settle_time_{0.}, ready_duration_{1.},
      approach_progress_m_{0.}, approach_confirm_time_{0.},
      approach_contact_start_progress_m_{0.},
      unloaded_retract_time_{0.},
      normal_anchor_{0.};
  int approach_confirm_frames_{0};
  double z_{0.}, dz_{0.}, z0_{0.}, observed_{0.}, goal_{0.};
  Eigen::Vector2d slider_{Eigen::Vector2d::Zero()};
  int ready_stage_{0};
  uint64_t last_sequence_{0};
  bool activated_{false}, retract_incomplete_{false};
  void fail(Fault fault);
  V3 face(const Sample& s) const;
  Eigen::Matrix3d pad_rotation(const Sample& s) const;
  void begin_ready(const Sample& s);
  void begin_retract(const Sample& s);
  bool inside(const V3& p) const;
};
const char* mechanism_name(JoystickMechanism mechanism);
const char* mode_name(Mode mode);
const char* fault_name(Fault fault);
}  // namespace franka_eit_vmc
