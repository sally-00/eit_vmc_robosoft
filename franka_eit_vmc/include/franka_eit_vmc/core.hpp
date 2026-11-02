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
enum class JoystickAxis { X, Y };  // Base direction of shaft tilt, not rotation axis.
enum class ContactSource { Baseline, Eit, Kinematic };
enum class BallAxis { X, Y };
enum class Mode { Hold, Ready, Approach, Preload, Interact, Retract, Done, Stopping, Fault };
enum class Fault {
  None,
  Period,
  Nonfinite,
  Workspace,
  JointLimit,
  JointSpeed,
  CartesianSpeed,
  Joint7,  // Reserved historical fault code; no longer emitted.
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
  // New direct ball VMC. Both position-feedback conditions recover by default.
  // Set recover=false only for the held-brake recovery ablation; use EIT in both.
  bool ball_recover_contact{true}, ball_initial_attachment{false};
  double ball_baseline_motion_ratio{1.};  // ball travel / pad travel; sliding on base
  double ball_goal_d{20.}, ball_force_slew{10.}, ball_pause_force_slew{50.},
      ball_goal_ramp_s{.4};
  double ball_reference_speed_m_s{.030};
  double ball_reference_advance_speed_m_s{.004}, ball_reference_advance_limit_m{.04},
      ball_reference_advance_start_error_m{.05};
  double ball_normal_mass{.4}, ball_normal_damping{20.}, ball_normal_speed{.015},
      ball_normal_travel{.025}, ball_normal_up_travel{.002};
  double ball_pause_k{300.}, ball_pause_d{35.};
  double ball_position_filter{.35}, ball_velocity_filter{.25};
  double ready_speed{0.015}, ready_angular_speed{0.12}, ready_timeout{90.};
  double approach_speed{0.004}, approach_distance{0.020}, approach_timeout{8.};
  double joystick_approach_final_hold_s{2.};
  double ball_approach_force_limit{15.};
  double approach_contact_confirm_s{0.25};
  double approach_contact_speed{0.002}, approach_contact_travel_m{0.001};
  double retract_distance{0.015}, preload{3.}, preload_time{0.55};
  double ball_contact_settle_s{.30}, ball_contact_loss_grace_s{.20},
      ball_interact_dropout_grace_s{.04};
  double ball_goal_tolerance_m{.005}, ball_goal_dwell_s{.10};
  double joystick_approach_below_tip_m{.010};
  double tool_weight_compensation_mass_kg{0.};
  double hold_k{600.}, hold_d{35.}, ready_k{600.}, ready_d{35.};
  double hold_kr{30.}, hold_dr{3.};
  double ready_kr{90.}, ready_dr{5.}, ready_moment_limit{4.};
  double approach_kr{90.}, approach_dr{5.}, approach_moment_limit{4.};
  double coupling_k{700.}, coupling_d{30.}, force_xy_limit{3.};
  double normal_k{800.}, normal_d{60.}, normal_force_limit{7.};
  double orientation_k{3.}, orientation_d{1.}, moment_limit{1.};
  double null_k{2.}, null_d{1.}, joint_margin{0.06};
  double joint_speed_limit{0.6}, point_speed_limit{0.10}, angular_speed_limit{0.5};
  double sensor_timeout{0.30}, heartbeat_timeout{0.30}, max_period{0.010},
         idle_max_period{0.020};
  double active_radius{0.045};
  double joystick_length{0.092}, joystick_angle{0.20943951023931953}, joystick_duration{9.1};
  double joystick_return_zero_start_s{0.};  // Contact-relative time; zero disables the final phase.
  JoystickAxis joystick_axis{JoystickAxis::X};
  // InwardArc uses the guide/link settings. Kinematic contact also uses pivot/shaft feedback.
  JoystickMechanism joystick_mechanism{JoystickMechanism::Legacy};
  bool joystick_alignment_enabled{true};  // Four-link orientation guide in inward_arc contact stages.
  bool joystick_task_spring_uses_shaft{false};  // Shaft-angle error in XY; palm contact-height error in Z.
  bool joystick_update_eit_attachment{true};
  double joystick_attachment_filter_tau_s{.080};  // Time constant of each of two low-pass stages.
  double joystick_slip_contact_mismatch_m{.020};
  double joystick_slip_confirm_s{.06};
  double joystick_inward_arc_m{.010};
  V3 joystick_pivot_m{.45, 0., .035};
  double joystick_alignment_half_span_m{.080}, joystick_slider_radius_m{.036};
  double joystick_alignment_spring_k{234.375}, joystick_alignment_spring_d{78.125};
  double joystick_task_spring_k{700.}, joystick_task_spring_d{30.};
  double joystick_task_spring_force_limit{15.}, joystick_alignment_spring_force_limit{5.};
  double joystick_feedback_timeout{.20};
  double joystick_release_drop{0.17453292519943295};
  double joystick_release_upright{0.08726646259971647};
  double joystick_release_min_command{0.17453292519943295};
  double joystick_release_window_s{.10};
  double angle_speed{0.28}, angle_acceleration{2.0};
  BallAxis ball_axis{BallAxis::X};
  double initial_ball_x{0.45}, initial_ball_y{-0.10}, roll_distance{0.020}, ball_duration{8.};
  // Legacy horizontal-carriage settings retained for config compatibility; unused by direct ball VMC.
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
  double acquired_steady_s{-1.}, angle{0.}, rate{0.}, angle_y{0.}, rate_y{0.};
  bool valid{false}, clipped{false};
};
struct Output {
  Mode mode{Mode::Hold};
  Fault fault{Fault::None};
  V7 torque{V7::Zero()};
  V3 pad_position{V3::Zero()}, point{V3::Zero()}, reference{V3::Zero()}, attachment{V3::Zero()};
  V3 pad_normal{V3::Zero()}, desired_pad_normal{V3::Zero()};
  V3 coupling_force{V3::Zero()};
  V3 requested_force{V3::Zero()}, requested_moment{V3::Zero()};
  V3 tool_weight_compensation_force{V3::Zero()};
  double progress{0.}, progress_velocity{0.}, goal{0.}, angle_command{0.},
      angle_command_y{0.}, trial_time{0.};
  bool use_eit{true}, finished{false}, retract_incomplete{false}, trial_contact_released{false},
      joystick_slipped{false};
  ContactSource contact_source{ContactSource::Eit};
  BallAxis ball_axis{BallAxis::X};
  bool ball_contact_ready{false}, ball_contact_paused{false};
  bool ball_goal_reached{false};
  double ball_contact_loss_duration_s{0.}, ball_loaded_contact_duration_s{0.};
  double ball_normal_position{0.}, ball_normal_velocity{0.}, ball_normal_force{0.};
  double ball_normal_origin{0.}, ball_estimated_velocity{0.};
  double ball_drive_reference{0.};
  bool ball_normal_brake{true}, ball_normal_at_stop{false}, ball_recover_contact{true};
  double approach_distance_limit_m{0.};
  V3 joystick_tip_position{V3::Zero()};
  JoystickAxis joystick_axis{JoystickAxis::X};
  JoystickMechanism joystick_mechanism{JoystickMechanism::Legacy};
  double inward_arc_m{0.}, shaft_angle{0.}, shaft_rate{0.}, shaft_angle_y{0.}, shaft_rate_y{0.},
      shaft_age_s{0.}, force_scale{1.};
  double task_spring_scale{1.}, alignment_min_spring_scale{1.};
  bool shaft_fresh{false}, shaft_clipped{false}, slider_at_stop{false};
  V3 alignment_force{V3::Zero()}, alignment_centre{V3::Zero()}, alignment_target{V3::Zero()};
  Eigen::Vector2d slider{Eigen::Vector2d::Zero()};
  uint64_t sensor_sequence{0};
  bool sensor_contact{false}, sensor_contact_candidate{false}, sensor_fresh{false}, sensor_measurement_valid{false},
      sensor_position_valid{false}, sensor_physical_model{false};
  double sensor_age_s{0.}, sensor_signal_strength{0.}, sensor_contact_threshold{0.};
  double sensor_pattern_distance{0.}, sensor_pattern_support_limit{0.},
      sensor_predicted_radius_mm{0.};
  V3 sensor_contact_pad{V3::Zero()}, sensor_contact_base{V3::Zero()};
};
Eigen::Matrix3d skew(const V3& x);
J67 point_jacobian(const J67& ee_jacobian, const V3& world_offset);
V3 orientation_error(const Eigen::Matrix3d& actual, const Eigen::Matrix3d& desired);
class Core {
 public:
  explicit Core(Config config);
  void activate(const Sample& s);
  bool set_condition(bool use_eit);
  bool set_condition(ContactSource source);
  bool set_joystick_mechanism(JoystickMechanism mechanism, double inward_arc_m);
  bool set_joystick_axis(JoystickAxis axis);
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
  ContactSource contact_source_{ContactSource::Eit};
  V7 posture_{V7::Zero()};
  V3 hold_{V3::Zero()}, reference_{V3::Zero()}, attachment_{V3::Zero()}, h0_{V3::Zero()};
  V3 joystick_contact_start_point_{V3::Zero()}, joystick_contact_start_axis_{V3::UnitZ()};
  V3 joystick_attachment_stage_{V3::Zero()}, joystick_attachment_target_{V3::Zero()},
      joystick_slip_attachment_{V3::Zero()};
  Eigen::Matrix3d hold_rotation_{Eigen::Matrix3d::Identity()};
  V3 phase_start_position_{V3::Zero()}, initial_face_{V3::Zero()};
  Eigen::Quaterniond phase_start_rotation_{Eigen::Quaterniond::Identity()};
  double phase_time_{0.}, trial_time_{0.}, settle_time_{0.}, ready_duration_{1.},
      approach_progress_m_{0.}, approach_confirm_time_{0.},
      approach_contact_start_progress_m_{0.},
      approach_distance_limit_m_{0.},
      approach_final_hold_time_s_{0.},
      unloaded_retract_time_{0.},
      joystick_slip_duration_s_{0.},
      normal_anchor_{0.};
  double ball_contact_loss_duration_s_{0.}, ball_loaded_contact_duration_s_{0.},
      ball_drive_dropout_duration_s_{0.};
  double ball_goal_hold_s_{0.};
  int ball_loaded_contact_frames_{0};
  int ball_goal_hold_frames_{0};
  bool ball_contact_ready_{false}, ball_contact_paused_{false};
  bool ball_goal_reached_{false};
  double ball_initial_preload_{0.}, approach_dropout_time_s_{0.};
  double ball_normal_z_{0.}, ball_normal_v_{0.}, ball_normal_origin_{0.};
  double ball_observed_v_{0.}, ball_observation_time_{-1.}, ball_last_force_{0.},
      ball_reference_progress_m_{0.}, ball_reference_advance_m_{0.};
  double ball_drive_start_{0.};
  V3 ball_pause_point_{V3::Zero()};
  bool ball_started_{false}, ball_previous_contact_{true};
  bool approach_seen_contact_{false};
  int approach_confirm_frames_{0};
  double z_{0.}, dz_{0.}, y_{0.}, dy_{0.}, z0_{0.}, observed_{0.}, goal_{0.};
  Eigen::Vector2d slider_{Eigen::Vector2d::Zero()};
  int ready_stage_{0};
  uint64_t last_sequence_{0};
  uint64_t release_shaft_sequence_{0};
  double release_peak_tilt_{0.}, release_peak_time_{-1.};
  bool trial_contact_released_{false};
  bool joystick_slipped_{false};
  bool activated_{false}, retract_incomplete_{false};
  void fail(Fault fault);
  V3 face(const Sample& s) const;
  Eigen::Matrix3d pad_rotation(const Sample& s) const;
  void begin_ready(const Sample& s);
  void begin_retract(const Sample& s);
  bool inside(const V3& p) const;
};
const char* mechanism_name(JoystickMechanism mechanism);
const char* contact_source_name(ContactSource source);
const char* mode_name(Mode mode);
const char* fault_name(Fault fault);
}  // namespace franka_eit_vmc
