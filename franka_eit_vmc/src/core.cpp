#include "franka_eit_vmc/core.hpp"
#include "franka_eit_vmc/joystick_mechanism.hpp"
#include "franka_eit_vmc/ball_mechanism.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
namespace franka_eit_vmc {
namespace {
constexpr double kReadyPositionTolerance = .005;
constexpr double kTrialStartPositionTolerance = .005;
double clip(double x, double low, double high) { return std::clamp(x, low, high); }
double smooth(double u) {
  u = clip(u, 0., 1.);
  return u * u * u * (10. + u * (-15. + 6. * u));
}
double smooth_d(double u) {
  u = clip(u, 0., 1.);
  return 30. * u * u * (1. - u) * (1. - u);
}
V3 limit_norm(V3 v, double max) {
  const double n = v.norm();
  if (n > max) v *= max / n;
  return v;
}
bool rotation(const Eigen::Matrix3d& r) {
  return r.allFinite() && (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
         std::abs(r.determinant() - 1.) < 1e-6;
}
}  // namespace
void Config::validate() const {
  if (!std::isfinite(joystick_approach_final_hold_s) || joystick_approach_final_hold_s < 0.)
    throw std::invalid_argument("joystick_approach_final_hold_s must be finite and nonnegative");
  if (!std::isfinite(joystick_return_zero_start_s) || joystick_return_zero_start_s < 0. ||
      (task == Task::Joystick && joystick_return_zero_start_s > 0. &&
       (joystick_return_zero_start_s <= std::max(4.5, preload_time) ||
        joystick_return_zero_start_s >= joystick_duration)))
    throw std::invalid_argument("joystick_return_zero_start_s must be zero (disabled), or after reversal/preload and before joystick_duration");
  if (!std::isfinite(joystick_attachment_filter_tau_s) || joystick_attachment_filter_tau_s <= 0.)
    throw std::invalid_argument("joystick_attachment_filter_tau_s must be finite and positive");
  if (!std::isfinite(tool_weight_compensation_mass_kg) || tool_weight_compensation_mass_kg < 0.)
    throw std::invalid_argument("tool_weight_compensation_mass_kg must be finite and nonnegative");
  if (!ee_to_pad.allFinite() || ee_to_pad.norm() > .40 || !rotation(ee_R_pad) ||
      !rotation(ready_pad_rotation))
    throw std::invalid_argument("Invalid measured EE-to-pad transform");
  if (!ready_pad_position.allFinite() || !workspace_min.allFinite() || !workspace_max.allFinite() ||
      !(workspace_min.array() < workspace_max.array()).all() || workspace_min.x() <= 0. ||
      workspace_min.z() <= 0.)
    throw std::invalid_argument("Workspace must have positive X and Z bounds");
  if ((ready_pad_position.array() < workspace_min.array()).any() ||
      (ready_pad_position.array() > workspace_max.array()).any())
    throw std::invalid_argument("Ready pad position is outside workspace");
  if (ready_pad_rotation.col(2).dot(-V3::UnitZ()) < .99)
    throw std::invalid_argument("Ready pad normal must point down");
  if (!q_min.allFinite() || !q_max.allFinite() || !(q_min.array() < q_max.array()).all())
    throw std::invalid_argument("Invalid joint/torque limits");
  if (use_saved_posture && (!posture_reference.allFinite() ||
                            (posture_reference.array() <= q_min.array() + joint_margin).any() ||
                            (posture_reference.array() >= q_max.array() - joint_margin).any()))
    throw std::invalid_argument("Saved reference posture is outside joint limits");
  if (ball_position_filter <= 0. || ball_position_filter > 1. ||
      ball_velocity_filter <= 0. || ball_velocity_filter > 1. ||
      !std::isfinite(ball_position_filter) || !std::isfinite(ball_velocity_filter))
    throw std::invalid_argument("Ball observation filters must be in (0,1]");
  if (!std::isfinite(ball_baseline_motion_ratio) ||
      ball_baseline_motion_ratio <= 0. || ball_baseline_motion_ratio > 2.)
    throw std::invalid_argument("Ball baseline motion ratio must be in (0,2]");
  if (!std::isfinite(ball_interact_dropout_grace_s) ||
      ball_interact_dropout_grace_s < 0. || ball_interact_dropout_grace_s > .20)
    throw std::invalid_argument("Ball interaction dropout grace must be in [0,0.20] s");
  if (!std::isfinite(ball_reference_speed_m_s) ||
      ball_reference_speed_m_s <= 0. || ball_reference_speed_m_s > .10 ||
      !std::isfinite(ball_reference_advance_speed_m_s) ||
      ball_reference_advance_speed_m_s < 0. || ball_reference_advance_speed_m_s > .05 ||
      !std::isfinite(ball_reference_advance_limit_m) ||
      ball_reference_advance_limit_m < 0. || ball_reference_advance_limit_m > .10 ||
      !std::isfinite(ball_reference_advance_start_error_m) ||
      ball_reference_advance_start_error_m <= 0. || ball_reference_advance_start_error_m > .10)
    throw std::invalid_argument("Ball reference advance parameters are invalid");
  const double positive[] = {ball_goal_tolerance_m, ball_goal_dwell_s,
                             ball_goal_d, ball_force_slew, ball_pause_force_slew,
                             ball_goal_ramp_s,
                             ball_normal_mass, ball_normal_damping, ball_normal_speed,
                             ball_normal_travel, ball_normal_up_travel, ball_pause_k, ball_pause_d,
                             ready_speed,
                             ready_angular_speed,
                             ready_timeout,
                             approach_speed,
                             approach_distance,
                             approach_timeout,
                             ball_approach_force_limit,
                             approach_contact_confirm_s,
                             approach_contact_speed,
                             approach_contact_travel_m,
                             ball_contact_settle_s,
                             ball_contact_loss_grace_s,
                             joystick_approach_below_tip_m,
                             retract_distance,
                             preload,
                             preload_time,
                             hold_k,
                             hold_d,
                             ready_k,
                             ready_d,
                             hold_kr,
                             hold_dr,
                             ready_kr,
                             ready_dr,
                             ready_moment_limit,
                             approach_kr,
                             approach_dr,
                             approach_moment_limit,
                             coupling_k,
                             coupling_d,
                             force_xy_limit,
                             normal_k,
                             normal_d,
                             normal_force_limit,
                             orientation_k,
                             orientation_d,
                             moment_limit,
                             null_k,
                             null_d,
                             joint_margin,
                             joint_speed_limit,
                             point_speed_limit,
                             angular_speed_limit,
                             sensor_timeout,
                             heartbeat_timeout,
                             max_period,
                             idle_max_period,
                             active_radius,
                             joystick_length,
                             joystick_angle,
                             joystick_duration,
                             joystick_task_spring_k,
                             joystick_task_spring_d,
                             joystick_task_spring_force_limit,
                             joystick_alignment_spring_force_limit,
                             joystick_release_drop,
                             joystick_release_upright,
                             joystick_release_min_command,
                             joystick_release_window_s,
                             angle_speed,
                             angle_acceleration,
                             ball_duration,
                             virtual_mass,
                             virtual_damping,
                             goal_k,
                             goal_force_limit,
                             progress_speed};
  for (double x : positive)
    if (!std::isfinite(x) || x <= 0.)
      throw std::invalid_argument(
          "Control gains, limits and durations must be finite and positive");
  if (!std::isfinite(initial_ball_x) || !std::isfinite(initial_ball_y) ||
      !std::isfinite(roll_distance) || roll_distance == 0. || active_radius > .045 ||
      max_period > .02 || idle_max_period > .02 || idle_max_period < max_period ||
      preload > normal_force_limit || ready_speed >= point_speed_limit ||
      approach_speed >= point_speed_limit || sensor_timeout > 1. ||
      approach_contact_confirm_s >= approach_timeout ||
      approach_contact_speed > approach_speed ||
      (task == Task::Joystick && approach_contact_travel_m >= approach_distance) ||
      joystick_duration <= preload_time || ball_duration <= preload_time + ball_contact_settle_s ||
      ball_contact_loss_grace_s > 1.)
    throw std::invalid_argument("Invalid task geometry or timing bounds");
  if (ball_axis != BallAxis::X && ball_axis != BallAxis::Y)
    throw std::invalid_argument("ball_axis must be x or y");
  if (task == Task::Ball) {
    if (ball_approach_force_limit > 30.)
      throw std::invalid_argument("Ball approach force limit must be at most 30 N");
    if (ball_goal_tolerance_m >= std::abs(roll_distance) ||
        ball_goal_dwell_s >= ball_duration - preload_time - ball_contact_settle_s)
      throw std::invalid_argument("Ball goal tolerance or dwell exceeds the trial bounds");
    const int axis = ball_axis == BallAxis::Y ? 1 : 0;
    const double initial = axis == 1 ? initial_ball_y : initial_ball_x;
    const double goal = initial + roll_distance;
    const double palm_goal = ready_pad_position[axis] +
                             roll_distance / ball_baseline_motion_ratio;
    const double drive_limit = goal + std::copysign(ball_reference_advance_limit_m,roll_distance);
    if (initial < workspace_min[axis] || initial > workspace_max[axis] ||
        goal < workspace_min[axis] || goal > workspace_max[axis] ||
        palm_goal < workspace_min[axis] || palm_goal > workspace_max[axis] ||
        drive_limit < workspace_min[axis] || drive_limit > workspace_max[axis])
      throw std::invalid_argument("Ball start/goal or nominal palm stroke is outside the workspace");
  }
  if (joystick_mechanism == JoystickMechanism::InwardArc) {
    if (task != Task::Joystick)
      throw std::invalid_argument("inward_arc requires the joystick task");
    if (!std::isfinite(joystick_inward_arc_m) || joystick_inward_arc_m < 0. ||
        joystick_inward_arc_m >= joystick_length)
      throw std::invalid_argument("joystick_inward_arc_m must be nonnegative and below joystick_length");
    if (!joystick_pivot_m.allFinite())
      throw std::invalid_argument("joystick_pivot_m must be finite");
    if (!std::isfinite(joystick_alignment_half_span_m) || joystick_alignment_half_span_m <= 0.)
      throw std::invalid_argument("joystick_alignment_half_span_m must be positive");
    // The virtual palm slider is distinct from the EIT pad's active contact radius.
    if (!std::isfinite(joystick_slider_radius_m) || joystick_slider_radius_m <= 0.)
      throw std::invalid_argument("joystick_slider_radius_m must be positive");
    if (!std::isfinite(joystick_alignment_spring_k) || joystick_alignment_spring_k <= 0.)
      throw std::invalid_argument("joystick_alignment_spring_k must be positive");
    if (!std::isfinite(joystick_alignment_spring_d) || joystick_alignment_spring_d <= 0.)
      throw std::invalid_argument("joystick_alignment_spring_d must be positive");
    if (!std::isfinite(joystick_slip_contact_mismatch_m) ||
        joystick_slip_contact_mismatch_m <= 0. ||
        joystick_slip_contact_mismatch_m > .10 ||
        !std::isfinite(joystick_slip_confirm_s) ||
        joystick_slip_confirm_s <= 0. || joystick_slip_confirm_s > .5)
      throw std::invalid_argument("Invalid joystick slip detection thresholds");
    if (!std::isfinite(joystick_feedback_timeout) || joystick_feedback_timeout <= 0. ||
        joystick_feedback_timeout > .5)
      throw std::invalid_argument("joystick_feedback_timeout must be in (0, 0.5] seconds");
    if (joystick_release_upright + joystick_release_drop >= 1.5707963267948966 ||
        joystick_release_min_command >= 1.5707963267948966 || joystick_release_window_s > .5)
      throw std::invalid_argument("Invalid joystick release angle thresholds or window");
  }

}
Eigen::Matrix3d skew(const V3& x) {
  Eigen::Matrix3d m;
  m << 0., -x.z(), x.y(), x.z(), 0., -x.x(), -x.y(), x.x(), 0.;
  return m;
}
J67 point_jacobian(const J67& j, const V3& offset) {
  J67 out = j;
  out.topRows<3>() = j.topRows<3>() - skew(offset) * j.bottomRows<3>();
  return out;
}
V3 orientation_error(const Eigen::Matrix3d& actual, const Eigen::Matrix3d& desired) {
  Eigen::Quaterniond q(desired * actual.transpose());
  if (q.w() < 0.) q.coeffs() *= -1.;
  Eigen::AngleAxisd a(q);
  return a.angle() * a.axis();
}
Core::Core(Config config) : c_(std::move(config)) { c_.validate(); }
V3 Core::face(const Sample& s) const { return s.ee_position + s.ee_rotation * c_.ee_to_pad; }
Eigen::Matrix3d Core::pad_rotation(const Sample& s) const { return s.ee_rotation * c_.ee_R_pad; }
bool Core::inside(const V3& p) const {
  return p.allFinite() && (p.array() >= c_.workspace_min.array()).all() &&
         (p.array() <= c_.workspace_max.array()).all();
}
void Core::fail(Fault f) {
  if (fault_ == Fault::None) fault_ = f;
  mode_ = Mode::Fault;
}
void Core::activate(const Sample& s) {
  if (!s.q.allFinite() || !s.dq.allFinite() || !s.ee_position.allFinite() ||
      !rotation(s.ee_rotation))
    throw std::invalid_argument("Invalid activation state");
  posture_ = c_.use_saved_posture ? c_.posture_reference : s.q;
  trial_contact_released_ = false;
  joystick_slipped_ = false;
  hold_ = face(s);
  reference_ = hold_;
  hold_rotation_ = pad_rotation(s);
  mode_ = Mode::Hold;
  fault_ = Fault::None;
  phase_time_ = 0.;
  approach_progress_m_ = 0.;
  approach_confirm_time_ = 0.;
  approach_confirm_frames_ = 0;
  approach_contact_start_progress_m_ = 0.;
  approach_distance_limit_m_ = c_.approach_distance;
  ball_contact_loss_duration_s_ = ball_loaded_contact_duration_s_ =
      ball_drive_dropout_duration_s_ = 0.;
  ball_loaded_contact_frames_ = 0;
  ball_contact_ready_ = ball_contact_paused_ = false;
  ball_goal_hold_s_ = 0.; ball_goal_hold_frames_ = 0; ball_goal_reached_ = false;
  ball_reference_progress_m_ = ball_reference_advance_m_ = 0.;
  approach_dropout_time_s_ = 0.;
  approach_seen_contact_ = false;
  joystick_slip_duration_s_ = 0.;
  settle_time_ = 0.;
  unloaded_retract_time_ = 0.;
  retract_incomplete_ = false;
  activated_ = true;
  last_sequence_ = 0;
}
bool Core::set_condition(bool use_eit) {
  return set_condition(use_eit ? ContactSource::Eit : ContactSource::Baseline);
}
bool Core::set_condition(ContactSource source) {
  if (mode_ != Mode::Hold && mode_ != Mode::Done) return false;
  if (source != ContactSource::Baseline && source != ContactSource::Eit &&
      source != ContactSource::Kinematic) return false;
  if (source == ContactSource::Kinematic &&
      (c_.task != Task::Joystick || !c_.joystick_pivot_m.allFinite())) return false;
  contact_source_ = source;
  return true;
}
bool Core::set_joystick_mechanism(JoystickMechanism mechanism, double depth) {
  if (c_.task != Task::Joystick || (mode_ != Mode::Hold && mode_ != Mode::Done) ||
      !std::isfinite(depth) || depth < 0. || depth >= c_.joystick_length) return false;
  c_.joystick_mechanism = mechanism;
  c_.joystick_inward_arc_m = depth;
  return true;
}
bool Core::set_joystick_axis(JoystickAxis axis) {
  if (c_.task != Task::Joystick || (mode_ != Mode::Hold && mode_ != Mode::Done) ||
      (axis != JoystickAxis::X && axis != JoystickAxis::Y)) return false;
  c_.joystick_axis = axis;
  return true;
}
void Core::begin_ready(const Sample& s) {
  mode_ = Mode::Ready;
  ready_stage_ = 0;
  phase_time_ = 0.;
  settle_time_ = 0.;
  retract_incomplete_ = false;
  phase_start_position_ = face(s);
  phase_start_rotation_ = Eigen::Quaterniond(pad_rotation(s));
}
void Core::begin_retract(const Sample& s) {
  mode_ = Mode::Retract;
  phase_time_ = 0.;
  settle_time_ = 0.;
  unloaded_retract_time_ = 0.;
  retract_incomplete_ = false;
  phase_start_position_ = face(s);
  hold_ = phase_start_position_ + V3(0., 0., c_.retract_distance);
  hold_rotation_ = pad_rotation(s);
}
Output Core::step(const Sample& s, const Contact& contact, double now, double dt, bool heartbeat,
                  Request request, const ShaftFeedback& shaft) {
  Output out;
  const double safe_dt = std::isfinite(dt) && dt > 0. ? std::min(dt, c_.max_period) : .001;
  const double period_limit =
      (mode_ == Mode::Hold || mode_ == Mode::Done) ? c_.idle_max_period : c_.max_period;
  if (!activated_) throw std::logic_error("Activate the core before stepping");
  if (!std::isfinite(dt) || dt <= 0. || dt > period_limit) fail(Fault::Period);
  if (!std::isfinite(now) || !s.q.allFinite() || !s.dq.allFinite() || !s.coriolis.allFinite() ||
      !s.previous_torque.allFinite() || !s.ee_position.allFinite() || !rotation(s.ee_rotation) ||
      !s.jacobian.allFinite())
    fail(Fault::Nonfinite);
  const V3 pface = face(s);
  const auto Rp = pad_rotation(s);
  const J67 jf = point_jacobian(s.jacobian, s.ee_rotation * c_.ee_to_pad);
  const V3 omega = s.jacobian.bottomRows<3>() * s.dq;
  const V3 vface = jf.topRows<3>() * s.dq;
  if (!inside(pface) || !inside(s.ee_position)) fail(Fault::Workspace);
  if ((s.q.array() < c_.q_min.array() + c_.joint_margin).any() ||
      (s.q.array() > c_.q_max.array() - c_.joint_margin).any())
    fail(Fault::JointLimit);
  if (s.dq.cwiseAbs().maxCoeff() > c_.joint_speed_limit) fail(Fault::JointSpeed);
  if (vface.norm() > c_.point_speed_limit || omega.norm() > c_.angular_speed_limit)
    fail(Fault::CartesianSpeed);
  const bool in_trial =
      mode_ == Mode::Approach || mode_ == Mode::Preload || mode_ == Mode::Interact;
  const double sensor_age = now - contact.acquired_steady_s;
  const bool sensor_fresh = contact.physical_model && contact.sequence > 0 &&
                            std::isfinite(sensor_age) && sensor_age >= -.01 &&
                            sensor_age <= c_.sensor_timeout;
  const bool new_frame = contact.sequence != last_sequence_;
  const bool inward = c_.task == Task::Joystick &&
                      c_.joystick_mechanism == JoystickMechanism::InwardArc;
  const bool use_eit = contact_source_ == ContactSource::Eit;
  const bool kinematic = contact_source_ == ContactSource::Kinematic;
  const int ball_axis_index = c_.ball_axis == BallAxis::Y ? 1 : 0;
  const V3 ball_direction = ball_axis_index == 1 ? V3::UnitY() : V3::UnitX();
  const double initial_ball_coordinate = ball_axis_index == 1 ? c_.initial_ball_y : c_.initial_ball_x;
  const double shaft_age = now - shaft.acquired_steady_s;
  const bool shaft_fresh = shaft.sequence > 0 && shaft.valid &&
      std::isfinite(shaft.angle) && std::isfinite(shaft.rate) &&
      std::isfinite(shaft.angle_y) && std::isfinite(shaft.rate_y) &&
      std::isfinite(shaft_age) && shaft_age >= -.01 && shaft_age <= c_.joystick_feedback_timeout;
  if ((inward || kinematic) && in_trial && !shaft_fresh) fail(Fault::JoystickFeedback);
  if (in_trial && !sensor_fresh) fail(Fault::SensorStale);
  if (!heartbeat && mode_ != Mode::Hold && mode_ != Mode::Done && mode_ != Mode::Fault &&
      mode_ != Mode::Stopping)
    fail(Fault::Heartbeat);
  if (request == Request::Stop && mode_ != Mode::Fault) mode_ = Mode::Stopping;
  if (request == Request::Ready && (mode_ == Mode::Hold || mode_ == Mode::Done) && heartbeat)
    begin_ready(s);
  if (request == Request::Start && (mode_ == Mode::Hold || mode_ == Mode::Done) && heartbeat) {
    trial_contact_released_ = false;
    joystick_slipped_ = false;
    if ((inward || kinematic) && !shaft_fresh)
      fail(Fault::JoystickFeedback);
    else if (!sensor_fresh || contact.contact)
      fail(Fault::SensorStale);
    else if ((pface - c_.ready_pad_position).norm() > kTrialStartPositionTolerance ||
             orientation_error(Rp, c_.ready_pad_rotation).norm() > .05)
      fail(Fault::Tracking);
    else {
      mode_ = Mode::Approach;
      phase_time_ = 0.;
      approach_progress_m_ = 0.;
      approach_final_hold_time_s_ = 0.;
      approach_confirm_time_ = 0.;
      approach_confirm_frames_ = 0;
      approach_contact_start_progress_m_ = 0.;
      if (c_.task == Task::Ball) {
        // Search to the configured lower workspace boundary, independent of
        // the height from which Ready began. Leave a small numerical margin.
        approach_distance_limit_m_ = std::max(0.,
            pface.z() - c_.workspace_min.z() - .0001);
      } else {
        approach_distance_limit_m_ = c_.approach_distance;
        const double tip_z = c_.joystick_pivot_m.z() + c_.joystick_length *
            (shaft_fresh ? shaft_geometry(shaft.angle, shaft.angle_y, shaft.rate, shaft.rate_y).axis.z() : 1.);
        approach_distance_limit_m_ = std::min(c_.approach_distance,
            std::max(0., pface.z() - tip_z + c_.joystick_approach_below_tip_m));
      }
      ball_contact_loss_duration_s_ = ball_loaded_contact_duration_s_ =
          ball_drive_dropout_duration_s_ = 0.;
      ball_loaded_contact_frames_ = 0;
      ball_contact_ready_ = ball_contact_paused_ = false;
      ball_goal_hold_s_ = 0.; ball_goal_hold_frames_ = 0; ball_goal_reached_ = false;
      ball_reference_progress_m_ = ball_reference_advance_m_ = 0.;
      approach_dropout_time_s_ = 0.;
      approach_seen_contact_ = false;
      hold_ = pface;
      hold_rotation_ = Rp;
      last_sequence_ = contact.sequence;
    }
  }
  V3 force = V3::Zero(), moment = V3::Zero(), fc = V3::Zero(), point = pface, ref = hold_,
     vref = V3::Zero();
  J67 jc = jf;
  V7 alignment_torque_correction = V7::Zero();
  V3 alignment_force = V3::Zero(), alignment_centre = pface, alignment_target = pface;
  Eigen::Vector2d slider_used = slider_;
  bool slider_at_stop = false;
  double task_spring_scale = 1., alignment_min_spring_scale = 1.;
  Eigen::Matrix3d desired_rotation = hold_rotation_;
  phase_time_ += safe_dt;
  if (mode_ == Mode::Ready) {
    // Lift to the ready height, translate and orient above the task, then descend.
    V3 target = c_.ready_pad_position;
    Eigen::Quaterniond target_rotation(c_.ready_pad_rotation);
    if (ready_stage_ == 0) {
      target = phase_start_position_;
      target.z() = std::max(phase_start_position_.z(), c_.ready_pad_position.z());
      target_rotation = phase_start_rotation_;
    }
    if (ready_stage_ == 1)
      target.z() = std::max(phase_start_position_.z(), c_.ready_pad_position.z());
    ready_duration_ = std::max(
        {1., 1.875 * (target - phase_start_position_).norm() / c_.ready_speed,
         1.875 * phase_start_rotation_.angularDistance(target_rotation) / c_.ready_angular_speed});
    const double u = phase_time_ / ready_duration_;
    ref = phase_start_position_ + smooth(u) * (target - phase_start_position_);
    vref = smooth_d(u) / ready_duration_ * (target - phase_start_position_);
    desired_rotation = phase_start_rotation_.slerp(smooth(u), target_rotation).toRotationMatrix();
    if (phase_time_ >= ready_duration_ &&
        (pface - target).norm() <= kReadyPositionTolerance &&
        orientation_error(Rp, desired_rotation).norm() < .04 && s.dq.norm() < .08)
      settle_time_ += safe_dt;
    else
      settle_time_ = 0.;
    if (settle_time_ > .4) {
      if (ready_stage_ == 2) {
        mode_ = Mode::Hold;
        hold_ = target;
        hold_rotation_ = desired_rotation;
      } else {
        ++ready_stage_;
        phase_start_position_ = pface;
        phase_start_rotation_ = Eigen::Quaterniond(Rp);
        phase_time_ = 0.;
        settle_time_ = 0.;
      }
    }
    if (phase_time_ > ready_duration_ + c_.ready_timeout) fail(Fault::ReadyTimeout);
  }
  if (mode_ == Mode::Approach) {
    const bool need_position =
        c_.task == Task::Joystick ? use_eit : use_eit || c_.update_ball_attachment || c_.ball_initial_attachment;
    const bool position_ok =
        !need_position || (contact.position_valid && std::isfinite(contact.x) &&
                           std::isfinite(contact.y) &&
                           std::hypot(contact.x, contact.y) <= c_.active_radius);
    const bool reported_contact = contact.contact && contact.contact_candidate &&
                                  contact.measurement_valid && sensor_fresh;
    // A provisional EIT report is followed by a bounded, slower descent.
    // Preload needs sustained EIT contact through the commanded verification move;
    // a contacting pad may resist and move less than the reference.
    if (reported_contact) {
      approach_seen_contact_ = true;
      approach_dropout_time_s_ = 0.;
      if (position_ok) {
        if (approach_confirm_frames_ == 0) {
          approach_contact_start_progress_m_ = approach_progress_m_;
        }
        approach_confirm_time_ += safe_dt;
        if (new_frame) ++approach_confirm_frames_;
        const double confirm_end = std::min(approach_distance_limit_m_,
            approach_contact_start_progress_m_ + c_.approach_contact_travel_m);
        if (approach_progress_m_ < confirm_end) {
          approach_progress_m_ = std::min(confirm_end,
              approach_progress_m_ + c_.approach_contact_speed * safe_dt);
          vref = V3(0., 0., -c_.approach_contact_speed);
        }
      } else {
        approach_confirm_time_ = 0.;
        approach_confirm_frames_ = 0;
      }
    } else {
      approach_confirm_time_ = 0.;
      approach_confirm_frames_ = 0;
      // A disappearing provisional ball contact must not trigger a fast plunge.
      const bool cautious = c_.task == Task::Ball && approach_seen_contact_;
      approach_dropout_time_s_ += safe_dt;
      const double speed = cautious ? c_.approach_contact_speed : c_.approach_speed;
      if (!cautious || approach_dropout_time_s_ > c_.ball_contact_loss_grace_s) {
        approach_progress_m_ = std::min(approach_distance_limit_m_,
            approach_progress_m_ + speed * safe_dt);
        vref = V3(0., 0., -speed);
      }
    }
    ref = hold_ - V3(0., 0., approach_progress_m_);
    const bool at_approach_end = approach_progress_m_ >= approach_distance_limit_m_ - 1e-9;
    const bool final_hold_enabled = c_.task == Task::Joystick &&
        c_.joystick_approach_final_hold_s > 0.;
    if (at_approach_end && final_hold_enabled) {
      approach_final_hold_time_s_ += safe_dt;
      // The reference has stopped. Do not retain a fictitious downward
      // reference velocity (and its extra damping force) while waiting.
      vref.setZero();
    } else {
      approach_final_hold_time_s_ = 0.;
    }
    const double confirm_end = std::min(approach_distance_limit_m_,
        approach_contact_start_progress_m_ + c_.approach_contact_travel_m);
    const double available_verification = confirm_end - approach_contact_start_progress_m_;
    if (position_ok && approach_confirm_time_ >= c_.approach_contact_confirm_s &&
        approach_confirm_frames_ >= 2 &&
        (available_verification > 1e-9 || (final_hold_enabled && at_approach_end)) &&
        approach_progress_m_ >= confirm_end - 1e-9) {
      attachment_ = V3::Zero();
      if ((c_.task == Task::Joystick && use_eit) ||
          (c_.task == Task::Ball && (c_.update_ball_attachment || c_.ball_initial_attachment)))
        attachment_ = V3(contact.x, contact.y, 0.);
      if (kinematic) {
        const auto geometry = shaft_geometry(shaft.angle, shaft.angle_y, shaft.rate, shaft.rate_y);
        const V3 tip = c_.joystick_pivot_m + c_.joystick_length * geometry.axis;
        // Capture the geometrical tip projected onto the measured palm plane,
        // then retain the same material point throughout the trial as in EIT.
        attachment_ = Rp.transpose() * (tip - pface);
        attachment_.z() = 0.;
        if (!attachment_.allFinite() || attachment_.head<2>().norm() > c_.active_radius)
          fail(Fault::SensorPosition);
      }
      if (mode_ == Mode::Approach) {
        joystick_attachment_stage_ = joystick_attachment_target_ = attachment_;
        joystick_slip_attachment_ = attachment_;
        point = pface + Rp * attachment_;
        h0_ = point;
        normal_anchor_ = point.z();
        // Match the bounded approach normal command at the start of Preload,
        // then blend its virtual load and force cap to the configured values.
        const double incoming_limit = c_.task == Task::Ball
            ? c_.ball_approach_force_limit : c_.normal_force_limit;
        const double incoming = clip(c_.hold_k * (ref.z() - pface.z()) +
            c_.hold_d * (vref.z() - vface.z()), -incoming_limit, incoming_limit);
        ball_initial_preload_ = clip(-incoming - c_.normal_d * vface.z(),
                                    0., incoming_limit);
        if (c_.task == Task::Ball) {
          // Preserve the incoming bounded approach force; this is initialization,
          // not a measured-load feedback law. Subsequent dynamics match the simulation.
          ball_normal_z_ = std::max(point.z() - ball_initial_preload_ / c_.normal_k,
              std::min(c_.workspace_min.z()+.0001,point.z()));
          ball_normal_v_ = 0.; ball_normal_origin_ = point.z();
          ball_observed_v_ = 0.; ball_observation_time_ = -1.; ball_last_force_ = 0.;
          ball_reference_advance_m_ = 0.;
          ball_pause_point_ = point; ball_started_ = false; ball_previous_contact_ = true;
          ball_drive_dropout_duration_s_ = 0.;
        }
        initial_face_ = pface;
        if (inward) {
          const auto initial_shaft = shaft_geometry(shaft.angle, shaft.angle_y, shaft.rate, shaft.rate_y);
          // The task pivot is contact-defined, while both angle commands remain
          // absolute with respect to the calibrated upright shaft direction.
          h0_ = point - c_.joystick_length * (initial_shaft.axis - V3::UnitZ());
          const V3 tip = c_.joystick_pivot_m + c_.joystick_length * initial_shaft.axis;
          joystick_contact_start_point_ = point;
          joystick_contact_start_axis_ = initial_shaft.axis;
          if (c_.joystick_alignment_enabled) {
            slider_ = (Rp.transpose() * (tip - pface)).head<2>();
            if (slider_.norm() > c_.joystick_slider_radius_m)
              slider_ *= c_.joystick_slider_radius_m / slider_.norm();
          } else {
            slider_.setZero();
          }
        }
        trial_time_ = 0.;
        joystick_slip_duration_s_ = 0.;
        phase_time_ = 0.;
        z_ = c_.task == Task::Ball ? initial_ball_coordinate : inward ? shaft.angle : 0.;
        y_ = inward ? shaft.angle_y : 0.;
        z0_ = z_;
        dz_ = 0.;
        dy_ = 0.;
        observed_ = z_;
        release_peak_tilt_ = std::atan(std::hypot(std::tan(shaft.angle), std::tan(shaft.angle_y)));
        release_peak_time_ = shaft.acquired_steady_s;
        release_shaft_sequence_ = shaft.sequence;
        goal_ = initial_ball_coordinate + c_.roll_distance;
        mode_ = Mode::Preload;
        last_sequence_ = contact.sequence;
      }
    }
    const double approach_deadline = c_.approach_timeout +
        (c_.task == Task::Ball ? approach_distance_limit_m_ / c_.approach_speed : 0.);
    if (mode_ == Mode::Approach && (phase_time_ > approach_deadline ||
        (at_approach_end && (final_hold_enabled
            ? approach_final_hold_time_s_ >= c_.joystick_approach_final_hold_s
            : !reported_contact))))
      fail(Fault::ApproachTimeout);
  }
  if (c_.task == Task::Joystick && use_eit && c_.joystick_update_eit_attachment &&
      (mode_ == Mode::Preload || mode_ == Mode::Interact) && sensor_fresh &&
      contact.measurement_valid && contact.contact && contact.position_valid &&
      std::isfinite(contact.x) && std::isfinite(contact.y) &&
      std::hypot(contact.x, contact.y) <= c_.active_radius) {
    // Fresh frames replace the held observation. Integrate at servo rate so
    // 50 Hz sensor updates do not cause attachment/force steps. Freeze both
    // stages during rejected frames; do not chase the invalid zero position.
    if (new_frame) joystick_attachment_target_ = V3(contact.x, contact.y, 0.);
    const double u = safe_dt / c_.joystick_attachment_filter_tau_s;
    const double e = std::exp(-u);
    const V3 previous_stage = joystick_attachment_stage_;
    joystick_attachment_stage_ = joystick_attachment_target_ +
        (previous_stage - joystick_attachment_target_) * e;
    attachment_ = joystick_attachment_target_ +
        (attachment_ - joystick_attachment_target_ +
         (previous_stage - joystick_attachment_target_) * u) * e;
    // h0_ and normal_anchor_ deliberately stay at engagement: translating
    // the target along with the attachment would cancel the position feedback.
    // The damper continues to use robot material-point velocity, not the
    // derivative of a noisy contact estimate.
  }
  if (inward && shaft_fresh && (mode_ == Mode::Preload || mode_ == Mode::Interact)) {
    const double tilt = std::atan(std::hypot(std::tan(shaft.angle), std::tan(shaft.angle_y)));
    if (shaft.sequence != release_shaft_sequence_) {
      if (shaft.acquired_steady_s - release_peak_time_ > c_.joystick_release_window_s ||
          tilt >= release_peak_tilt_) {
        release_peak_tilt_ = tilt;
        release_peak_time_ = shaft.acquired_steady_s;
      }
      release_shaft_sequence_ = shaft.sequence;
    }
    // A brief electrical dropout alone is insufficient. Catch the combined
    // release/snapback before replacing the alignment target with upright.
    if (sensor_fresh && !contact.contact && tilt <= c_.joystick_release_upright &&
        std::hypot(z_, y_) >= c_.joystick_release_min_command &&
        now - release_peak_time_ >= 0. &&
        now - release_peak_time_ <= c_.joystick_release_window_s &&
        release_peak_tilt_ - tilt >= c_.joystick_release_drop) {
      trial_contact_released_ = true;
      begin_retract(s);
    }
  }
  if (inward && (mode_ == Mode::Preload || mode_ == Mode::Interact)) {
    const auto measured_shaft = shaft_geometry(shaft.angle, shaft.angle_y, shaft.rate, shaft.rate_y);
    const double radius = c_.joystick_length - c_.joystick_inward_arc_m;
    const V3 expected_point = joystick_contact_start_point_ +
        radius * (measured_shaft.axis - joystick_contact_start_axis_);
    // Keep the original material-point check so a migrating contact estimate
    // cannot conceal the earlier palm-runaway failure.
    const V3 actual_point = pface + Rp * joystick_slip_attachment_;
    // With reported contact, a shaft that moves farther than the palm is not
    // evidence that the palm has run past the joystick. Only palm lead along
    // the current commanded displacement indicates that runaway. The sign
    // follows the arc reference, so it does not flip at the + to - reversal
    // until the reference passes its captured contact position.
    const auto commanded_shaft = shaft_geometry(z_, y_, dz_, dy_);
    const V3 commanded_point = inward_arc(h0_, c_.joystick_length,
                                           c_.joystick_inward_arc_m, commanded_shaft).position;
    const int axis = c_.joystick_axis == JoystickAxis::Y ? 1 : 0;
    const double commanded_travel = commanded_point[axis] - joystick_contact_start_point_[axis];
    const double direction = commanded_travel > 1e-6 ? 1. : commanded_travel < -1e-6 ? -1. : 0.;
    const double palm_lead = direction * (actual_point[axis] - expected_point[axis]);
    // A no-contact classifier report, however long, does not prove release.
    // Hold the last valid attachment and keep driving unless shaft snapback
    // provides release evidence above. This separate check only handles a
    // palm running ahead while the sensor still reports contact.
    const bool slip_evidence = shaft_fresh && sensor_fresh && contact.contact &&
        palm_lead > c_.joystick_slip_contact_mismatch_m;
    joystick_slip_duration_s_ = slip_evidence ? joystick_slip_duration_s_ + safe_dt : 0.;
    if (joystick_slip_duration_s_ >= c_.joystick_slip_confirm_s) {
      joystick_slipped_ = true;
      trial_contact_released_ = true;
      begin_retract(s);
    }
  }
  if (mode_ == Mode::Preload || mode_ == Mode::Interact) {
    trial_time_ += safe_dt;
    // Joystick EIT attachments track filtered valid contact; rejected frames
    // retain the last attachment. Baseline/kinematic attachments stay fixed.
    const bool ball_contact_ok = sensor_fresh && contact.measurement_valid && contact.contact;
    const bool ball_position_ok = contact.position_valid && std::isfinite(contact.x) &&
        std::isfinite(contact.y) && std::hypot(contact.x,contact.y) <= c_.active_radius;
    const bool need_ball_position = use_eit || c_.update_ball_attachment || c_.ball_initial_attachment;
    const bool ball_drive_observation_ok = ball_contact_ok && (!need_ball_position || ball_position_ok);
    if (ball_drive_observation_ok) ball_drive_dropout_duration_s_ = 0.;
    else if (sensor_fresh && contact.measurement_valid)
      ball_drive_dropout_duration_s_ += safe_dt;
    // A few rejected frames hold the last observation and brake; sustained
    // loss switches to the bounded pause and downward contact search.
    const bool ball_dropout_grace = ball_started_ && c_.ball_recover_contact &&
        sensor_fresh && contact.measurement_valid && !ball_drive_observation_ok &&
        ball_drive_dropout_duration_s_ <= c_.ball_interact_dropout_grace_s;
    const bool ball_drive_usable = ball_drive_observation_ok || ball_dropout_grace;
    if (c_.task == Task::Ball) {
      if (!contact.measurement_valid) fail(Fault::SensorStale);
      if (ball_previous_contact_ && !ball_drive_usable)
        ball_pause_point_ = pface + Rp * attachment_;
      ball_previous_contact_ = ball_drive_usable;
      ball_contact_loss_duration_s_ = ball_contact_ok ? 0. : ball_contact_loss_duration_s_ + safe_dt;
      // Retain the initial settling gate. After engagement, tolerate only the
      // configured brief classifier dropout before either estimator recovers.
      if (!ball_started_) {
        if (ball_contact_ok && trial_time_ >= c_.preload_time) {
          ball_loaded_contact_duration_s_ += safe_dt;
          if (new_frame) ++ball_loaded_contact_frames_;
        } else { ball_loaded_contact_duration_s_ = 0.; ball_loaded_contact_frames_ = 0; }
        if (ball_loaded_contact_duration_s_ >= c_.ball_contact_settle_s &&
            ball_loaded_contact_frames_ >= 2) { ball_started_ = true; ball_drive_start_ = trial_time_; }
      }
      ball_contact_ready_ = ball_started_ &&
          (ball_drive_usable || !c_.ball_recover_contact);
      ball_contact_paused_ = !ball_contact_ready_;
      if (mode_ != Mode::Fault && new_frame && ball_drive_observation_ok) {
        const double observation = use_eit
            ? (pface + Rp * V3(contact.x, contact.y, 0.))[ball_axis_index]
            : initial_ball_coordinate + c_.ball_baseline_motion_ratio *
                  (pface[ball_axis_index] - initial_face_[ball_axis_index]);
        const double previous = observed_;
        observed_ += c_.ball_position_filter * (observation - observed_);
        const double sample_dt = ball_observation_time_ < 0. ? .05 : std::max(safe_dt, now - ball_observation_time_);
        ball_observed_v_ += c_.ball_velocity_filter * ((observed_ - previous) / sample_dt - ball_observed_v_);
        ball_observation_time_ = now;
        if (c_.update_ball_attachment) {
          V3 updated = attachment_ + limit_norm(.45 * (V3(contact.x, contact.y, 0.) - attachment_), .0025);
          if (updated.head<2>().norm() > c_.active_radius)
            updated.head<2>() *= c_.active_radius / updated.head<2>().norm();
          const V3 shift = Rp * (updated - attachment_);
          attachment_ = updated;
          h0_ += shift; ball_pause_point_ += shift;
          ball_normal_z_ += shift.z(); ball_normal_origin_ += shift.z();
        }
      }
      // These telemetry fields now represent the filtered observation, not a virtual mass.
      z_ = observed_; dz_ = ball_observed_v_;
    }
    const V3 offset = s.ee_rotation * c_.ee_to_pad + Rp * attachment_;
    point = s.ee_position + offset;
    jc = point_jacobian(s.jacobian, offset);
    const V3 velocity = jc.topRows<3>() * s.dq;
    const double ball_goal_sign = c_.roll_distance > 0. ? 1. : -1.;
    const double remaining = ball_goal_sign * (goal_ - observed_);
    if (c_.task == Task::Ball && mode_ == Mode::Interact &&
        ball_drive_observation_ok && ball_contact_ready_) {
      // Prescribed spring-equilibrium motion; it pauses with the lateral drive.
      // Advance beyond the fixed stopping goal only after the ramp gets there.
      if (ball_reference_progress_m_ < std::abs(c_.roll_distance))
        ball_reference_progress_m_ = std::min(std::abs(c_.roll_distance),
            ball_reference_progress_m_ + c_.ball_reference_speed_m_s * safe_dt);
      else if (remaining > c_.ball_goal_tolerance_m &&
               remaining <= c_.ball_reference_advance_start_error_m)
        ball_reference_advance_m_ = std::min(c_.ball_reference_advance_limit_m,
            ball_reference_advance_m_ + c_.ball_reference_advance_speed_m_s * safe_dt);
    }
    const double ball_drive_goal = initial_ball_coordinate + ball_goal_sign *
        (ball_reference_progress_m_ + ball_reference_advance_m_);
    const double activation = smooth(trial_time_ / c_.preload_time);
    V3 G = 2. * ball_direction;
    double normal_ref = normal_anchor_;
    if (c_.task == Task::Joystick) {
      const bool returning_to_zero = c_.joystick_return_zero_start_s > 0. &&
          trial_time_ >= c_.joystick_return_zero_start_s;
      const double target = returning_to_zero ? 0. :
          trial_time_ < 4.5 ? c_.joystick_angle : -c_.joystick_angle;
      const bool along_y = c_.joystick_axis == JoystickAxis::Y;
      const double target_x = along_y ? 0. : target;
      const double target_y = along_y ? target : 0.;
      const double desired_speed = clip(2. * (target_x - z_), -c_.angle_speed, c_.angle_speed);
      if (trial_time_ > c_.preload_time) {
        dz_ += clip((desired_speed - dz_) / .12, -c_.angle_acceleration, c_.angle_acceleration) *
               safe_dt;
        dz_ = clip(dz_, -c_.angle_speed, c_.angle_speed);
        z_ += dz_ * safe_dt;
        if (inward || along_y) {
          const double desired_y_speed = clip(2. * (target_y - y_), -c_.angle_speed, c_.angle_speed);
          dy_ += clip((desired_y_speed - dy_) / .12, -c_.angle_acceleration,
                      c_.angle_acceleration) * safe_dt;
          dy_ = clip(dy_, -c_.angle_speed, c_.angle_speed);
          y_ += dy_ * safe_dt;
        }
      }
      const double theta = along_y ? y_ : z_;
      const V3 direction = along_y ? V3::UnitY() : V3::UnitX();
      ref = h0_ + c_.joystick_length *
          (std::sin(theta) * direction + (std::cos(theta) - 1.) * V3::UnitZ());
      G = c_.joystick_length * (std::cos(theta) * direction - std::sin(theta) * V3::UnitZ());
      normal_ref = normal_anchor_ + c_.joystick_length * (std::cos(theta) - 1.);
      desired_rotation =
          Eigen::AngleAxisd(theta, along_y ? V3(-1., 0., 0.) : V3(0., 1., 0.)).toRotationMatrix() * hold_rotation_;
    } else {
      // A virtual spring equilibrium; goal_ remains the actual stopping goal.
      ref = point + ball_direction * (ball_drive_goal - observed_);
      ref[1-ball_axis_index] = h0_[1-ball_axis_index];
      ref.z() = ball_normal_z_;
      desired_rotation = c_.ready_pad_rotation;
    }
    if (c_.task == Task::Ball && mode_ != Mode::Fault) {
      const bool brake = ball_contact_ok || ball_dropout_grace || !c_.ball_recover_contact;
      const double normal_limit = (1. - activation) *
          std::max(c_.normal_force_limit,ball_initial_preload_) +
          activation * c_.normal_force_limit;
      const double virtual_preload = (1. - activation) * ball_initial_preload_ +
          activation * c_.preload;
      const auto normal = ball_normal_step(c_, ball_normal_z_, ball_normal_v_,
          point.z(), velocity.z(), brake, sensor_fresh && contact.measurement_valid,
          safe_dt, ball_normal_origin_,
          std::min(c_.workspace_min.z()+.0001,ball_normal_origin_),
          normal_limit,virtual_preload);
      out.ball_normal_position = ball_normal_z_;
      out.ball_normal_velocity = normal.used_velocity;
      out.ball_normal_origin = ball_normal_origin_;
      out.ball_normal_brake = brake; out.ball_normal_at_stop = normal.at_stop;
      out.ball_normal_force = normal.force;
      ball_normal_z_ = normal.position; ball_normal_v_ = normal.velocity;
      if (normal.at_stop && !ball_contact_ok) fail(Fault::ContactLoss);
      const double age = std::max(0., trial_time_ - ball_drive_start_);
      ball_last_force_ = ball_drive_force(c_, ball_drive_goal, observed_, ball_observed_v_,
          point[ball_axis_index], velocity[ball_axis_index], ball_pause_point_[ball_axis_index],
          age, ball_contact_ready_, ball_last_force_, safe_dt);
      force = ball_direction * ball_last_force_;
      const int cross = 1 - ball_axis_index;
      force[cross] = clip(c_.coupling_k * (h0_[cross] - point[cross]) -
                          c_.coupling_d * velocity[cross], -c_.force_xy_limit, c_.force_xy_limit);
      force.z() = normal.force; fc = force; fc.z() = 0.;
      if (!ball_contact_ready_) ref.head<2>() = ball_pause_point_.head<2>();
      const bool establishing = !ball_started_;
      moment = limit_norm((establishing ? c_.approach_kr : c_.orientation_k) *
                    orientation_error(Rp, desired_rotation) -
                    (establishing ? c_.approach_dr : c_.orientation_d) * omega,
                    establishing ? c_.approach_moment_limit : c_.moment_limit);
    } else if (inward && mode_ != Mode::Fault) {
      const auto task_shaft = shaft_geometry(z_, y_, dz_, dy_);
      const auto arc = inward_arc(h0_, c_.joystick_length, c_.joystick_inward_arc_m, task_shaft);
      const double radius = c_.joystick_length - c_.joystick_inward_arc_m;
      const auto measured_shaft = shaft_geometry(shaft.angle, shaft.angle_y, shaft.rate, shaft.rate_y);
      // In shaft mode, tangential stretch comes from joystick-angle error.
      // Normal stretch still holds the palm near its contact-defined arc so
      // weight compensation cannot lift the pad away from the joystick.
      ref = arc.position;
      V3 spring_extension = ref - point;
      V3 spring_velocity_error = arc.velocity - velocity;
      if (c_.joystick_task_spring_uses_shaft) {
        ref = c_.joystick_pivot_m + radius * task_shaft.axis;
        const V3 shaft_extension = radius * (task_shaft.axis - measured_shaft.axis);
        spring_extension.head<2>() = shaft_extension.head<2>();
      }
      const auto task_spring = saturating_spring(spring_extension, c_.joystick_task_spring_k,
                                                 c_.joystick_task_spring_force_limit);
      task_spring_scale = task_spring.scale;
      fc = activation * (task_spring.force + c_.joystick_task_spring_d * spring_velocity_error);
      alignment_target = c_.joystick_pivot_m + radius * measured_shaft.axis;
      const V3 target_velocity = radius * measured_shaft.axis_rate;
      desired_rotation = measured_shaft.rotation * hold_rotation_;
      force = fc;
      if (c_.joystick_alignment_enabled) {
        slider_used = slider_;
        const auto links = slider_alignment(pface, Rp, vface, omega,
            alignment_target, target_velocity, desired_rotation, measured_shaft.angular_velocity,
            slider_, c_.joystick_alignment_half_span_m, c_.joystick_alignment_spring_k,
            c_.joystick_alignment_spring_d, c_.joystick_slider_radius_m,
            c_.joystick_alignment_spring_force_limit);
        alignment_centre = links.centre;
        alignment_min_spring_scale = links.min_spring_scale;
        alignment_force = activation * links.force;
        moment = limit_norm(activation * links.moment, c_.moment_limit);
        force += alignment_force;
        const auto js = point_jacobian(s.jacobian, alignment_centre - s.ee_position);
        alignment_torque_correction =
            (js.topRows<3>() - jc.topRows<3>()).transpose() * alignment_force;
        slider_at_stop = links.at_stop;
        slider_ += safe_dt * links.rate;
        if (slider_.norm() > c_.joystick_slider_radius_m)
          slider_ *= c_.joystick_slider_radius_m / slider_.norm();
      }
      if (!inside(alignment_centre) || !inside(alignment_target) ||
          !slider_.allFinite() || !force.allFinite() || !moment.allFinite()) fail(Fault::Workspace);
    } else {
      const double rate = c_.task == Task::Joystick && c_.joystick_axis == JoystickAxis::Y ? dy_ : dz_;
      fc = c_.coupling_k * (ref - point) + c_.coupling_d * (G * rate - velocity);
      fc.z() = 0.;
      for (int i = 0; i < 2; ++i) fc[i] = clip(fc[i], -c_.force_xy_limit, c_.force_xy_limit);
      force = fc;
      const double preload = c_.task == Task::Ball
          ? (1. - activation) * ball_initial_preload_ + activation * c_.preload
          : c_.preload * activation;
      force.z() =
          clip(c_.normal_k * (normal_ref - preload / c_.normal_k - point.z()) -
                   c_.normal_d * velocity.z(),
               -c_.normal_force_limit, c_.normal_force_limit);
      const bool establishing_contact = mode_ == Mode::Preload;
      moment = limit_norm(
          (establishing_contact ? c_.approach_kr : c_.orientation_k) *
                  orientation_error(Rp, desired_rotation) -
              (establishing_contact ? c_.approach_dr : c_.orientation_d) * omega,
          establishing_contact ? c_.approach_moment_limit : c_.moment_limit);
    }
    if (mode_ == Mode::Preload && trial_time_ >= c_.preload_time &&
        (c_.task != Task::Ball || ball_contact_ready_)) mode_ = Mode::Interact;
    if (c_.task == Task::Ball && mode_ == Mode::Interact) {
      const double signed_progress =
          (observed_ - initial_ball_coordinate) * (c_.roll_distance > 0. ? 1. : -1.);
      if (ball_drive_observation_ok && ball_contact_ready_ &&
          signed_progress >= std::abs(c_.roll_distance) - c_.ball_goal_tolerance_m) {
        ball_goal_hold_s_ += safe_dt;
        if (new_frame) ++ball_goal_hold_frames_;
        if (ball_goal_hold_s_ >= c_.ball_goal_dwell_s && ball_goal_hold_frames_ >= 2) {
          ball_goal_reached_ = true;
          begin_retract(s);
        }
      } else {
        ball_goal_hold_s_ = 0.;
        ball_goal_hold_frames_ = 0;
      }
    }
    if (mode_ != Mode::Fault && mode_ != Mode::Retract &&
        trial_time_ >= (c_.task == Task::Joystick ? c_.joystick_duration : c_.ball_duration))
      begin_retract(s);
  }
  if (mode_ == Mode::Retract) {
    unloaded_retract_time_ = sensor_fresh && !contact.contact
                                 ? unloaded_retract_time_ + safe_dt : 0.;
    const double duration = std::max(1., 1.875 * c_.retract_distance / c_.ready_speed);
    ref = phase_start_position_ + smooth(phase_time_ / duration) * (hold_ - phase_start_position_);
    vref = smooth_d(phase_time_ / duration) / duration * (hold_ - phase_start_position_);
    point = pface;
    jc = jf;
    desired_rotation = hold_rotation_;
    if (phase_time_ > duration && (pface - hold_).norm() < .004 && s.dq.norm() < .08)
      mode_ = Mode::Done;
    if (phase_time_ > duration + 10. && mode_ == Mode::Retract) {
      if (unloaded_retract_time_ >= .5) {
        retract_incomplete_ = true;
        mode_ = Mode::Done;
        hold_ = pface;
        hold_rotation_ = Rp;
      } else {
        fail(Fault::RetractTimeout);
      }
    }
  }
  if (mode_ == Mode::Hold || mode_ == Mode::Ready || mode_ == Mode::Approach ||
      mode_ == Mode::Retract || mode_ == Mode::Done) {
    if (mode_ == Mode::Hold || mode_ == Mode::Done) {
      ref = hold_;
      vref.setZero();
      desired_rotation = hold_rotation_;
    }
    const bool moving_to_clearance = mode_ == Mode::Ready || mode_ == Mode::Retract;
    const double position_k = moving_to_clearance ? c_.ready_k : c_.hold_k;
    const double position_d = moving_to_clearance ? c_.ready_d : c_.hold_d;
    force = position_k * (ref - pface) + position_d * (vref - vface);
    if (c_.task == Task::Ball && mode_ == Mode::Approach)
      force.z() = clip(force.z(), -c_.ball_approach_force_limit,
                      c_.ball_approach_force_limit);
    const bool ready = mode_ == Mode::Ready;
    const bool approach = mode_ == Mode::Approach;
    moment = limit_norm((ready ? c_.ready_kr : approach ? c_.approach_kr : c_.hold_kr) *
                            orientation_error(Rp, desired_rotation) -
                            (ready ? c_.ready_dr : approach ? c_.approach_dr : c_.hold_dr) * omega,
                        ready ? c_.ready_moment_limit
                              : approach ? c_.approach_moment_limit : c_.moment_limit);
    point = pface;
    jc = jf;
    alignment_torque_correction.setZero();
    alignment_force.setZero();
  }
  if (mode_ != Mode::Fault && mode_ != Mode::Stopping) {
    if (!inside(ref) || !inside(point)) fail(Fault::Workspace);
  }
  V7 target = V7::Zero();
  if (mode_ != Mode::Fault && mode_ != Mode::Stopping) {
    const Eigen::Matrix<double, 6, 6> inverse =
        (s.jacobian * s.jacobian.transpose() + .0025 * Eigen::Matrix<double, 6, 6>::Identity())
            .ldlt()
            .solve(Eigen::Matrix<double, 6, 6>::Identity());
    const Eigen::Matrix<double, 7, 7> projector =
        Eigen::Matrix<double, 7, 7>::Identity() - s.jacobian.transpose() * inverse * s.jacobian;
    const V7 posture_torque = projector * (c_.null_k * (posture_ - s.q) - c_.null_d * s.dq);
    target = jc.topRows<3>().transpose() * force + s.jacobian.bottomRows<3>().transpose() * moment +
             alignment_torque_correction + posture_torque + s.coriolis;
    // Additional tool weight acts at the EE, independently of the selected contact point.
    out.tool_weight_compensation_force = V3(0., 0., 9.80665 * c_.tool_weight_compensation_mass_kg);
    target += s.jacobian.topRows<3>().transpose() * out.tool_weight_compensation_force;
  }
  if (!target.allFinite()) {
    fail(Fault::Nonfinite);
    target.setZero();
    out.tool_weight_compensation_force.setZero();
  }
  // Send the requested torque directly. libfranka filters/rate-limits it once.
  // Limiting again against filtered state.tau_J_d introduces feedback lag.
  out.torque = target;
  out.mode = mode_;
  out.fault = fault_;
  out.pad_position = pface;
  out.pad_normal = Rp.col(2);
  out.desired_pad_normal = desired_rotation.col(2);
  out.point = point;
  out.reference = ref;
  out.attachment = attachment_;
  out.coupling_force = fc;
  out.requested_force = force + out.tool_weight_compensation_force;
  out.requested_moment = moment;
  const bool progress_y = c_.task == Task::Joystick && c_.joystick_axis == JoystickAxis::Y;
  out.progress = progress_y ? y_ : z_;
  out.progress_velocity = progress_y ? dy_ : dz_;
  out.goal = goal_;
  out.ball_drive_reference = c_.task == Task::Ball
      ? initial_ball_coordinate + std::copysign(
          ball_reference_progress_m_ + ball_reference_advance_m_,c_.roll_distance) : 0.;
  out.angle_command = c_.task == Task::Joystick ? z_ : 0.;
  out.angle_command_y = c_.task == Task::Joystick ? y_ : 0.;
  out.trial_time = trial_time_;
  out.use_eit = use_eit;
  out.contact_source = contact_source_;
  out.ball_axis = c_.ball_axis;
  out.ball_recover_contact = c_.ball_recover_contact;
  out.ball_estimated_velocity = ball_observed_v_;
  out.ball_contact_ready = ball_contact_ready_;
  out.ball_contact_paused = ball_contact_paused_;
  out.ball_goal_reached = ball_goal_reached_;
  out.ball_contact_loss_duration_s = ball_contact_loss_duration_s_;
  out.ball_loaded_contact_duration_s = ball_loaded_contact_duration_s_;
  out.approach_distance_limit_m = approach_distance_limit_m_;
  if (c_.task == Task::Joystick && shaft_fresh)
    out.joystick_tip_position = c_.joystick_pivot_m + c_.joystick_length *
        shaft_geometry(shaft.angle, shaft.angle_y, shaft.rate, shaft.rate_y).axis;
  out.retract_incomplete = retract_incomplete_;
  out.trial_contact_released = trial_contact_released_;
  out.joystick_slipped = joystick_slipped_;
  out.joystick_mechanism = c_.joystick_mechanism;
  out.joystick_axis = c_.joystick_axis;
  out.inward_arc_m = c_.joystick_inward_arc_m;
  out.shaft_angle = shaft.angle; out.shaft_rate = shaft.rate;
  out.shaft_angle_y = shaft.angle_y; out.shaft_rate_y = shaft.rate_y;
  out.shaft_age_s = shaft_age;
  out.shaft_fresh = shaft_fresh; out.shaft_clipped = shaft.clipped;
  out.force_scale = 1.;  // Historical telemetry field: no aggregate-force limiter.
  out.alignment_force = alignment_force;
  out.task_spring_scale = task_spring_scale;
  out.alignment_min_spring_scale = alignment_min_spring_scale;
  out.alignment_centre = alignment_centre; out.alignment_target = alignment_target;
  out.slider = slider_used; out.slider_at_stop = slider_at_stop;
  out.sensor_sequence = contact.sequence;
  out.sensor_contact = contact.contact;
  out.sensor_contact_candidate = contact.contact_candidate;
  out.sensor_fresh = sensor_fresh;
  out.sensor_measurement_valid = contact.measurement_valid;
  out.sensor_position_valid = contact.position_valid;
  out.sensor_physical_model = contact.physical_model;
  out.sensor_age_s = sensor_age;
  out.sensor_signal_strength = contact.signal_strength;
  out.sensor_contact_threshold = contact.contact_threshold;
  out.sensor_pattern_distance = contact.pattern_distance;
  out.sensor_pattern_support_limit = contact.pattern_support_limit;
  out.sensor_predicted_radius_mm = contact.predicted_radius_mm;
  out.sensor_contact_pad = V3(contact.x, contact.y, 0.);
  if (std::isfinite(contact.x) && std::isfinite(contact.y))
    out.sensor_contact_base = pface + Rp * out.sensor_contact_pad;
  else
    out.sensor_contact_base = V3::Constant(std::numeric_limits<double>::quiet_NaN());
  out.finished =
      (mode_ == Mode::Fault || mode_ == Mode::Stopping) && s.previous_torque.allFinite() &&
      s.previous_torque.cwiseAbs().maxCoeff() < 1e-4;
  last_sequence_ = contact.sequence;
  reference_ = ref;
  return out;
}
const char* contact_source_name(ContactSource source) {
  switch(source) {
    case ContactSource::Baseline: return "baseline";
    case ContactSource::Eit: return "eit";
    case ContactSource::Kinematic: return "kinematic";
  }
  return "unknown";
}
const char* mechanism_name(JoystickMechanism m) {
  return m == JoystickMechanism::InwardArc ? "inward_arc" : "legacy";
}
const char* mode_name(Mode x) {
  static const char* names[] = {"hold",    "ready", "approach", "preload", "interact",
                                "retract", "done",  "stopping", "fault"};
  return names[static_cast<int>(x)];
}
const char* fault_name(Fault x) {
  static const char* names[] = {"none",
                                "period",
                                "nonfinite",
                                "workspace",
                                "joint_limit",
                                "joint_speed",
                                "cartesian_speed",
                                "joint7",
                                "sensor_stale_or_nonphysical",
                                "sensor_position",
                                "contact_loss",
                                "approach_timeout",
                                "ready_timeout",
                                "tracking",
                                "heartbeat",
                                "robot",
                                "retract_timeout",
                                "joystick_feedback_stale_or_invalid"};
  return names[static_cast<int>(x)];
}
}  // namespace franka_eit_vmc
