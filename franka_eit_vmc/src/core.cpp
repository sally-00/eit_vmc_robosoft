#include "franka_eit_vmc/core.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace franka_eit_vmc {
namespace {
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
  if (!q_min.allFinite() || !q_max.allFinite() || !(q_min.array() < q_max.array()).all() ||
      !torque_limit.allFinite() || (torque_limit.array() <= 0).any())
    throw std::invalid_argument("Invalid joint/torque limits");
  if (use_saved_posture && (!posture_reference.allFinite() ||
                            (posture_reference.array() <= q_min.array() + joint_margin).any() ||
                            (posture_reference.array() >= q_max.array() - joint_margin).any()))
    throw std::invalid_argument("Saved reference posture is outside joint limits");
  const double positive[] = {ready_speed,
                             ready_angular_speed,
                             ready_timeout,
                             approach_speed,
                             approach_distance,
                             approach_timeout,
                             retract_distance,
                             preload,
                             preload_time,
                             hold_k,
                             hold_d,
                             hold_kr,
                             hold_dr,
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
                             torque_rate,
                             joint_margin,
                             joint_speed_limit,
                             point_speed_limit,
                             angular_speed_limit,
                             external_force_limit,
                             external_moment_limit,
                             q7_tolerance,
                             sensor_timeout,
                             heartbeat_timeout,
                             max_period,
                             max_tracking_error,
                             active_radius,
                             joystick_length,
                             joystick_angle,
                             joystick_duration,
                             angle_speed,
                             angle_acceleration,
                             roll_distance,
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
  if (!std::isfinite(initial_ball_x) || initial_ball_x < workspace_min.x() ||
      initial_ball_x + roll_distance > workspace_max.x() || active_radius > .045 ||
      max_period > .02 || preload > normal_force_limit || ready_speed >= point_speed_limit ||
      approach_speed >= point_speed_limit || joystick_angle > .35 || sensor_timeout > 1. ||
      joystick_duration <= preload_time || ball_duration <= preload_time)
    throw std::invalid_argument("Invalid task geometry or timing bounds");
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
  hold_ = face(s);
  reference_ = hold_;
  hold_rotation_ = pad_rotation(s);
  mode_ = Mode::Hold;
  fault_ = Fault::None;
  phase_time_ = 0.;
  settle_time_ = 0.;
  activated_ = true;
  last_sequence_ = 0;
}
bool Core::set_condition(bool use_eit) {
  if (mode_ != Mode::Hold && mode_ != Mode::Done) return false;
  use_eit_ = use_eit;
  return true;
}
void Core::begin_ready(const Sample& s) {
  mode_ = Mode::Ready;
  ready_stage_ = 0;
  phase_time_ = 0.;
  settle_time_ = 0.;
  phase_start_position_ = face(s);
  phase_start_rotation_ = Eigen::Quaterniond(pad_rotation(s));
}
void Core::begin_retract(const Sample& s) {
  mode_ = Mode::Retract;
  phase_time_ = 0.;
  settle_time_ = 0.;
  phase_start_position_ = face(s);
  hold_ = phase_start_position_ + V3(0., 0., c_.retract_distance);
  hold_rotation_ = pad_rotation(s);
}
Output Core::step(const Sample& s, const Contact& contact, double now, double dt, bool heartbeat,
                  Request request) {
  Output out;
  const double safe_dt = std::isfinite(dt) && dt > 0. ? std::min(dt, c_.max_period) : .001;
  if (!activated_) throw std::logic_error("Activate the core before stepping");
  if (!std::isfinite(dt) || dt <= 0. || dt > c_.max_period) fail(Fault::Period);
  if (!std::isfinite(now) || !s.q.allFinite() || !s.dq.allFinite() || !s.coriolis.allFinite() ||
      !s.previous_torque.allFinite() || !s.ee_position.allFinite() || !rotation(s.ee_rotation) ||
      !s.jacobian.allFinite() || !s.external_force.allFinite() || !s.external_moment.allFinite())
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
  if (s.external_force.norm() > c_.external_force_limit ||
      s.external_moment.norm() > c_.external_moment_limit)
    fail(Fault::Wrench);
  if (std::abs(s.q[6] - posture_[6]) > c_.q7_tolerance) fail(Fault::Joint7);
  const bool in_trial =
      mode_ == Mode::Approach || mode_ == Mode::Preload || mode_ == Mode::Interact;
  const double sensor_age = now - contact.acquired_steady_s;
  const bool sensor_fresh = contact.physical_model && contact.sequence > 0 &&
                            std::isfinite(sensor_age) && sensor_age >= -.01 &&
                            sensor_age <= c_.sensor_timeout;
  const bool new_frame = contact.sequence != last_sequence_;
  if (in_trial && !sensor_fresh) fail(Fault::SensorStale);
  if (!heartbeat && mode_ != Mode::Hold && mode_ != Mode::Done && mode_ != Mode::Fault &&
      mode_ != Mode::Stopping)
    fail(Fault::Heartbeat);
  if (request == Request::Stop && mode_ != Mode::Fault) mode_ = Mode::Stopping;
  if (request == Request::Ready && (mode_ == Mode::Hold || mode_ == Mode::Done) && heartbeat)
    begin_ready(s);
  if (request == Request::Start && (mode_ == Mode::Hold || mode_ == Mode::Done) && heartbeat) {
    if (!sensor_fresh || contact.contact)
      fail(Fault::SensorStale);
    else if ((pface - c_.ready_pad_position).norm() > .004 ||
             orientation_error(Rp, c_.ready_pad_rotation).norm() > .05)
      fail(Fault::Tracking);
    else {
      mode_ = Mode::Approach;
      phase_time_ = 0.;
      hold_ = pface;
      hold_rotation_ = Rp;
      last_sequence_ = contact.sequence;
    }
  }
  V3 force = V3::Zero(), moment = V3::Zero(), fc = V3::Zero(), point = pface, ref = hold_,
     vref = V3::Zero();
  J67 jc = jf;
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
    if (phase_time_ >= ready_duration_ && (pface - target).norm() < .003 &&
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
    ref = hold_ - V3(0., 0., std::min(c_.approach_distance, c_.approach_speed * phase_time_));
    vref = V3(0., 0., -c_.approach_speed);
    if (contact.contact && sensor_fresh) {
      const bool need_position =
          c_.task == Task::Joystick ? use_eit_ : use_eit_ || c_.update_ball_attachment;
      if (need_position &&
          (!contact.position_valid || !std::isfinite(contact.x) || !std::isfinite(contact.y)))
        fail(Fault::SensorPosition);
      else {
        attachment_ = V3::Zero();
        if ((c_.task == Task::Joystick && use_eit_) ||
            (c_.task == Task::Ball && c_.update_ball_attachment))
          attachment_ = V3(contact.x, contact.y, 0.);
        if (attachment_.head<2>().norm() > c_.active_radius)
          fail(Fault::SensorPosition);
        else {
          point = pface + Rp * attachment_;
          h0_ = point;
          normal_anchor_ = point.z();
          initial_face_ = pface;
          trial_time_ = 0.;
          phase_time_ = 0.;
          z_ = c_.task == Task::Ball ? c_.initial_ball_x : 0.;
          z0_ = z_;
          dz_ = 0.;
          observed_ = z_;
          goal_ = c_.initial_ball_x + c_.roll_distance;
          mode_ = Mode::Preload;
          last_sequence_ = contact.sequence;
        }
      }
    }
    if (mode_ == Mode::Approach && (phase_time_ > c_.approach_timeout ||
                                    c_.approach_speed * phase_time_ > c_.approach_distance))
      fail(Fault::ApproachTimeout);
  }
  if (mode_ == Mode::Preload || mode_ == Mode::Interact) {
    trial_time_ += safe_dt;
    if (!contact.contact) fail(Fault::ContactLoss);
    if (mode_ != Mode::Fault && c_.task == Task::Ball && new_frame) {
      if (use_eit_ &&
          (!contact.position_valid || !std::isfinite(contact.x) || !std::isfinite(contact.y)))
        fail(Fault::SensorPosition);
      else {
        const double observation = use_eit_
                                       ? (pface + Rp * V3(contact.x, contact.y, 0.)).x()
                                       : c_.initial_ball_x + .5 * (pface.x() - initial_face_.x());
        observed_ = .65 * observed_ + .35 * observation;
        if (c_.update_ball_attachment) {
          if (!contact.position_valid)
            fail(Fault::SensorPosition);
          else {
            V3 delta = .45 * (V3(contact.x, contact.y, 0.) - attachment_);
            delta = limit_norm(delta, .0025);
            const V3 updated = attachment_ + delta;
            if (updated.head<2>().norm() > c_.active_radius)
              fail(Fault::SensorPosition);
            else {
              const V3 shift = Rp * delta;
              attachment_ = updated;
              h0_ += shift;
              normal_anchor_ += shift.z();
            }
          }
        }
        const double correction = clip(.20 * (observed_ - z_), -.003, .003);
        z_ += correction;
        h0_.x() -= 2. * correction;
      }
    }
    const V3 offset = s.ee_rotation * c_.ee_to_pad + Rp * attachment_;
    point = s.ee_position + offset;
    jc = point_jacobian(s.jacobian, offset);
    const V3 velocity = jc.topRows<3>() * s.dq;
    const double activation = smooth(trial_time_ / c_.preload_time);
    V3 G = 2. * V3::UnitX();
    double normal_ref = normal_anchor_;
    if (c_.task == Task::Joystick) {
      const double target = trial_time_ < 4.5 ? c_.joystick_angle : -c_.joystick_angle;
      const double desired_speed = clip(2. * (target - z_), -c_.angle_speed, c_.angle_speed);
      if (trial_time_ > c_.preload_time) {
        dz_ += clip((desired_speed - dz_) / .12, -c_.angle_acceleration, c_.angle_acceleration) *
               safe_dt;
        dz_ = clip(dz_, -c_.angle_speed, c_.angle_speed);
        z_ += dz_ * safe_dt;
      }
      ref =
          h0_ + V3(c_.joystick_length * std::sin(z_), 0., c_.joystick_length * (std::cos(z_) - 1.));
      G = V3(c_.joystick_length * std::cos(z_), 0., -c_.joystick_length * std::sin(z_));
      normal_ref = normal_anchor_ + c_.joystick_length * (std::cos(z_) - 1.);
      desired_rotation =
          Eigen::AngleAxisd(z_, V3::UnitY()).toRotationMatrix() * c_.ready_pad_rotation;
    } else {
      ref = h0_ + 2. * V3::UnitX() * (z_ - z0_);
      desired_rotation = c_.ready_pad_rotation;
      if (Rp.col(2).dot(-V3::UnitZ()) < std::cos(.15)) fail(Fault::Tracking);
    }
    fc = activation * (c_.coupling_k * (ref - point) + c_.coupling_d * (G * dz_ - velocity));
    fc.z() = 0.;
    for (int i = 0; i < 2; ++i) fc[i] = clip(fc[i], -c_.force_xy_limit, c_.force_xy_limit);
    force = fc;
    force.z() =
        clip(c_.normal_k * (normal_ref - c_.preload * activation / c_.normal_k - point.z()) -
                 c_.normal_d * velocity.z(),
             -c_.normal_force_limit, c_.normal_force_limit);
    moment = limit_norm(
        c_.orientation_k * orientation_error(Rp, desired_rotation) - c_.orientation_d * omega,
        c_.moment_limit);
    if (c_.task == Task::Ball && trial_time_ > c_.preload_time) {
      const double effort =
          clip(c_.goal_k * (goal_ - z_), -c_.goal_force_limit, c_.goal_force_limit);
      dz_ += (effort - c_.virtual_damping * dz_ - G.dot(fc)) / c_.virtual_mass * safe_dt;
      dz_ = clip(dz_, -c_.progress_speed, c_.progress_speed);
      z_ += dz_ * safe_dt;
    }
    if (mode_ == Mode::Preload && trial_time_ >= c_.preload_time) mode_ = Mode::Interact;
    if (trial_time_ >= (c_.task == Task::Joystick ? c_.joystick_duration : c_.ball_duration))
      begin_retract(s);
  }
  if (mode_ == Mode::Retract) {
    const double duration = std::max(1., 1.875 * c_.retract_distance / c_.ready_speed);
    ref = phase_start_position_ + smooth(phase_time_ / duration) * (hold_ - phase_start_position_);
    vref = smooth_d(phase_time_ / duration) / duration * (hold_ - phase_start_position_);
    point = pface;
    jc = jf;
    desired_rotation = hold_rotation_;
    if (phase_time_ > duration && (pface - hold_).norm() < .004 && s.dq.norm() < .08)
      mode_ = Mode::Done;
    if (phase_time_ > duration + 10.) fail(Fault::ReadyTimeout);
  }
  if (mode_ == Mode::Hold || mode_ == Mode::Ready || mode_ == Mode::Approach ||
      mode_ == Mode::Retract || mode_ == Mode::Done) {
    if (mode_ == Mode::Hold || mode_ == Mode::Done) {
      ref = hold_;
      vref.setZero();
      desired_rotation = hold_rotation_;
    }
    force =
        limit_norm(c_.hold_k * (ref - pface) + c_.hold_d * (vref - vface), c_.normal_force_limit);
    moment = limit_norm(c_.hold_kr * orientation_error(Rp, desired_rotation) - c_.hold_dr * omega,
                        c_.moment_limit);
    point = pface;
    jc = jf;
  }
  if (mode_ != Mode::Fault && mode_ != Mode::Stopping) {
    if (!inside(ref) || !inside(point)) fail(Fault::Workspace);
    if ((ref - point).norm() > c_.max_tracking_error) fail(Fault::Tracking);
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
             posture_torque + s.coriolis;
  }
  if (!target.allFinite()) {
    fail(Fault::Nonfinite);
    target.setZero();
  }
  for (int i = 0; i < 7; ++i) {
    const double bounded = clip(target[i], -c_.torque_limit[i], c_.torque_limit[i]);
    // Slew all torque, including faults and shutdown, from the previous hardware command.
    const double previous = std::isfinite(s.previous_torque[i]) ? s.previous_torque[i] : 0.;
    out.torque[i] =
        previous + clip(bounded - previous, -c_.torque_rate * safe_dt, c_.torque_rate * safe_dt);
  }
  out.mode = mode_;
  out.fault = fault_;
  out.pad_position = pface;
  out.point = point;
  out.reference = ref;
  out.attachment = attachment_;
  out.coupling_force = fc;
  out.progress = z_;
  out.progress_velocity = dz_;
  out.goal = goal_;
  out.angle_command = c_.task == Task::Joystick ? z_ : 0.;
  out.trial_time = trial_time_;
  out.use_eit = use_eit_;
  out.finished =
      (mode_ == Mode::Fault || mode_ == Mode::Stopping) && out.torque.cwiseAbs().maxCoeff() < 1e-4;
  last_sequence_ = contact.sequence;
  reference_ = ref;
  return out;
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
                                "wrench",
                                "joint7",
                                "sensor_stale_or_nonphysical",
                                "sensor_position",
                                "contact_loss",
                                "approach_timeout",
                                "ready_timeout",
                                "tracking",
                                "heartbeat",
                                "robot"};
  return names[static_cast<int>(x)];
}
}  // namespace franka_eit_vmc
