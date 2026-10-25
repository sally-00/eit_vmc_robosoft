#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "franka_eit_vmc/core.hpp"
using namespace franka_eit_vmc;
void require(bool x, const char* m) {
  if (!x) throw std::runtime_error(m);
}
Sample state() {
  Sample s;
  s.q << 0., -.785, 0., -2.356, 0., 1.57, .785;
  s.ee_position << .45, 0., .49;
  s.ee_rotation = Eigen::AngleAxisd(3.141592653589793, V3::UnitX()).toRotationMatrix();
  s.jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  return s;
}
Contact electrical(double now, uint64_t seq = 1, bool touching = false) {
  Contact c;
  c.acquired_steady_s = now;
  c.sequence = seq;
  c.physical_model = true;
  c.position_valid = true;
  c.contact = touching;
  c.x = .01;
  c.y = .02;
  return c;
}
int main() {
  try {
    const auto j = point_jacobian(state().jacobian, V3(.1, .2, .3));
    V7 dq = V7::Zero();
    dq[5] = 1.;
    require((j.topRows<3>() * dq - V3(-.2, .1, 0.)).norm() < 1e-12,
            "point Jacobian must use omega cross r");
    Config invalid;
    invalid.workspace_min.x() = -.1;
    bool rejected = false;
    try {
      invalid.validate();
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    require(rejected, "negative X workspace accepted");
    {
      Core core{Config{}};
      auto s = state();
      core.activate(s);
      s.ee_position.x() -= .05;
      s.external_force.x() = 100.;
      s.external_moment.x() = 100.;
      auto o = core.step(s, electrical(1.), 1., .001, true, Request::None);
      require(o.fault == Fault::None,
              "position lag or estimated external wrench caused a package stop");
    }
    {
      Config cfg;
      cfg.torque_rate = 20000.;
      auto s = state();
      Core hold(cfg);
      hold.activate(s);
      s.ee_position.x() -= .03;
      auto held = hold.step(s, electrical(1.), 1., .001, true, Request::None);
      require(std::abs(held.torque[0] - 18.) < 1e-9 &&
                  std::abs(held.requested_force.x() - 18.) < 1e-9,
              "hold spring force was clipped at the former joint torque limit");
      s = state();
      Core ready(cfg);
      ready.activate(s);
      ready.step(s, electrical(1.), 1., .001, true, Request::Ready);
      s.ee_position.x() -= .03;
      auto moving = ready.step(s, electrical(1.001), 1.001, .001, true, Request::None);
      require(moving.mode == Mode::Ready && std::abs(moving.torque[0] - 18.) < 1e-9 &&
                  std::abs(moving.requested_force.x() - 18.) < 1e-9,
              "ready spring force was clipped at the former joint torque limit");
      cfg.ready_k = 1800.;
      cfg.ready_d = 60.;
      s = state();
      Core stiff_ready(cfg);
      stiff_ready.activate(s);
      stiff_ready.step(s, electrical(1.), 1., .001, true, Request::Ready);
      s.ee_position.x() -= .005;
      auto stiff = stiff_ready.step(s, electrical(1.001), 1.001, .001, true, Request::None);
      require(stiff.mode == Mode::Ready && std::abs(stiff.torque[0] - 9.) < 1e-9,
              "ready-only position stiffness was not applied");
      cfg.ee_to_pad.setZero();
      cfg.ready_kr = 90.;
      cfg.ready_moment_limit = 4.;
      s = state();
      Core angular_hold(cfg);
      Core angular_ready(cfg);
      angular_hold.activate(s);
      angular_ready.activate(s);
      angular_ready.step(s, electrical(1.), 1., .001, true, Request::Ready);
      s.ee_rotation =
          Eigen::AngleAxisd(.1, V3::UnitZ()).toRotationMatrix() * s.ee_rotation;
      auto held_angle = angular_hold.step(s, electrical(1.001), 1.001, .001, true,
                                          Request::None);
      auto moving_angle = angular_ready.step(s, electrical(1.001), 1.001, .001, true,
                                             Request::None);
      require(std::abs(held_angle.torque[5]) <= cfg.moment_limit + 1e-9 &&
                  std::abs(moving_angle.torque[5]) > 3.9,
              "ready-only orientation limit was not applied");
    }
    {
      Config cfg;
      cfg.ready_pad_position.z() = .34;
      auto s = state();
      Core core(cfg);
      core.activate(s);
      double now = 1.;
      auto o = core.step(s, electrical(now), now, .01, true, Request::Ready);
      bool began_descent = false;
      for (int i = 0; i < 500 && !began_descent; ++i) {
        now += .01;
        if (i >= 150) s.ee_position.x() = .4545;
        o = core.step(s, electrical(now), now, .01, true, Request::None);
        began_descent = o.reference.z() < .3499;
      }
      require(began_descent && o.fault == Fault::None,
              "ready did not descend with a 4.5 mm elevated-waypoint residual");
    }
    for (double offset : {.0049, .006}) {
      Core core{Config{}};
      auto s = state();
      core.activate(s);
      s.ee_position.x() += offset;
      auto o = core.step(s, electrical(1.), 1., .001, true, Request::Start);
      require(offset < .005 ? o.mode == Mode::Approach && o.fault == Fault::None
                            : o.mode == Mode::Fault && o.fault == Fault::Tracking,
              "trial-start ready-pose tolerance rejected a small drift or accepted a large one");
    }
    {
      Config cfg;
      cfg.ee_to_pad.setZero();
      cfg.ready_pad_position.z() = .49;
      cfg.approach_kr = 90.;
      cfg.approach_dr = 5.;
      cfg.approach_moment_limit = 4.;
      cfg.orientation_k = 3.;
      cfg.moment_limit = 1.;
      Core core(cfg);
      auto s = state();
      core.activate(s);
      core.step(s, electrical(1.), 1., .001, true, Request::Start);
      s.ee_rotation = Eigen::AngleAxisd(.02, V3::UnitZ()).toRotationMatrix() * s.ee_rotation;
      auto approaching = core.step(s, electrical(1.001, 2), 1.001, .001, true, Request::None);
      require(approaching.mode == Mode::Approach &&
                  std::abs(approaching.requested_moment.norm() - 1.8) < 1e-8,
              "approach did not use its own orientation stiffness");
      auto touching = core.step(s, electrical(1.002, 3, true), 1.002, .001, true, Request::None);
      require(touching.mode == Mode::Preload &&
                  std::abs(touching.requested_moment.norm() - 1.8) < 1e-8,
              "preload did not retain approach orientation stiffness");
    }
    for (bool eit : {false, true}) {
      Config cfg;
      cfg.task = Task::Joystick;
      Core core(cfg);
      auto s = state();
      core.activate(s);
      require(core.set_condition(eit), "condition must change in hold");
      auto o = core.step(s, electrical(1.), 1., .001, true, Request::Start);
      require(o.mode == Mode::Approach, "trial did not approach");
      o = core.step(s, electrical(1.001, 2, true), 1.001, .001, true, Request::None);
      require(o.mode == Mode::Preload, "touch did not engage");
      require((o.attachment - (eit ? V3(.01, .02, 0.) : V3::Zero())).norm() < 1e-12,
              "wrong joystick attachment");
      auto moved = electrical(1.002, 3, true);
      moved.x = -.01;
      moved.y = -.01;
      auto o2 = core.step(s, moved, 1.002, .001, true, Request::None);
      require((o2.attachment - o.attachment).norm() < 1e-12,
              "joystick attachment changed after engagement");
      require(!core.set_condition(!eit), "condition changed during active trial");
      auto loss = core.step(s, electrical(1.003, 4, false), 1.003, .001, true, Request::None);
      require(loss.fault == Fault::ContactLoss, "contact loss not stopped");
    }
    {
      Config cfg;
      cfg.task = Task::Ball;
      Core core(cfg);
      auto s = state();
      core.activate(s);
      core.step(s, electrical(1.), 1., .001, true, Request::Start);
      auto touch = electrical(1.001, 2, true);
      touch.x = 0.;
      touch.y = 0.;
      auto a = core.step(s, touch, 1.001, .001, true, Request::None);
      touch.sequence = 3;
      touch.acquired_steady_s = 1.002;
      touch.x = .01;
      auto b = core.step(s, touch, 1.002, .001, true, Request::None);
      require(b.progress > a.progress, "EIT did not correct progress");
      require((a.reference - b.reference).norm() < 1e-12,
              "state correction changed spring extension");
      require(b.attachment.norm() == 0., "proof-of-concept ball attachment must stay at centre");
    }
    {
      Core core{Config{}};
      auto s = state();
      core.activate(s);
      auto c = electrical(1.);
      c.physical_model = false;
      auto o = core.step(s, c, 1., .001, true, Request::Start);
      require(o.fault == Fault::SensorStale, "demo sensor accepted for physical control");
    }
    {
      Core core{Config{}};
      auto s = state();
      core.activate(s);
      core.step(s, electrical(1.), 1., .001, true, Request::Start);
      auto o = core.step(s, electrical(1.), 2., .001, true, Request::None);
      require(o.fault == Fault::SensorStale, "old contact accepted");
    }
    for (bool released : {false, true}) {
      Config cfg;
      cfg.joystick_duration = .5;
      cfg.preload_time = .1;
      Core core(cfg);
      auto s = state();
      core.activate(s);
      double now = 1.;
      uint64_t sequence = 1;
      auto o = core.step(s, electrical(now, sequence, false), now, .009, true, Request::Start);
      now += .009;
      o = core.step(s, electrical(now, ++sequence, true), now, .009, true, Request::None);
      for (int i = 0; i < 200 && o.mode != Mode::Retract; ++i) {
        now += .009;
        o = core.step(s, electrical(now, ++sequence, true), now, .009, true, Request::None);
      }
      require(o.mode == Mode::Retract, "trial did not enter retract");
      for (int i = 0; i < 1600 && o.mode == Mode::Retract; ++i) {
        now += .009;
        o = core.step(s, electrical(now, ++sequence, !released), now, .009, true,
                      Request::None);
      }
      if (released) {
        require(o.mode == Mode::Done && o.fault == Fault::None && o.retract_incomplete &&
                    (o.reference - o.pad_position).norm() < 1e-9,
                "fresh unloaded retract timeout did not hold for the next ready move");
      } else {
        require(o.mode == Mode::Fault && o.fault == Fault::RetractTimeout &&
                    !o.retract_incomplete,
                "contacting retract timeout incorrectly continued");
      }
    }
    {
      Config stiff_cfg;
      stiff_cfg.joystick_duration = .5;
      stiff_cfg.preload_time = .1;
      stiff_cfg.ready_k = 3000.;
      stiff_cfg.ready_d = stiff_cfg.hold_d;
      Config soft_cfg = stiff_cfg;
      soft_cfg.ready_k = soft_cfg.hold_k;
      Core stiff(stiff_cfg), soft(soft_cfg);
      auto s = state();
      stiff.activate(s);
      soft.activate(s);
      double now = 1.;
      uint64_t sequence = 1;
      stiff.step(s, electrical(now, sequence), now, .009, true, Request::Start);
      soft.step(s, electrical(now, sequence), now, .009, true, Request::Start);
      Output a, b;
      for (int i = 0; i < 200; ++i) {
        now += .009;
        auto contact = electrical(now, ++sequence, true);
        a = stiff.step(s, contact, now, .009, true, Request::None);
        b = soft.step(s, contact, now, .009, true, Request::None);
        if (a.mode == Mode::Retract) break;
      }
      require(a.mode == Mode::Retract && b.mode == Mode::Retract,
              "retract gain test did not enter retract");
      for (int i = 0; i < 100; ++i) {
        now += .009;
        auto contact = electrical(now, ++sequence, true);
        a = stiff.step(s, contact, now, .009, true, Request::None);
        b = soft.step(s, contact, now, .009, true, Request::None);
      }
      const double displacement = a.reference.z() - a.pad_position.z();
      require(a.mode == Mode::Retract && b.mode == Mode::Retract && displacement > .001 &&
                  std::abs((a.requested_force.z() - b.requested_force.z()) -
                           (stiff_cfg.ready_k - soft_cfg.ready_k) * displacement) < 1e-9,
              "retraction did not use the ready position stiffness");
    }
    for (Fault desired :
         {Fault::Workspace, Fault::Joint7, Fault::Period, Fault::Heartbeat}) {
      Core core{Config{}};
      auto s = state();
      core.activate(s);
      core.step(s, electrical(1.), 1., .001, true, Request::Ready);
      if (desired == Fault::Workspace) s.ee_position.x() = -.1;
      if (desired == Fault::Joint7) s.q[6] += .5;
      auto o = core.step(s, electrical(1.001), 1.001, desired == Fault::Period ? .02 : .001,
                         desired != Fault::Heartbeat, Request::None);
      require(o.fault == desired, "guard did not stop control");
    }
    {
      Core core{Config{}};
      auto s = state();
      core.activate(s);
      auto held = core.step(s, electrical(1.), 1., .015, true, Request::None);
      require(held.mode == Mode::Hold && held.fault == Fault::None,
              "idle hold rejected a 15 ms period");
      auto gap = core.step(s, electrical(1.021), 1.021, .021, true, Request::None);
      require(gap.fault == Fault::Period, "idle hold accepted a period over 20 ms");
    }
    {
      Core core{Config{}};
      auto s = state();
      core.activate(s);
      s.previous_torque.setConstant(.5);
      auto o = core.step(s, electrical(1.), 1., .001, true, Request::Stop);
      require((o.torque.array() - .45).abs().maxCoeff() < 1e-12,
              "stop torque did not slew from previous command");
    }
    {
      Core core{Config{}};
      auto s = state();
      core.activate(s);
      s.previous_torque[0] = std::numeric_limits<double>::quiet_NaN();
      auto o = core.step(s, electrical(1.), 1., .001, true, Request::None);
      require(o.fault == Fault::Nonfinite && o.torque.allFinite(),
              "nonfinite previous command propagated to output");
    }
    std::cout << "Core geometry, matched-condition, rebase and fault tests passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
