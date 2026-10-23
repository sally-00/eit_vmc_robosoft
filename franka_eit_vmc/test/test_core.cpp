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
    for (Fault desired :
         {Fault::Workspace, Fault::Joint7, Fault::Period, Fault::Heartbeat, Fault::Wrench}) {
      Core core{Config{}};
      auto s = state();
      core.activate(s);
      core.step(s, electrical(1.), 1., .001, true, Request::Ready);
      if (desired == Fault::Workspace) s.ee_position.x() = -.1;
      if (desired == Fault::Joint7) s.q[6] += .5;
      if (desired == Fault::Wrench) s.external_force.x() = 20.;
      auto o = core.step(s, electrical(1.001), 1.001, desired == Fault::Period ? .02 : .001,
                         desired != Fault::Heartbeat, Request::None);
      require(o.fault == desired, "guard did not stop control");
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
