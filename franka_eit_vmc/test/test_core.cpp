#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "franka_eit_vmc/core.hpp"
#include "franka_eit_vmc/joystick_mechanism.hpp"
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
  c.measurement_valid = true;
  c.position_valid = true;
  c.contact = touching;
  c.contact_candidate = touching;
  c.signal_strength = touching ? .2 : .01;
  c.contact_threshold = .05;
  c.pattern_distance = touching ? .1 : .7;
  c.pattern_support_limit = .4;
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
      Config cfg;
      cfg.joystick_angle = 45. * 3.141592653589793 / 180.;
      cfg.validate();
      cfg.joystick_angle = 0.;
      bool zero_rejected = false;
      try {
        cfg.validate();
      } catch (const std::invalid_argument&) {
        zero_rejected = true;
      }
      require(zero_rejected, "nonpositive joystick command accepted");
    }
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
      cfg.approach_contact_confirm_s = .001;
      cfg.approach_contact_travel_m = .000001;
      cfg.orientation_k = 3.;
      cfg.moment_limit = 1.;
      Core core(cfg);
      auto s = state();
      s.ee_rotation = Eigen::AngleAxisd(.02, V3::UnitZ()).toRotationMatrix() * s.ee_rotation;
      core.activate(s);
      core.step(s, electrical(1.), 1., .001, true, Request::Start);
      s.ee_rotation = Eigen::AngleAxisd(.02, V3::UnitZ()).toRotationMatrix() * s.ee_rotation;
      auto approaching = core.step(s, electrical(1.001, 2), 1.001, .001, true, Request::None);
      require(approaching.mode == Mode::Approach &&
                  std::abs(approaching.requested_moment.norm() - 1.8) < 1e-8,
              "approach did not use its own orientation stiffness");
      core.step(s, electrical(1.002, 3, true), 1.002, .001, true, Request::None);
      s.ee_position.z() -= .00001;
      auto touching = core.step(s, electrical(1.003, 4, true), 1.003, .001, true, Request::None);
      require(touching.mode == Mode::Preload &&
                  std::abs(touching.requested_moment.norm() - 1.8) < 1e-8,
              "preload changed the approach orientation target");
      s.ee_position.x() += .001;
      auto loaded = core.step(s, electrical(1.004, 5, true), 1.004, .001, true, Request::None);
      require(std::abs(loaded.coupling_force.x() + cfg.coupling_k * .001) < 1e-8,
              "contact-point spring was weak at the start of preload");
    }
    for (bool eit : {false, true}) {
      Config cfg;
      cfg.task = Task::Joystick;
      cfg.approach_contact_confirm_s = .001;
      cfg.approach_contact_travel_m = .000001;
      Core core(cfg);
      auto s = state();
      core.activate(s);
      require(core.set_condition(eit), "condition must change in hold");
      auto o = core.step(s, electrical(1.), 1., .001, true, Request::Start);
      require(o.mode == Mode::Approach, "trial did not approach");
      o = core.step(s, electrical(1.001, 2, true), 1.001, .001, true, Request::None);
      require(o.mode == Mode::Approach, "first touch should remain provisional");
      s.ee_position.z() -= .00001;
      o = core.step(s, electrical(1.002, 3, true), 1.002, .001, true, Request::None);
      require(o.mode == Mode::Preload, "touch did not engage");
      require((o.attachment - (eit ? V3(.01, .02, 0.) : V3::Zero())).norm() < 1e-12,
              "wrong joystick attachment");
      auto moved = electrical(1.003, 4, true);
      moved.x = -.01;
      moved.y = -.01;
      auto o2 = core.step(s, moved, 1.003, .001, true, Request::None);
      require((o2.attachment - o.attachment).norm() < 1e-12,
              "joystick attachment changed after engagement");
      require(!core.set_condition(!eit), "condition changed during active trial");
      auto loss = core.step(s, electrical(1.004, 5, false), 1.004, .001, true, Request::None);
      require(loss.fault == Fault::ContactLoss && !loss.sensor_contact &&
                  !loss.sensor_contact_candidate && loss.sensor_fresh &&
                  loss.sensor_measurement_valid && loss.sensor_sequence == 5 &&
                  std::abs(loss.sensor_signal_strength - .01) < 1e-12 &&
                  std::abs(loss.sensor_pattern_distance - .7) < 1e-12,
              "contact loss telemetry did not capture the triggering sensor state");
    }
    {
      Config cfg;
      cfg.approach_contact_confirm_s = .05;
      cfg.approach_contact_travel_m = .0001;
      Core core(cfg);
      auto s = state();
      core.activate(s);
      double now = 1.;
      core.step(s, electrical(now), now, .01, true, Request::Start);
      now += .01;
      auto descending = core.step(s, electrical(now, 2), now, .01, true, Request::None);
      const double z_before_touch = descending.reference.z();
      for (int i = 0; i < 3; ++i) {
        now += .01;
        auto provisional = core.step(s, electrical(now, 3 + i, true), now, .01, true,
                                     Request::None);
        require(provisional.mode == Mode::Approach && provisional.fault == Fault::None &&
                    provisional.reference.z() <= z_before_touch &&
                    provisional.reference.z() >= z_before_touch - cfg.approach_contact_travel_m - 1e-12,
                "provisional contact did not use bounded slow descent");
      }
      now += .01;
      auto resumed = core.step(s, electrical(now, 6, false), now, .01, true, Request::None);
      require(resumed.mode == Mode::Approach && resumed.fault == Fault::None &&
                  resumed.reference.z() < z_before_touch,
              "a vanished provisional contact did not resume approach");
      for (int i = 0; i < 6; ++i) {
        now += .01;
        s.ee_position.z() -= .00002;
        auto stable = core.step(s, electrical(now, 7 + i, true), now, .01, true,
                                Request::None);
        require(stable.fault == Fault::None, "valid confirming contact caused a fault");
        if (i == 5) require(stable.mode == Mode::Preload, "stable contact did not engage");
      }
    }
    {
      Config cfg;
      cfg.approach_contact_confirm_s = .02;
      cfg.approach_contact_travel_m = .0001;
      Core core(cfg);
      auto s = state();
      core.activate(s);
      core.step(s, electrical(1.), 1., .01, true, Request::Start);
      Output seated;
      for (int i = 0; i < 7; ++i) {
        const double now = 1.01 + .01 * i;
        seated = core.step(s, electrical(now, 2 + i, true), now, .01, true,
                           Request::None);
        if (i < 4)
          require(seated.mode == Mode::Approach && seated.fault == Fault::None,
                  "contact engaged before the verification descent finished");
      }
      require(seated.mode == Mode::Preload && seated.fault == Fault::None,
              "stable contact with pad resistance did not engage");
    }
    {
      Config cfg;
      cfg.task = Task::Ball;
      cfg.approach_contact_confirm_s = .001;
      cfg.approach_contact_travel_m = .000001;
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
      s.ee_position.z() -= .00001;
      a = core.step(s, touch, 1.002, .001, true, Request::None);
      touch.sequence = 4;
      touch.acquired_steady_s = 1.003;
      touch.x = .01;
      auto b = core.step(s, touch, 1.003, .001, true, Request::None);
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
      cfg.approach_contact_confirm_s = .001;
      cfg.approach_contact_travel_m = .000001;
      Core core(cfg);
      auto s = state();
      core.activate(s);
      double now = 1.;
      uint64_t sequence = 1;
      auto o = core.step(s, electrical(now, sequence, false), now, .009, true, Request::Start);
      now += .009;
      o = core.step(s, electrical(now, ++sequence, true), now, .009, true, Request::None);
      s.ee_position.z() -= .00001;
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
      stiff_cfg.approach_contact_confirm_s = .001;
      stiff_cfg.approach_contact_travel_m = .000001;
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
        if (i == 1) s.ee_position.z() -= .00001;
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

    // Smaller task radius includes an initial inward offset and the correct derivative.
    for (double theta : {-.7, -.2, 0., .3, .7}) {
      const V3 anchor(.45, 0., .35);
      const double rate = .3, eps = 1e-6;
      const auto short_arc = inward_arc(anchor, .092, .01, theta, rate);
      const auto full_arc = inward_arc(anchor, .092, 0., theta, rate);
      const V3 axis(std::sin(theta), 0., std::cos(theta));
      require((short_arc.position-full_arc.position+.01*axis).norm()<1e-12,
              "task arc is not exactly 10 mm inward at each angle");
      const auto plus=inward_arc(anchor,.092,.01,theta+eps*rate,rate);
      const auto minus=inward_arc(anchor,.092,.01,theta-eps*rate,rate);
      require(((plus.position-minus.position)/(2*eps)-short_arc.velocity).norm()<1e-9,
              "short task arc velocity uses the wrong radius");
      const auto R=(Eigen::AngleAxisd(theta,V3::UnitY())*
          Eigen::AngleAxisd(3.141592653589793,V3::UnitX())).toRotationMatrix();
      const V3 surface=anchor+.092*(axis-V3::UnitZ());
      const auto links=slider_alignment(surface,R,V3::Zero(),V3::Zero(),surface-.01*axis,
          V3::Zero(),R,V3::Zero(),Eigen::Vector2d::Zero(),.08,234.375,78.125,.036);
      require((links.force+9.375*axis).norm()<1e-10 && links.moment.norm()<1e-10,
              "alignment preload is not inward along the current shaft at alignment");
      require((R.leftCols<2>().transpose()*links.force).norm()<1e-10,
              "free planar slider transmitted tangential preload");
    }
    {
      const auto I=Eigen::Matrix3d::Identity();
      const auto Q=Eigen::AngleAxisd(.2,V3::UnitY()).toRotationMatrix();
      const auto links=slider_alignment(V3::Zero(),I,V3::Zero(),V3::Zero(),V3(.01,.02,-.01),
          V3::Zero(),Q,V3::Zero(),Eigen::Vector2d::Zero(),.08,234.375,78.125,.036);
      require(links.force.head<2>().norm()<1e-10 &&
              std::abs(links.moment.y()-3.*std::sin(.2))<1e-10,
              "symmetric slider links lost force balance or orientation spring moment");
      const auto stop=slider_alignment(V3::Zero(),I,V3::Zero(),V3::Zero(),V3(.06,0.,0.),
          V3::Zero(),I,V3::Zero(),Eigen::Vector2d(.036,0.),.08,234.375,78.125,.036);
      require(stop.at_stop && std::abs(stop.rate.x())<1e-10 && stop.force.x()>0.,
              "slider travel stop did not transmit the blocked force");
    }
    {
      Config cfg; cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      cfg.joystick_length=.01;
      bool rejected_depth=false;
      try { cfg.validate(); } catch(const std::invalid_argument&) {rejected_depth=true;}
      require(rejected_depth,"inward depth reached or exceeded the physical length");
    }
    for (int bad : {0,1,2}) {
      Config cfg; cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      Core core(cfg);auto s=state();core.activate(s);
      ShaftFeedback shaft;shaft.sequence=1;shaft.valid=true;shaft.acquired_steady_s=1.;
      if(bad==0)shaft.valid=false;
      if(bad==1)shaft.acquired_steady_s=.5;
      if(bad==2)shaft.clipped=true;
      auto o=core.step(s,electrical(1.),1.,.001,true,Request::Start,shaft);
      require(o.fault==Fault::JoystickFeedback,"new mode accepted unusable live angle feedback");
    }
    {
      Config cfg;cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
      cfg.preload_time=.05;cfg.torque_rate=1000000.;
      auto s=state();
      cfg.joystick_pivot_m=cfg.ready_pad_position-V3(0.,0.,cfg.joystick_length)+V3(.015,0.,0.);
      Config changed=cfg;changed.preload=6.;changed.orientation_k=25.;changed.orientation_d=3.;
      Core core(cfg),other(changed);core.activate(s);other.activate(s);
      double now=1.;uint64_t seq=1;
      ShaftFeedback shaft;shaft.valid=true;
      auto advance=[&](bool touching,Request request) {
        shaft.acquired_steady_s=now;shaft.sequence=seq;
        auto c=electrical(now,seq++,touching);
        auto a=core.step(s,c,now,.001,true,request,shaft);
        auto b=other.step(s,c,now,.001,true,request,shaft);
        require((a.requested_force-b.requested_force).norm()<1e-10 &&
                (a.requested_moment-b.requested_moment).norm()<1e-10,
                "legacy preload/orientation gains leaked into the inward mechanism");
        now+=.001;return a;
      };
      advance(false,Request::Start);advance(true,Request::None);
      s.ee_position.z()-=.00001;
      Output o;
      for(int i=0;i<70;++i)o=advance(true,Request::None);
      require(o.mode==Mode::Interact && o.fault==Fault::None,
              "new mechanism did not engage with fresh angle and contact");
      require(std::abs(o.requested_force.z())<=cfg.normal_force_limit+1e-10 &&
              o.force_scale<1. && o.alignment_force.z()<0.,
              "summed branch force was not capped or inward preload was absent");
      const auto jt=point_jacobian(s.jacobian,o.point-s.ee_position);
      const auto js=point_jacobian(s.jacobian,o.alignment_centre-s.ee_position);
      const V7 expected=jt.topRows<3>().transpose()*(o.requested_force-o.alignment_force)+
          js.topRows<3>().transpose()*o.alignment_force+
          s.jacobian.bottomRows<3>().transpose()*o.requested_moment;
      require((o.torque-expected).norm()<1e-9,
              "alignment force was mapped at the task contact instead of the slider");
      require(!core.set_joystick_mechanism(JoystickMechanism::Legacy,.01),
              "mechanism could change during active control");
      const V3 previous=o.alignment_target;
      shaft.angle=.2;o=advance(true,Request::None);
      const V3 delta=(cfg.joystick_length-cfg.joystick_inward_arc_m)*
          V3(std::sin(.2),0.,std::cos(.2)-1.);
      require((o.alignment_target-previous-delta).norm()<1e-10,
              "alignment followed task command instead of current shaft angle");
      shaft.acquired_steady_s=now-.3;
      auto stale=core.step(s,electrical(now,seq,true),now,.001,true,Request::None,shaft);
      require(stale.fault==Fault::JoystickFeedback,"stale shaft feedback did not stop an active trial");
    }
    std::cout << "Core geometry, matched-condition, rebase and fault tests passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
