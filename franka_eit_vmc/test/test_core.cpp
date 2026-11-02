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
    for(double tau : {0.,-1.,std::numeric_limits<double>::infinity()}) {
      Config cfg;cfg.joystick_attachment_filter_tau_s=tau;bool bad=false;
      try{cfg.validate();}catch(const std::invalid_argument&){bad=true;}
      require(bad,"invalid attachment time constant accepted");
    }
    for(double t : {-1.,4.5,19.,std::numeric_limits<double>::quiet_NaN()}) {
      Config cfg;cfg.joystick_duration=19.;cfg.joystick_return_zero_start_s=t;bool bad=false;
      try{cfg.validate();}catch(const std::invalid_argument&){bad=true;}
      require(bad,"invalid joystick return-to-zero timing accepted");
    }
    for(double hold : {-1.,std::numeric_limits<double>::quiet_NaN()}) {
      Config cfg;cfg.joystick_approach_final_hold_s=hold;bool bad=false;
      try{cfg.validate();}catch(const std::invalid_argument&){bad=true;}
      require(bad,"invalid joystick final approach hold accepted");
    }
    Config invalid;
    invalid.workspace_min.x() = -.1;
    bool rejected = false;
    try {
      invalid.validate();
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    require(rejected, "negative X workspace accepted");
    for(double mass : {-0.5, std::numeric_limits<double>::infinity()}) {
      Config cfg;cfg.tool_weight_compensation_mass_kg=mass;bool invalid_mass=false;
      try{cfg.validate();}catch(const std::invalid_argument&){invalid_mass=true;}
      require(invalid_mass,"invalid weight compensation mass accepted");
    }
    {
      Config plain;
      plain.approach_contact_confirm_s=.001;plain.approach_contact_travel_m=.000001;
      plain.preload_time=.05;plain.joystick_duration=.4;
      Config weighted=plain;weighted.tool_weight_compensation_mass_kg=.5;
      Core a(plain),b(weighted);auto s=state();a.activate(s);b.activate(s);
      double now=1.;uint64_t sequence=1;
      auto compare=[&](bool touching,Request request=Request::None) {
        const auto contact=electrical(now,sequence++,touching);
        const auto first=a.step(s,contact,now,.001,true,request);
        const auto second=b.step(s,contact,now,.001,true,request);
        now+=.001;
        require(first.mode==second.mode && first.fault==second.fault,"compensation changed state transitions");
        const V3 up=second.mode==Mode::Stopping || second.mode==Mode::Fault ? V3::Zero() : V3(0.,0.,4.903325);
        require((second.tool_weight_compensation_force-up).norm()<1e-12 &&
                (second.torque-first.torque-s.jacobian.topRows<3>().transpose()*up).norm()<1e-9,
                "500 g compensation was not applied upward at the EE Jacobian");
        require((second.requested_force-first.requested_force-up).norm()<1e-9 &&
                (second.requested_moment-first.requested_moment).norm()<1e-9,
                "weight compensation incorrectly changed the contact moment or force telemetry");
        return second;
      };
      require(compare(false).mode==Mode::Hold,"compensation Hold test failed");
      compare(false,Request::Start);compare(true);s.ee_position.z()-=.00001;
      bool preload=false,interact=false,retract=false,done=false;
      for(int i=0;i<4000 && !done;++i) {
        auto out=compare(true);
        preload|=out.mode==Mode::Preload;interact|=out.mode==Mode::Interact;
        retract|=out.mode==Mode::Retract;done|=out.mode==Mode::Done;
        if(out.mode==Mode::Retract)s.ee_position+=out.reference-out.pad_position;
      }
      require(preload && interact && retract && done,"weight compensation trial stages not covered");
      require(compare(false,Request::Ready).mode==Mode::Ready,"compensation Ready test failed");
      require(compare(false,Request::Stop).mode==Mode::Stopping,"compensation did not leave normal control on stop");
      a.activate(s);b.activate(s);s.dq[0]=2.;
      require(compare(false).mode==Mode::Fault,"compensation fault ramp-down test failed");
    }
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
      const auto contact = electrical(1.);
      const auto logged = core.step(s, contact, 1., .001, true, Request::None);
      const V3 local(contact.x, contact.y, 0.);
      require((logged.sensor_contact_pad - local).norm() < 1e-12 &&
                  (logged.sensor_contact_base -
                   (logged.pad_position + s.ee_rotation * Config{}.ee_R_pad * local)).norm() < 1e-12,
              "EIT contact position telemetry must follow the pad-to-base transform");
      s.ee_position.x() -= .05;
      s.external_force.x() = 100.;
      s.external_moment.x() = 100.;
      auto o = core.step(s, electrical(1.), 1., .001, true, Request::None);
      require(o.fault == Fault::None,
              "position lag or estimated external wrench caused a package stop");
    }
    {
      Config cfg;
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
    // Contact arriving after the descent budget is exhausted must be confirmable
    // while the final reference holds, for both baseline and EIT attachments.
    for(bool eit : {false,true}) for(bool late_contact : {false,true}) {
      Config cfg;cfg.approach_distance=.002;cfg.approach_speed=.004;
      cfg.joystick_approach_final_hold_s=2.;cfg.approach_contact_confirm_s=.25;
      auto s=state();Core core(cfg);core.activate(s);core.set_condition(eit);
      double now=1.;uint64_t sequence=1;
      auto step=[&](bool touching,Request req=Request::None) {
        auto out=core.step(s,electrical(now,sequence++,touching),now,.001,true,req);
        now+=.001;return out;
      };
      auto out=step(false,Request::Start);
      const double initial_z=out.reference.z()+cfg.approach_speed*.001;
      for(int i=0;i<699;++i) {
        out=step(false);
        require(out.mode==Mode::Approach && out.fault==Fault::None,
                "joystick failed immediately at the descent limit");
        require(out.reference.z()>=initial_z-cfg.approach_distance-1e-12,
                "final approach hold descended beyond its limit");
      }
      const double held_z=out.reference.z();
      const double expected_z_force=cfg.hold_k*(held_z-out.pad_position.z());
      require(std::abs(out.requested_force.z()-expected_z_force)<1e-10,
              "final hold retained a downward reference-velocity force");
      if(late_contact) {
        if(eit) {
          for(int i=0;i<300;++i) {
            auto invalid=electrical(now,sequence++,true);invalid.position_valid=false;
            out=core.step(s,invalid,now,.001,true,Request::None);now+=.001;
          }
          require(out.mode==Mode::Approach,"final hold bypassed the EIT position check");
        }
        // First contact occurs after all descent is used; an intervening
        // rejected frame must reset the full confirmation period.
        for(int i=0;i<150;++i)out=step(true);
        require(out.mode==Mode::Approach,"short late contact engaged early");
        out=step(false);
        for(int i=0;i<200;++i)out=step(true);
        require(out.mode==Mode::Approach,"contact dropout did not reset confirmation");
        for(int i=0;i<60;++i)out=step(true);
        require(out.mode==Mode::Preload && out.fault==Fault::None,
                "stable contact at the final reference could not engage");
      } else {
        for(int i=0;i<1900 && out.mode==Mode::Approach;++i) {
          out=step(false);
          if(out.mode==Mode::Approach)
            require(std::abs(out.reference.z()-held_z)<1e-12,
                    "final approach reference moved while waiting");
        }
        require(out.mode==Mode::Fault && out.fault==Fault::ApproachTimeout &&
                now>=3.49 && now<3.52,
                "no-contact final hold was not bounded to two seconds");
      }
    }
    {
      // The real ball trace first reported stable contact with only about
      // 0.5 mm left in a 30 mm approach. Confirmation must use that remaining
      // travel rather than wait forever for the configured full 1 mm.
      Config cfg;cfg.task=Task::Ball;cfg.ee_to_pad.setZero();
      cfg.ready_pad_position=V3(.45,0.,.49);
      cfg.workspace_min.z()=.46;
      cfg.approach_distance=.03;cfg.approach_speed=.01;
      cfg.approach_contact_speed=.005;cfg.approach_contact_travel_m=.001;
      cfg.approach_contact_confirm_s=.25;cfg.approach_timeout=8.;
      auto s=state();Core core(cfg);core.activate(s);
      double now=1.;uint64_t sequence=1;
      auto advance=[&](bool touching,Request request=Request::None) {
        auto out=core.step(s,electrical(now,sequence++,touching),now,.01,true,request);
        now+=.01;return out;
      };
      Output out=advance(false,Request::Start);
      for(int i=0;i<400 && out.reference.z()>cfg.ready_pad_position.z()-.0295;++i)
        out=advance(false);
      const double first_contact_depth=cfg.ready_pad_position.z()-out.reference.z();
      require(out.mode==Mode::Approach && out.fault==Fault::None &&
                  first_contact_depth>.0294 && first_contact_depth<.03,
              "late-contact regression did not reach the approach boundary");
      bool confirmed=false;
      for(int i=0;i<50;++i) {
        out=advance(true);
        if(out.mode==Mode::Approach)
          require(out.reference.z()>=cfg.ready_pad_position.z()-cfg.approach_distance-1e-9,
                  "contact verification exceeded the bounded approach distance");
        if(out.mode==Mode::Preload) {confirmed=true;break;}
      }
      require(confirmed && out.fault==Fault::None,
              "stable late contact could not confirm within the remaining approach travel");
    }
    {
      // A distant ball Ready pose may need more than the old fixed approach
      // distance. Search stays above the configured workspace floor, bounds
      // vertical force, and faults if no contact is found there.
      Config cfg;cfg.task=Task::Ball;cfg.ee_to_pad.setZero();
      cfg.ready_pad_position=V3(.45,0.,.49);
      cfg.workspace_min.z()=.05;cfg.approach_distance=.005;
      cfg.approach_speed=.04;cfg.approach_timeout=8.;
      auto s=state();Core core(cfg);core.activate(s);
      double now=1.;uint64_t sequence=1;
      auto advance=[&](Request request=Request::None) {
        auto out=core.step(s,electrical(now,sequence++,false),now,.01,true,request);
        now+=.01;return out;
      };
      Output out=advance(Request::Start);
      require(out.approach_distance_limit_m>.43,
              "ball approach still uses the fixed distance cap");
      for(int i=0;i<1000;++i) {
        out=advance();
        require(out.mode==Mode::Approach && out.fault==Fault::None &&
                    out.reference.z()>=cfg.workspace_min.z() &&
                    std::abs(out.requested_force.z())<=cfg.ball_approach_force_limit+1e-9,
                "distant ball approach hit a fixed timeout, floor, or force excess");
      }
      bool reached_floor=false;
      for(int i=0;i<200 && out.mode==Mode::Approach;++i) {
        out=advance();
        if(out.mode==Mode::Approach &&
           out.reference.z()<=cfg.workspace_min.z()+.0001+cfg.approach_speed*.01+1e-9)
          reached_floor=true;
      }
      require(reached_floor && out.mode==Mode::Fault && out.fault==Fault::ApproachTimeout,
              "no-contact approach did not stop at the workspace floor");
    }
    for (bool update : {false,true}) for (bool eit : {false, true}) {
      Config cfg;
      cfg.task = Task::Joystick;
      cfg.preload_time = 1.;
      cfg.joystick_update_eit_attachment=update;
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
      const double u=.001/cfg.joystick_attachment_filter_tau_s;
      const V3 target=eit ? V3(-.01,-.01,0.) : V3::Zero();
      const V3 expected=eit && update ? target+(o.attachment-target)*(1.+u)*std::exp(-u) : o.attachment;
      require((o2.attachment-expected).norm()<1e-12,
              "attachment filter does not match the exact two-pole response");
      require(!eit || (o2.attachment-o.attachment).norm()<.00001,
              "new sensor frame stepped the attachment");
      require(!core.set_condition(!eit), "condition changed during active trial");
      auto loss = core.step(s, electrical(1.004, 5, false), 1.004, .001, true, Request::None);
      require(loss.fault == Fault::None && loss.mode == Mode::Preload &&
                  (loss.attachment-o2.attachment).norm()<1e-12 && !loss.sensor_contact &&
                  !loss.sensor_contact_candidate && loss.sensor_fresh &&
                  loss.sensor_measurement_valid && loss.sensor_sequence == 5 &&
                  std::abs(loss.sensor_signal_strength - .01) < 1e-12 &&
                  std::abs(loss.sensor_pattern_distance - .7) < 1e-12,
              "joystick contact loss did not retain the established attachment");
      auto rejected=electrical(1.005,6,true);rejected.x=.1;
      auto held=core.step(s,rejected,1.005,.001,true,Request::None);
      require((held.attachment-loss.attachment).norm()<1e-12,
              "out-of-pad measurement changed attachment");
      rejected=electrical(1.006,7,true);rejected.x=std::numeric_limits<double>::quiet_NaN();
      held=core.step(s,rejected,1.006,.001,true,Request::None);
      require((held.attachment-loss.attachment).norm()<1e-12,
              "nonfinite measurement changed attachment");
      rejected=electrical(1.007,8,true);rejected.position_valid=false;
      held=core.step(s,rejected,1.007,.001,true,Request::None);
      require((held.attachment-loss.attachment).norm()<1e-12,
              "invalid position changed attachment");
      for(int i=0;i<600;++i) {
        double now=1.008+i*.001;
        auto frame=electrical(now,9+i/20,true);frame.x=-.01;frame.y=-.01;
        held=core.step(s,frame,now,.001,true,Request::None);
        require(held.fault==Fault::None,"filter test faulted");
      }
      require((held.attachment-(eit && update ? target : o.attachment)).norm()<.0002,
              "attachment did not converge to held 50 Hz contact, or baseline moved");
      require((held.point-(s.ee_position+s.ee_rotation*cfg.ee_to_pad+
                  s.ee_rotation*cfg.ee_R_pad*held.attachment)).norm()<1e-12,
              "point did not use filtered attachment and robot orientation");
      require((held.reference-o.reference).norm()<.00001,
              "updating attachment translated the task target during preload");
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
      auto loss=core.step(s,electrical(1.004,5,false),1.004,.001,true,Request::None);
      require(loss.fault==Fault::None && loss.ball_contact_paused,
              "brief ball contact dropout was not paused");
    }
    {
      Config cfg;cfg.task=Task::Ball;cfg.normal_force_limit=4.;cfg.hold_k=1500.;
      cfg.approach_contact_confirm_s=.02;cfg.approach_contact_travel_m=.000001;
      Core core(cfg);auto s=state();core.activate(s);
      core.step(s,electrical(1.),1.,.001,true,Request::Start);
      Output before,after;double now=1.001;uint64_t seq=2;
      // The search cap remains above the 4 N preload cap because a 4 N
      // approach cap previously left the pad well above its reference.
      for(int i=0;i<1000;++i) {
        before=core.step(s,electrical(now,seq++,false),now,.001,true,Request::None);now+=.001;
        require(before.fault==Fault::None,"ball approach faulted before contact");
      }
      do {
        before=after.mode==Mode::Approach ? after : before;
        after=core.step(s,electrical(now,seq++,true),now,.001,true,Request::None);now+=.001;
      } while(after.mode==Mode::Approach);
      require(before.requested_force.z() < -4. &&
              before.requested_force.z()>=-cfg.ball_approach_force_limit-1e-9 &&
              after.mode==Mode::Preload &&
              std::abs(after.requested_force.z()-before.requested_force.z())<.5 &&
              after.ball_normal_force>=-cfg.ball_approach_force_limit-1e-9,
              "ball approach-to-preload handoff abruptly unloaded contact");
      for(int i=0;i<700;++i) {
        after=core.step(s,electrical(now,seq++,true),now,.001,true,Request::None);
        now+=.001;
      }
      require(after.fault==Fault::None &&
                  after.ball_normal_force>=-cfg.normal_force_limit-1e-9,
              "ball preload did not return to its configured steady force cap");
    }
    {
      // A lost ball contact must stop the bounded virtual search at the
      // workspace floor, never command a reference below that floor.
      Config cfg;cfg.task=Task::Ball;cfg.ee_to_pad.setZero();
      cfg.ready_pad_position=V3(.45,0.,.49);cfg.workspace_min.z()=.46;
      cfg.ball_normal_travel=.05;cfg.approach_contact_confirm_s=.001;
      cfg.approach_contact_travel_m=.000001;
      auto s=state();Core core(cfg);core.activate(s);
      double now=1.;uint64_t sequence=1;
      auto advance=[&](bool touching,Request request=Request::None) {
        auto out=core.step(s,electrical(now,sequence++,touching),now,.001,true,request);
        now+=.001;return out;
      };
      advance(false,Request::Start);
      Output out=advance(true);
      for(int i=0;i<10 && out.mode==Mode::Approach;++i)out=advance(true);
      require(out.mode==Mode::Preload,"contact-loss floor test did not engage");
      for(int i=0;i<5000 && out.mode!=Mode::Fault;++i) {
        out=advance(false);
        if(out.mode!=Mode::Fault)
          require(out.reference.z()>=cfg.workspace_min.z(),
                  "ball recovery commanded a reference below the workspace floor");
      }
      require(out.fault==Fault::ContactLoss && out.ball_normal_at_stop &&
                  out.pad_position.z()>cfg.workspace_min.z(),
              "exhausted ball recovery was mislabeled as a workspace fault");
    }
    // Loaded contact settles before motion. Isolated bad frames keep the last
    // observation; sustained loss pauses lateral drive and starts recovery.
    {
      const bool use_eit=true;
      Config cfg; cfg.task=Task::Ball;
      cfg.approach_contact_confirm_s=.001; cfg.approach_contact_travel_m=.000001;
      cfg.preload_time=.05; cfg.ball_contact_settle_s=.03; cfg.ball_contact_loss_grace_s=.02;
      Core core(cfg); auto s=state(); core.activate(s); core.set_condition(use_eit);
      double now=1.; uint64_t seq=1;
      auto advance=[&](bool touching, bool valid_position=true, Request request=Request::None) {
        auto contact=electrical(now,seq++,touching);contact.x=0.;contact.y=0.;
        contact.position_valid=valid_position;
        if(!valid_position)contact.x=std::numeric_limits<double>::quiet_NaN();
        auto out=core.step(s,contact,now,.001,true,request);now+=.001;return out;
      };
      advance(false,true,Request::Start);
      Output out;
      do {out=advance(true);} while(out.mode==Mode::Approach);
      require(out.mode==Mode::Preload && out.requested_force.z()<-.001,
              "approach-to-preload handoff unloaded the ball");
      const V3 initial=out.reference;
      while(out.trial_time<.078) {
        out=advance(true);
        require(out.fault==Fault::None && !out.ball_contact_ready &&
                (out.reference-initial).head<2>().norm()<1e-10,
                "ball rolled before loaded contact settled");
      }
      for(int i=0;i<80;++i)out=advance(true);
      require(out.mode==Mode::Interact && out.ball_contact_ready && out.requested_force.x()>0.,
              "stable loaded contact did not start rolling");
      const double held_estimate=out.progress;
      const double held_drive_reference=out.ball_drive_reference;
      for(int i=0;i<20;++i) {
        out=advance(true,false);
        require(out.fault==Fault::None && out.ball_contact_ready &&
                !out.ball_contact_paused && out.ball_normal_brake &&
                out.progress==held_estimate &&
                out.ball_drive_reference==held_drive_reference,
                "brief invalid EIT position did not retain the last estimate and brake");
      }
      for(int i=0;i<21;++i)out=advance(true,false);
      require(out.fault==Fault::None && out.ball_contact_paused &&
              out.ball_normal_brake && !out.ball_contact_ready,
              "sustained missing EIT position did not pause lateral drive");
      out=advance(true);
      require(out.fault==Fault::None && out.ball_contact_ready && !out.ball_contact_paused,
              "valid EIT position did not resume lateral drive immediately");
      for(int i=0;i<20;++i) {
        out=advance(false,false);
        require(out.fault==Fault::None && out.ball_contact_ready &&
                !out.ball_contact_paused && out.ball_normal_brake,
                "brief no-contact report released the pad during grace");
      }
      for(int i=0;i<21;++i)out=advance(false,false);
      const V3 paused=out.reference;
      require(out.fault==Fault::None && out.ball_contact_paused &&
              out.progress_velocity==0. && (out.reference-out.point).head<2>().norm()<1e-10,
              "dropout did not hold the current lateral contact point");
      for(int i=0;i<40;++i) {
        out=advance(false,false);
        require(out.fault==Fault::None && out.ball_contact_paused &&
                (out.reference-paused).head<2>().norm()<1e-10 &&
                out.requested_force.z()<=0. && out.requested_force.z()>=-cfg.normal_force_limit &&
                !out.ball_normal_brake,
                "valid loss did not release bounded normal carriage and hold lateral point");
      }
      out=advance(true);
      require(out.ball_contact_ready && !out.ball_contact_paused && out.ball_normal_brake,
              "contact return did not resume direct control with one-way brake");
      for(int i=0;i<50;++i)out=advance(false,false);
      require(out.fault==Fault::None && out.ball_contact_paused,
              "dropout grace incorrectly terminated bounded carriage recovery");
    }
    // Default baseline and EIT use identical contact recovery; only observations differ.
    {
      Config cfg;cfg.task=Task::Ball;cfg.ee_to_pad.setZero();cfg.ready_pad_position.z()=.49;
      cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
      cfg.preload_time=.05;cfg.ball_contact_settle_s=.03;
      Core core(cfg);auto s=state();core.activate(s);
      double now=1.;uint64_t seq=1;
      auto advance=[&](bool touching,Request request=Request::None) {
        auto out=core.step(s,electrical(now,seq++,touching),now,.001,true,request);
        now+=.001;return out;
      };
      advance(false,Request::Start);advance(true);
      s.ee_position.z()-=.00001;
      Output out;
      for(int i=0;i<120;++i)out=advance(true);
      require(out.mode==Mode::Interact && out.fault==Fault::None,
              "ball orientation test did not enter rolling");
      s.ee_rotation=Eigen::AngleAxisd(.20,V3::UnitX()).toRotationMatrix()*s.ee_rotation;
      out=advance(true);
      require(out.mode==Mode::Interact && out.fault==Fault::None &&
              out.requested_moment.x()<-.1 &&
              out.requested_moment.norm()<=cfg.moment_limit+1e-9,
              "ball tilt stopped rolling or disabled bounded orientation control");
    }
    for(bool eit : {false,true}) {
      Config cfg;cfg.task=Task::Ball;cfg.preload_time=.05;
      cfg.ball_contact_settle_s=.03;cfg.ball_contact_loss_grace_s=.02;
      cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
      Core core(cfg);auto s=state();core.activate(s);core.set_condition(eit);
      double now=1.;uint64_t seq=1;
      auto advance=[&](bool touching,Request request=Request::None) {
        auto c=electrical(now,seq++,touching);c.x=0.;c.y=0.;
        if(!touching){c.position_valid=false;c.x=std::numeric_limits<double>::quiet_NaN();}
        auto out=core.step(s,c,now,.001,true,request);now+=.001;return out;
      };
      advance(false,Request::Start);Output out;
      do {out=advance(true);} while(out.mode==Mode::Approach);
      for(int i=0;i<100;++i)out=advance(true);
      require(out.mode==Mode::Interact && out.requested_force.x()>0.,"direct ball drive did not engage");
      const double estimate=out.progress;
      for(int i=0;i<100;++i)out=advance(false);
      require(out.fault==Fault::None && out.ball_contact_paused && !out.ball_normal_brake &&
              out.progress==estimate,"position comparison did not share recovery and hold missing observation");
      s.ee_position.x()+=.003;
      out=advance(true);
      require(out.fault==Fault::None && out.ball_contact_ready && out.progress>estimate,
              "position estimate failed to resume after contact returned");
    }
    {
      Config cfg;cfg.task=Task::Ball;cfg.approach_contact_confirm_s=.001;
      cfg.approach_contact_travel_m=.000001;
      Core core(cfg);auto s=state();core.activate(s);
      core.step(s,electrical(1.),1.,.001,true,Request::Start);
      Output out;double now=1.001;uint64_t seq=2;
      do {out=core.step(s,electrical(now,seq++,true),now,.001,true,Request::None);now+=.001;}
      while(out.mode==Mode::Approach);
      auto bad=electrical(now,seq,true);bad.measurement_valid=false;
      require(core.step(s,bad,now,.001,true,Request::None).fault==Fault::SensorStale,
              "ball dropout grace accepted invalid electrical measurements");
    }
    {
      Config cfg;cfg.task=Task::Ball;cfg.approach_contact_confirm_s=.1;
      cfg.ball_contact_loss_grace_s=.02;
      Core core(cfg);auto s=state();core.activate(s);
      core.step(s,electrical(1.),1.,.001,true,Request::Start);
      const auto touch=core.step(s,electrical(1.001,2,true),1.001,.001,true,Request::None);
      Output out;
      for(int i=0;i<15;++i) {
        const double now=1.002+.001*i;
        out=core.step(s,electrical(now,3+i,false),now,.001,true,Request::None);
        require(out.mode==Mode::Approach && (out.reference-touch.reference).norm()<1e-12,
                "provisional contact dropout caused renewed fast descent");
      }
      for(int i=0;i<15;++i) {
        const double now=1.017+.001*i;
        out=core.step(s,electrical(now,18+i,false),now,.001,true,Request::None);
      }
      require(out.reference.z()<touch.reference.z() &&
              touch.reference.z()-out.reference.z()<=.011*cfg.approach_contact_speed+1e-10,
              "provisional dropout did not resume at the verification speed");
    }
    // The longer shaft changes contact height; available descent follows live tilt.
    for(double ax : {0., .6108652381980153}) for(double ay : {0., .6108652381980153}) {
      Config cfg;cfg.ready_pad_position=V3(.45,0.,.1575);cfg.joystick_length=.1025;
      cfg.approach_distance=.060;cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      auto s=state();s.ee_position=cfg.ready_pad_position-s.ee_rotation*cfg.ee_to_pad;
      Core core(cfg);core.activate(s);
      ShaftFeedback shaft;shaft.sequence=1;shaft.acquired_steady_s=1.;shaft.valid=true;
      shaft.angle=ax;shaft.angle_y=ay;
      const auto out=core.step(s,electrical(1.),1.,.001,true,Request::Start,shaft);
      const double expected=.1575-(.035+.1025*shaft_geometry(ax,ay,0.,0.).axis.z())+.010;
      require(out.fault==Fault::None && std::abs(out.approach_distance_limit_m-expected)<1e-10,
              "extended joystick approach did not follow the measured tip height");
      require(out.approach_distance_limit_m<=.060,"extended joystick exceeded descent budget");
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
         {Fault::Workspace, Fault::Period, Fault::Heartbeat}) {
      Core core{Config{}};
      auto s = state();
      core.activate(s);
      core.step(s, electrical(1.), 1., .001, true, Request::Ready);
      if (desired == Fault::Workspace) s.ee_position.x() = -.1;
      auto o = core.step(s, electrical(1.001), 1.001, desired == Fault::Period ? .02 : .001,
                         desired != Fault::Heartbeat, Request::None);
      require(o.fault == desired, "guard did not stop control");
    }
    // Ready and Hold may move the wrist more than the former 0.35 rad
    // posture-deviation bound, while retaining the actual joint limits.
    for (const double delta : {-.5, .5}) {
      for (const Request request : {Request::None, Request::Ready}) {
        Config cfg;
        Core core{cfg};
        auto s = state();
        core.activate(s);
        s.q[6] += delta;
        auto o = core.step(s, electrical(1.), 1., .001, true, request);
        require(o.fault == Fault::None,
                "wrist deviation from the captured posture still stops control");
        require(o.mode == (request == Request::Ready ? Mode::Ready : Mode::Hold),
                "wrist deviation interrupted the requested mode");
        s.q[6] = delta > 0. ? cfg.q_max[6] : cfg.q_min[6];
        o = core.step(s, electrical(1.001), 1.001, .001, true, Request::None);
        require(o.fault == Fault::JointLimit,
                "removing the wrist-posture bound disabled actual joint limits");
      }
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
      require(o.torque.isZero(0.) && !o.finished,
              "stop must request zero and await SDK-applied torque before finishing");
      s.previous_torque.setZero();
      o = core.step(s, electrical(1.001), 1.001, .001, true, Request::None);
      require(o.finished,"stop did not finish once the SDK-applied torque reached zero");
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

    // One isotropic spring saturates radially and retains the corresponding energy.
    for (const V3 extension : {V3(.001,0.,0.), V3(0.,.001,0.), V3(0.,0.,.001),
                               V3(.01,.02,-.03)}) {
      const double k=1000., cap=2., eps=1e-7;
      const auto spring=saturating_spring(extension,k,cap);
      require(std::abs(spring.force.norm()-std::min(k*extension.norm(),cap))<1e-10,
              "isotropic spring has different axis gains or an incorrect radial cap");
      for(int axis=0;axis<3;++axis) {
        V3 step=V3::Zero();step[axis]=eps;
        const double gradient=(saturating_spring(extension+step,k,cap).energy-
                               saturating_spring(extension-step,k,cap).energy)/(2.*eps);
        require(std::abs(gradient-spring.force[axis])<1e-6,
                "saturated spring force does not match its stored energy");
      }
    }
    {
      const auto I=Eigen::Matrix3d::Identity();
      const auto Q=Eigen::AngleAxisd(.3,V3::UnitY()).toRotationMatrix();
      const auto links=slider_alignment(V3::Zero(),I,V3::Zero(),V3::Zero(),V3(.03,.02,-.02),
          V3(.01,-.02,.03),Q,V3::Zero(),Eigen::Vector2d::Zero(),.08,1000.,20.,.05,1.);
      require(links.min_spring_scale<1. && links.force.head<2>().norm()<1e-10,
              "individual spring saturation broke the free slider tangential balance");
      const auto damping=slider_alignment(V3::Zero(),I,V3::Zero(),V3::Zero(),V3::Zero(),
          V3(0.,0.,1.),I,V3::Zero(),Eigen::Vector2d::Zero(),.08,1000.,20.,.05,1.);
      require(std::abs(damping.force.z()-80.)<1e-10 && damping.min_spring_scale==1.,
              "elastic saturation incorrectly capped the independent damper forces");
    }
    // X/Y directions must transform covariantly, including saturated alignment links.
    for(double theta : {-.6,.6}) {
      const auto shaft_x=shaft_geometry(theta,0.,.1,0.);
      const auto shaft_y=shaft_geometry(0.,theta,0.,.1);
      const Eigen::Matrix3d turn=Eigen::AngleAxisd(1.5707963267948966,V3::UnitZ()).toRotationMatrix();
      require((shaft_y.axis-turn*shaft_x.axis).norm()<1e-12 &&
              (shaft_y.axis_rate-turn*shaft_x.axis_rate).norm()<1e-12 &&
              (shaft_y.angular_velocity-turn*shaft_x.angular_velocity).norm()<1e-12 &&
              (shaft_y.rotation-turn*shaft_x.rotation*turn.transpose()).norm()<1e-12,
              "Base-Y shaft geometry is not the rotated Base-X geometry");
      const Eigen::Matrix3d R=(Eigen::AngleAxisd(.67,V3::UnitZ())*
          Eigen::AngleAxisd(3.141592653589793,V3::UnitX())).toRotationMatrix();
      const V3 face(.4,.02,.06),target(.403,.018,.05),velocity(.001,.002,-.003),omega(.01,-.02,.03);
      const Eigen::Vector2d slider(.025,-.01);
      const auto x=slider_alignment(face,R,velocity,omega,target,velocity,
          shaft_x.rotation*R,shaft_x.angular_velocity,slider,.08,1500.,20.,.05,20.);
      const auto y=slider_alignment(turn*face,turn*R,turn*velocity,turn*omega,
          turn*target,turn*velocity,turn*shaft_x.rotation*R,
          turn*shaft_x.angular_velocity,slider,.08,1500.,20.,.05,20.);
      require((y.force-turn*x.force).norm()<1e-10 && (y.moment-turn*x.moment).norm()<1e-10 &&
              (y.rate-x.rate).norm()<1e-10 && std::abs(y.min_spring_scale-x.min_spring_scale)<1e-12,
              "Y-direction transform changed the saturated alignment force/moment");
      const auto nominal=slider_alignment(V3::Zero(),R,V3::Zero(),V3::Zero(),V3::Zero(),
          V3::Zero(),shaft_y.rotation*R,V3::Zero(),Eigen::Vector2d::Zero(),.08,1500.,20.,.05,20.);
      require(nominal.moment.x()*theta<0.,"Y alignment couple rotates the palm in the wrong direction");
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
      const double x=.15, y=.26, vx=.12, vy=-.08, dt=1e-6;
      const auto shaft=shaft_geometry(x,y,vx,vy);
      const auto plus=shaft_geometry(x+dt*vx,y+dt*vy,vx,vy);
      const auto minus=shaft_geometry(x-dt*vx,y-dt*vy,vx,vy);
      require((shaft.axis.norm()-1.)<1e-12 &&
              ((plus.axis-minus.axis)/(2*dt)-shaft.axis_rate).norm()<1e-9,
              "two-axis shaft direction or rate is inconsistent");
      require((shaft.rotation*V3::UnitZ()-shaft.axis).norm()<1e-12,
              "palm target normal does not follow the 3D shaft");
      const auto arc=inward_arc(V3(.45,0.,.055),.02,.01,shaft);
      require((arc.position-(V3(.45,0.,.055)+.01*shaft.axis-.02*V3::UnitZ())).norm()<1e-12 &&
              (arc.velocity-.01*shaft.axis_rate).norm()<1e-12,
              "two-axis inward task arc is not absolute to the upright pivot");
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
    {
      Config cfg;cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      cfg.joystick_length=.02;cfg.joystick_inward_arc_m=.005;
      cfg.active_radius=.036;cfg.joystick_slider_radius_m=.05;
      cfg.validate();
    }
    // A real release ends the attempt, rather than chasing a snapback to upright.
    // Isolated EIT dropouts, slow returns and snapbacks near zero command do not.
    for (int scenario : {0,1,2,3,4,5}) {
      Config cfg;cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
      cfg.preload_time=.05;cfg.joystick_angle=.3490658503988659;
      // Isolate the snapback release detector from reported-contact geometry
      // checks while the shaft rotates without corresponding palm motion.
      cfg.joystick_slip_contact_mismatch_m=.10;
      Core core(cfg);auto s=state();core.activate(s);
      double now=1.;uint64_t seq=1;
      ShaftFeedback shaft;shaft.valid=true;
      shaft.angle=scenario==4 ? 0. : scenario==5 ? -.6 : .6;
      auto advance=[&](bool touching,Request request=Request::None) {
        shaft.acquired_steady_s=now;shaft.sequence=seq;
        auto out=core.step(s,electrical(now,seq++,touching),now,.001,true,request,shaft);
        now+=.001;return out;
      };
      advance(false,Request::Start);advance(true);
      s.ee_position.z()-=.00001;
      Output out;
      for(int i=0;i<70;++i)out=advance(true);
      require(out.mode==Mode::Interact,"release test did not reach manipulation");
      if(scenario==4){shaft.angle=.6;out=advance(true);}
      if(scenario==3) {
        for(int i=1;i<=250;++i){shaft.angle=.6*(1.-i/250.);out=advance(false);}
      } else if(scenario==2) {
        out=advance(false);
      } else {
        shaft.angle=0.;out=advance(scenario==1);
        if(scenario==0) {
          // A lagging EIT frame can confirm the same recent shaft snapback.
          require(out.mode==Mode::Retract,"combined snapback did not start retraction");
        }
        if(scenario==1) {
          require(out.mode==Mode::Interact && !out.trial_contact_released,
                  "shaft return without contact loss aborted the attempt");
          out=advance(false);
        }
      }
      const bool released=scenario==0 || scenario==1 || scenario==5;
      require(out.trial_contact_released==released && out.fault==Fault::None,
              "release detector classified an isolated dropout or normal motion incorrectly");
      require(out.mode==(released ? Mode::Retract : Mode::Interact),
              "contact release did not enter the expected recovery mode");
      if(released) {
        require(out.requested_moment.norm()<1e-8,
                "release still chased the upright shaft instead of capturing the palm pose");
        for(int i=0;i<2500 && out.mode==Mode::Retract;++i) {
          s.ee_position+=out.reference-out.pad_position;
          out=advance(false);
        }
        require(out.mode==Mode::Done && out.trial_contact_released &&
                    !out.retract_incomplete && out.fault==Fault::None,
                "release did not complete retraction while retaining its unsuccessful outcome");
        s=state();out=advance(false,Request::Start);
        require(out.mode==Mode::Approach && !out.trial_contact_released,
                "previous release contaminated the next trial");
      }
    }
    for (int bad : {0,1}) {
      Config cfg; cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      Core core(cfg);auto s=state();core.activate(s);
      ShaftFeedback shaft;shaft.sequence=1;shaft.valid=true;shaft.acquired_steady_s=1.;
      if(bad==0)shaft.valid=false;
      if(bad==1)shaft.acquired_steady_s=.5;
      auto o=core.step(s,electrical(1.),1.,.001,true,Request::Start,shaft);
      require(o.fault==Fault::JoystickFeedback,"new mode accepted unusable live angle feedback");
    }
    {
      Config cfg;cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      Core core(cfg);auto s=state();core.activate(s);
      ShaftFeedback shaft;shaft.sequence=1;shaft.valid=true;shaft.clipped=true;
      shaft.angle=-.6108652381980153;shaft.acquired_steady_s=1.;
      auto out=core.step(s,electrical(1.),1.,.001,true,Request::Start,shaft);
      require(out.fault==Fault::None && out.shaft_fresh && out.shaft_clipped,
              "ADC rail still stopped inward-arc control");
    }
    {
      Config cfg;cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
      cfg.preload_time=.05;
      cfg.joystick_task_spring_force_limit=1.;
      auto s=state();
      cfg.joystick_pivot_m=cfg.ready_pad_position-V3(0.,0.,cfg.joystick_length)+V3(.015,0.,0.);
      Config changed=cfg;changed.preload=6.;changed.orientation_k=25.;changed.orientation_d=3.;
      changed.coupling_k=100.;changed.coupling_d=3.;changed.normal_k=100.;changed.normal_d=3.;
      changed.force_xy_limit=.1;changed.normal_force_limit=10.;
      Config independently_limited=cfg;independently_limited.joystick_alignment_spring_force_limit=.1;
      Config task_only_cfg=cfg;task_only_cfg.joystick_alignment_enabled=false;
      Config shaft_task_cfg=task_only_cfg;shaft_task_cfg.joystick_task_spring_uses_shaft=true;
      Core core(cfg),other(changed),limited(independently_limited),task_only(task_only_cfg),
          shaft_task(shaft_task_cfg);
      core.activate(s);other.activate(s);limited.activate(s);task_only.activate(s);
      shaft_task.activate(s);
      double now=1.;uint64_t seq=1;
      ShaftFeedback shaft;shaft.valid=true;
      Output shaft_task_last;
      auto advance=[&](bool touching,Request request) {
        shaft.acquired_steady_s=now;shaft.sequence=seq;
        auto c=electrical(now,seq++,touching);
        auto a=core.step(s,c,now,.001,true,request,shaft);
        auto b=other.step(s,c,now,.001,true,request,shaft);
        auto independent=limited.step(s,c,now,.001,true,request,shaft);
        auto only=task_only.step(s,c,now,.001,true,request,shaft);
        auto direct=shaft_task.step(s,c,now,.001,true,request,shaft);
        shaft_task_last=direct;
        require((a.coupling_force-independent.coupling_force).norm()<1e-10,
                "alignment spring saturation changed the independent task force");
        require((a.requested_force-b.requested_force).norm()<1e-10 &&
                (a.requested_moment-b.requested_moment).norm()<1e-10,
                "legacy preload/orientation gains leaked into the inward mechanism");
        if ((only.mode==Mode::Preload || only.mode==Mode::Interact) && only.trial_time>0.) {
          require(only.fault==Fault::None &&
                  (only.coupling_force-a.coupling_force).norm()<1e-10 &&
                  (only.requested_force-only.coupling_force).norm()<1e-10 &&
                  only.requested_moment.norm()<1e-10 && only.alignment_force.norm()<1e-10 &&
                  only.slider.norm()<1e-10,
                  "target-only mode retained an alignment force, moment, or slider motion");
          const auto jt=point_jacobian(s.jacobian,only.point-s.ee_position);
          require((only.torque-jt.topRows<3>().transpose()*only.requested_force).norm()<1e-9,
                  "target-only mode applied torque beyond the task spring at its contact point");
          const double radius=cfg.joystick_length-cfg.joystick_inward_arc_m;
          const auto command=shaft_geometry(direct.angle_command,direct.angle_command_y,
                                            direct.progress_velocity,0.);
          const auto measured=shaft_geometry(shaft.angle,shaft.angle_y,shaft.rate,shaft.rate_y);
          const V3 desired=cfg.joystick_pivot_m+radius*command.axis;
          const V3 actual=cfg.joystick_pivot_m+radius*measured.axis;
          require((direct.reference-desired).norm()<1e-10 &&
                  direct.requested_moment.norm()<1e-10 && direct.alignment_force.norm()<1e-10,
                  "shaft task spring was not connected to the measured joystick and target");
          if (direct.mode==Mode::Interact && direct.trial_time>=cfg.preload_time) {
            V3 extension=desired-actual;
            extension.z()=only.reference.z()-only.point.z();
            const V3 expected=saturating_spring(extension,cfg.joystick_task_spring_k,
                cfg.joystick_task_spring_force_limit).force+
                cfg.joystick_task_spring_d*radius*command.axis_rate;
            require((direct.coupling_force-expected).norm()<1e-9 &&
                    (direct.requested_force-expected).norm()<1e-9,
                    "shaft spring did not combine angle error with vertical palm support");
          }
        }
        now+=.001;return a;
      };
      advance(false,Request::Start);advance(true,Request::None);
      s.ee_position.z()-=.00001;
      Output o;
      for(int i=0;i<70;++i)o=advance(true,Request::None);
      require(o.mode==Mode::Interact && o.fault==Fault::None,
              "new mechanism did not engage with fresh angle and contact");
      require(std::abs(o.requested_force.z())>cfg.normal_force_limit &&
              o.force_scale==1. && o.task_spring_scale<1. && o.alignment_force.z()<0.,
              "new VMC still applies a shared resultant-force cap");
      const V3 task_velocity=(cfg.joystick_length-cfg.joystick_inward_arc_m)*o.progress_velocity*
          V3(std::cos(o.angle_command),0.,-std::sin(o.angle_command));
      const auto elastic=saturating_spring(o.reference-o.point,cfg.joystick_task_spring_k,
                                          cfg.joystick_task_spring_force_limit);
      require((o.coupling_force-elastic.force-cfg.joystick_task_spring_d*task_velocity).norm()<1e-9,
              "task spring does not use one scalar stiffness and a separate scalar damper");
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
      require(shaft_task_last.coupling_force.x()<-.5,
              "measured shaft overshoot did not request a restoring task force");
      const V3 delta=(cfg.joystick_length-cfg.joystick_inward_arc_m)*
          V3(std::sin(.2),0.,std::cos(.2)-1.);
      require((o.alignment_target-previous-delta).norm()<1e-10,
              "alignment followed task command instead of current shaft angle");
      const V3 attachment=o.attachment;
      o=advance(false,Request::None);
      require(o.fault==Fault::None && o.mode==Mode::Interact &&
              (o.attachment-attachment).norm()<1e-12 && !o.sensor_contact,
              "inward-arc joystick stopped or moved attachment after contact loss");
      shaft.acquired_steady_s=now-.3;
      auto stale=core.step(s,electrical(now,seq,true),now,.001,true,Request::None,shaft);
      require(stale.fault==Fault::JoystickFeedback,"stale shaft feedback did not stop an active trial");
    }
    {
      Config cfg;cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      cfg.joystick_task_spring_uses_shaft=true;cfg.joystick_alignment_enabled=false;
      cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
      cfg.preload_time=.05;
      auto engage=[&](Core& core,Sample& s,double& now,uint64_t& sequence) {
        ShaftFeedback shaft;shaft.valid=true;
        auto step=[&](bool touching,Request request) {
          shaft.acquired_steady_s=now;shaft.sequence=sequence;
          auto out=core.step(s,electrical(now,sequence++,touching),now,.001,true,request,shaft);
          now+=.001;return out;
        };
        step(false,Request::Start);step(true,Request::None);
        s.ee_position.z()-=.00001;
        Output out;
        for(int i=0;i<70;++i)out=step(true,Request::None);
        require(out.mode==Mode::Interact,"shaft-task safety test did not reach manipulation");
      };
      auto s=state();Core slip(cfg);slip.activate(s);double now=1.;uint64_t sequence=1;
      engage(slip,s,now,sequence);
      ShaftFeedback shaft;shaft.valid=true;
      Output out;
      for(int i=0;i<90;++i) {
        shaft.acquired_steady_s=now;shaft.sequence=sequence;
        out=slip.step(s,electrical(now,sequence++,false),now,.001,true,Request::None,shaft);
        now+=.001;
      }
      require(out.mode==Mode::Interact && !out.joystick_slipped && out.fault==Fault::None,
              "an EIT dropout without robot/shaft disagreement was misclassified as slip");
      s.ee_position.x()+=.006;
      for(int i=0;i<80 && slip.mode()==Mode::Interact;++i) {
        shaft.acquired_steady_s=now;shaft.sequence=sequence;
        out=slip.step(s,electrical(now,sequence++,false),now,.001,true,Request::None,shaft);
        now+=.001;
      }
      require(out.mode==Mode::Interact && !out.trial_contact_released &&
              !out.joystick_slipped && out.fault==Fault::None,
              "no-contact mismatch without shaft bounce incorrectly failed the trial");

      Config pad_cfg=cfg;pad_cfg.joystick_task_spring_uses_shaft=false;
      auto pad_state=state();Core pad(pad_cfg);pad.activate(pad_state);
      double pad_now=1.;uint64_t pad_sequence=1;
      engage(pad,pad_state,pad_now,pad_sequence);
      shaft.angle=.3;
      for(int i=0;i<80;++i) {
        shaft.acquired_steady_s=pad_now;shaft.sequence=pad_sequence;
        out=pad.step(pad_state,electrical(pad_now,pad_sequence++,true),
                     pad_now,.001,true,Request::None,shaft);
        pad_now+=.001;
      }
      require(out.mode==Mode::Interact && !out.joystick_slipped,
              "shaft overshoot with reported contact was misclassified as palm runaway");
      shaft.angle=0.;
      pad_state.ee_position.x()+=.015;
      for(int i=0;i<80;++i) {
        shaft.acquired_steady_s=pad_now;shaft.sequence=pad_sequence;
        out=pad.step(pad_state,electrical(pad_now,pad_sequence++,true),
                     pad_now,.001,true,Request::None,shaft);
        pad_now+=.001;
      }
      require(out.mode==Mode::Interact && !out.joystick_slipped &&
              out.coupling_force.x()<0.,
              "pad spring failed to pull back a displaced palm or tripped too early");
      pad_state.ee_position.x()+=.010;
      for(int i=0;i<80 && pad.mode()==Mode::Interact;++i) {
        shaft.acquired_steady_s=pad_now;shaft.sequence=pad_sequence;
        out=pad.step(pad_state,electrical(pad_now,pad_sequence++,true),
                     pad_now,.001,true,Request::None,shaft);
        pad_now+=.001;
      }
      require(out.mode==Mode::Retract && out.trial_contact_released &&
              out.joystick_slipped && out.fault==Fault::None,
              "pad-task slip was masked by a true EIT contact signal");
    }
    {
      Config cfg;cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
      cfg.preload_time=.05;
      Core core(cfg);auto s=state();core.activate(s);
      double now=1.;uint64_t seq=1;
      ShaftFeedback shaft;shaft.valid=true;shaft.angle_y=.26;
      auto advance=[&](bool touching,Request request) {
        shaft.acquired_steady_s=now;shaft.sequence=seq;
        auto out=core.step(s,electrical(now,seq++,touching),now,.001,true,request,shaft);
        now+=.001;return out;
      };
      advance(false,Request::Start);advance(true,Request::None);
      s.ee_position.z()-=.00001;
      Output out;
      for (int i=0;i<5;++i)out=advance(true,Request::None);
      const auto initial=shaft_geometry(0.,.26,0.,0.);
      require(out.mode==Mode::Preload && out.fault==Fault::None &&
              std::abs(out.angle_command_y-.26)<1e-12 &&
              (out.reference-(out.point-cfg.joystick_inward_arc_m*initial.axis)).norm()<1e-8,
              "tilted start was incorrectly rebased to upright");
      require((out.desired_pad_normal+initial.axis).norm()<1e-8,
              "initial palm target is not perpendicular to the tilted shaft");
      for (int i=0;i<200;++i)out=advance(true,Request::None);
      require(out.mode==Mode::Interact && out.angle_command_y<.26 &&
              out.angle_command>0.,
              "task did not drive Base Y toward zero and Base X toward its absolute goal");
    }
    // Each mechanism traverses both signs, returns smoothly to zero, then retracts.
    for (auto mechanism : {JoystickMechanism::Legacy, JoystickMechanism::InwardArc}) {
      for (auto axis : {JoystickAxis::X, JoystickAxis::Y}) {
        Config cfg;cfg.joystick_mechanism=mechanism;cfg.joystick_axis=axis;
        cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
        cfg.preload_time=.55;cfg.joystick_duration=19.;cfg.joystick_return_zero_start_s=13.;
        cfg.joystick_angle=20.*3.141592653589793/180.;cfg.angle_speed=.15;
        Core core(cfg);auto s=state();core.activate(s);
        const V3 initial_point=s.ee_position+s.ee_rotation*cfg.ee_to_pad+
                               s.ee_rotation*V3(.01,.02,0.);
        double now=1.;uint64_t seq=1;ShaftFeedback shaft;shaft.valid=true;
        const bool along_y=axis==JoystickAxis::Y;
        if(mechanism==JoystickMechanism::InwardArc) {
          shaft.angle=along_y ? -.08 : .12;
          shaft.angle_y=along_y ? .12 : -.08;
        }
        auto advance=[&](bool touching,Request request=Request::None) {
          shaft.acquired_steady_s=now;shaft.sequence=seq;
          auto out=core.step(s,electrical(now,seq++,touching),now,.001,true,request,shaft);
          now+=.001;return out;
        };
        require(core.set_joystick_axis(axis),"holding controller rejected axis selection");
        advance(false,Request::Start);advance(true);
        require(!core.set_joystick_axis(along_y ? JoystickAxis::X : JoystickAxis::Y),
                "axis changed during an active trial");
        s.ee_position.z()-=.00001;
        Output out;bool positive=false,negative=false,returned=false;
        double previous_command=along_y ? shaft.angle_y : shaft.angle;
        for(int i=0;i<19010;++i) {
          out=advance(true);
          require(out.fault==Fault::None,"selected-axis trajectory faulted");
          if(out.mode!=Mode::Interact)continue;
          const double command=along_y ? out.angle_command_y : out.angle_command;
          const double cross_command=along_y ? out.angle_command : out.angle_command_y;
          require(std::abs(command-previous_command)<=cfg.angle_speed*.001+1e-9 || out.trial_time<1.,
                  "return-to-zero reference jumped instead of using the existing trajectory filter");
          previous_command=command;
          if(out.trial_time>18.9) {
            returned=true;
            require(std::abs(command)<.001 && std::abs(cross_command)<.001,
                    "reference did not return to zero before retract");
          }
          require(out.joystick_axis==axis && std::abs(out.progress-command)<1e-12,
                  "axis or selected-angle telemetry incorrect");
          if(out.trial_time>3. && out.trial_time<4.) {
            positive|=command>.15;
            require(std::abs(cross_command)<.001,"unselected axis did not return to zero");
          }
          if(out.trial_time>7.)negative|=command<-.15;
          if(mechanism==JoystickMechanism::Legacy && out.trial_time>3. && out.trial_time<4.) {
            const V3 displacement=out.reference-initial_point;
            require(displacement[along_y ? 1 : 0]>.01 &&
                    std::abs(displacement[along_y ? 0 : 1])<1e-12,
                    "legacy arc used the wrong displacement axis");
            const V3 expected(std::sin(out.angle_command),std::sin(out.angle_command_y),std::cos(command));
            require((out.desired_pad_normal+expected).norm()<1e-9,
                    "legacy palm rotation has the wrong axis/sign");
          }
          if(mechanism==JoystickMechanism::InwardArc) {
            const auto measured=shaft_geometry(shaft.angle,shaft.angle_y,0.,0.);
            require((out.desired_pad_normal+measured.axis).norm()<1e-9,
                    "axis selection changed measured two-axis alignment");
          }
        }
        require(positive && negative && returned && out.mode==Mode::Retract,
                "selected-axis trajectory did not traverse both signs, return to zero and retract");
      }
    }
    // Even prolonged no-contact readings with palm mismatch do not establish
    // release without a shaft bounce. The EIT attachment holds throughout.
    for(bool eit : {false,true}) {
      Config cfg;cfg.joystick_mechanism=JoystickMechanism::InwardArc;
      cfg.joystick_alignment_enabled=false;
      cfg.preload_time=.55;cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
      auto s=state();Core core(cfg);core.activate(s);core.set_condition(eit);
      double now=1.;uint64_t sequence=1;ShaftFeedback shaft;shaft.valid=true;
      V3 observation(.01,.02,0.);
      auto advance=[&](bool touching,Request req=Request::None) {
        shaft.acquired_steady_s=now;shaft.sequence=sequence;
        auto contact=electrical(now,sequence++,touching);
        contact.x=touching ? observation.x() : 0.;contact.y=touching ? observation.y() : 0.;
        contact.position_valid=touching;
        auto out=core.step(s,contact,now,.001,true,req,shaft);
        now+=.001;return out;
      };
      advance(false,Request::Start);advance(true);s.ee_position.z()-=.00001;
      auto out=advance(true);require(out.mode==Mode::Preload,"dropout test did not engage");
      const V3 initial_attachment=out.attachment;
      observation=V3(.015,.025,0.);
      for(int i=0;i<50;++i)out=advance(true);
      require(!eit || (out.attachment-initial_attachment).norm()>1e-6,
              "valid EIT update never changed the attachment before dropout");
      s.ee_position.x()+=.006;V3 captured=out.attachment;
      for(int i=0;i<250;++i) {
        out=advance(false);
        require(out.mode==Mode::Preload && !out.joystick_slipped &&
                (out.attachment-captured).norm()<1e-12,
                "brief preload dropout stopped trial or changed attachment");
      }
      out=advance(true);require(out.fault==Fault::None,"contact recovery faulted");
      captured=out.attachment;
      // Continue well beyond the former 300 ms grace, across preload into
      // interaction, with fresh sensor frames and no shaft bounce.
      for(int i=0;i<2000;++i) {
        out=advance(false);
        require(!out.joystick_slipped && out.fault==Fault::None &&
                (out.attachment-captured).norm()<1e-12,
                "prolonged no-contact report stopped the trial or changed attachment");
      }
      require(out.mode==Mode::Interact && std::abs(out.angle_command)>.1 &&
              out.coupling_force.norm()>.1 && !out.trial_contact_released,
              "task spring or trajectory stopped during prolonged detector dropout");
      out=advance(true);
      require(out.mode==Mode::Interact && out.fault==Fault::None,
              "fresh contact recovery incorrectly failed the trial");
    }
    // Kinematic reference replaces only the captured contact point; EIT still detects contact.
    for(auto mechanism : {JoystickMechanism::Legacy,JoystickMechanism::InwardArc}) {
      Config cfg;cfg.joystick_mechanism=mechanism;
      cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;cfg.preload_time=.05;
      cfg.ready_pad_rotation=(Eigen::AngleAxisd(.67,V3::UnitZ())*
          Eigen::AngleAxisd(3.141592653589793,V3::UnitX())).toRotationMatrix();
      cfg.ee_R_pad=Eigen::AngleAxisd(-1.5707963267948966,V3::UnitZ()).toRotationMatrix();
      auto s=state();s.ee_rotation=cfg.ready_pad_rotation*cfg.ee_R_pad.transpose();
      ShaftFeedback shaft;shaft.valid=true;shaft.angle=.1;shaft.angle_y=-.08;
      const V3 axis=shaft_geometry(shaft.angle,shaft.angle_y,0.,0.).axis;
      const V3 face=s.ee_position+s.ee_rotation*cfg.ee_to_pad;
      cfg.joystick_pivot_m=face+cfg.ready_pad_rotation*V3(.012,-.018,0.)-cfg.joystick_length*axis;
      Core core(cfg);core.activate(s);
      require(core.set_condition(ContactSource::Kinematic),"kinematic condition rejected in Hold");
      double now=1.;uint64_t sequence=1;
      auto advance=[&](bool touching,Request request=Request::None) {
        shaft.acquired_steady_s=now;shaft.sequence=sequence;
        auto contact=electrical(now,sequence++,touching);
        contact.position_valid=false;contact.x=.04;contact.y=.04;
        auto out=core.step(s,contact,now,.001,true,request,shaft);now+=.001;return out;
      };
      advance(false,Request::Start);advance(true);s.ee_position.z()-=.00001;
      Output out;for(int i=0;i<80;++i)out=advance(true);
      V3 expected=cfg.ready_pad_rotation.transpose()*(cfg.joystick_pivot_m+cfg.joystick_length*axis-
          s.ee_position-s.ee_rotation*cfg.ee_to_pad);expected.z()=0.;
      require(out.mode==Mode::Interact && out.fault==Fault::None && !out.use_eit &&
                  out.contact_source==ContactSource::Kinematic && (out.attachment-expected).norm()<1e-10,
              "kinematic attachment did not use the measured tip projected into the yawed palm frame");
      const V3 captured=out.attachment;shaft.angle_y+=.1;out=advance(true);
      require((out.attachment-captured).norm()<1e-12 &&
                  (out.joystick_tip_position-cfg.joystick_pivot_m-cfg.joystick_length*
                      shaft_geometry(shaft.angle,shaft.angle_y,0.,0.).axis).norm()<1e-12,
              "kinematic attachment moved after capture or tip diagnostics failed to update");
      require(!core.set_condition(ContactSource::Eit),"contact source changed during motion");
      shaft.acquired_steady_s=now-.3;
      out=core.step(s,electrical(now,sequence,true),now,.001,true,Request::None,shaft);
      require(out.fault==Fault::JoystickFeedback,"kinematic condition accepted stale shaft feedback");
    }
    {
      Config cfg;cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
      cfg.joystick_pivot_m=V3(.65,0.,.35-cfg.joystick_length);
      Core core(cfg);auto s=state();core.activate(s);core.set_condition(ContactSource::Kinematic);
      ShaftFeedback shaft;shaft.valid=true;double now=1.;uint64_t seq=1;
      auto advance=[&](bool touching,Request request=Request::None) {
        shaft.acquired_steady_s=now;shaft.sequence=seq;
        auto out=core.step(s,electrical(now,seq++,touching),now,.001,true,request,shaft);now+=.001;return out;
      };
      advance(false,Request::Start);advance(true);s.ee_position.z()-=.00001;
      Output out;for(int i=0;i<5;++i)out=advance(true);
      require(out.fault==Fault::SensorPosition,"kinematic tip outside the active palm was accepted");
      cfg.task=Task::Ball;Core ball(cfg);
      require(!ball.set_condition(ContactSource::Kinematic),"ball accepted joystick kinematic contact source");
    }
    // A ball sliding with the pad uses 1:1 baseline motion. The previous
    // rolling ratio remains explicit for interpreting historical recordings.
    for(double ratio : {1.,.5}) {
      Config cfg;cfg.task=Task::Ball;cfg.ball_axis=BallAxis::Y;
      cfg.ball_baseline_motion_ratio=ratio;cfg.roll_distance=-.02;
      cfg.ee_to_pad.setZero();cfg.ready_pad_position=V3(.45,-.10,.49);
      cfg.preload_time=.05;cfg.ball_contact_settle_s=.03;
      cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
      auto s=state();s.ee_position.y()=-.10;
      Core core(cfg);core.activate(s);core.set_condition(false);
      double now=1.;uint64_t sequence=1;
      auto advance=[&](bool touching,Request request=Request::None) {
        auto out=core.step(s,electrical(now,sequence++,touching),now,.001,true,request);
        now+=.001;return out;
      };
      advance(false,Request::Start);advance(true);s.ee_position.z()-=.00001;
      Output out;
      for(int i=0;i<1000;++i) {
        out=advance(true);
        if(out.mode==Mode::Interact)break;
      }
      require(out.mode==Mode::Interact && out.fault==Fault::None,
              "baseline motion-ratio trial did not engage");
      s.ee_position.y()-=.01;
      for(int i=0;i<30;++i)out=advance(true);
      require(out.mode==Mode::Interact && out.fault==Fault::None &&
                  std::abs(out.progress-(-.10-.01*ratio))<1e-4,
              "baseline motion estimate does not follow its configured ratio");
    }
    // Direct ball feedback and force must follow either Base axis in either direction.
    // Both feedback conditions stop on their own estimate at the same signed ball goal;
    // an unmet goal ends at the common duration and is not labeled as reached.
    for(bool eit : {false,true}) {
      for(bool move_to_goal : {false,true}) {
        Config cfg;cfg.task=Task::Ball;cfg.ball_axis=BallAxis::Y;
        cfg.ee_to_pad.setZero();cfg.ready_pad_position=V3(.45,-.10,.49);
        cfg.roll_distance=-.010;cfg.ball_duration=.75;
        cfg.preload_time=.05;cfg.ball_contact_settle_s=.03;
        cfg.ball_goal_tolerance_m=.001;cfg.ball_goal_dwell_s=.02;
        cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;
        auto s=state();s.ee_position.y()=-.10;
        Core core(cfg);core.activate(s);core.set_condition(eit);
        double now=1.;uint64_t sequence=1;
        auto advance=[&](bool touching,Request request=Request::None) {
          auto contact=electrical(now,sequence++,touching);
          contact.x=0.;contact.y=.5*(s.ee_position.y()+.10);
          auto out=core.step(s,contact,now,.001,true,request);now+=.001;return out;
        };
        Output out=advance(false,Request::Start);
        out=advance(true);s.ee_position.z()-=.00001;
        bool saw_ramp=false,saw_fixed_goal=false,saw_extension=false;
        for(int i=0;i<1000 && out.mode!=Mode::Retract && out.mode!=Mode::Fault;++i) {
          if(move_to_goal && out.mode==Mode::Interact && s.ee_position.y()>-.122)
            s.ee_position.y()-=.0002;
          const double prior_reference=out.ball_drive_reference;
          out=advance(true);
          if(out.mode==Mode::Interact && out.ball_contact_ready) {
            const double displacement=prior_reference-out.ball_drive_reference;
            require(displacement>=-1e-12 &&
                        displacement<=cfg.ball_reference_speed_m_s*.001+1e-9,
                    "ball reference jumped instead of moving at bounded constant speed");
            if(out.ball_drive_reference>out.goal+1e-5) saw_ramp=true;
            if(std::abs(out.ball_drive_reference-out.goal)<1e-8) saw_fixed_goal=true;
            if(out.ball_drive_reference<out.goal-1e-5) saw_extension=true;
          }
        }
        require(out.mode==Mode::Retract && out.fault==Fault::None &&
                    out.ball_goal_reached==move_to_goal,
                "ball condition mislabeled estimated goal or time-limit completion");
        require(move_to_goal ? out.trial_time<cfg.ball_duration :
                              out.trial_time>=cfg.ball_duration,
                "ball goal and time-limit stops were not distinguished");
        if(!move_to_goal)
          require(saw_ramp && saw_fixed_goal && saw_extension &&
                      out.ball_drive_reference<out.goal-.0005 &&
                      out.ball_drive_reference>=out.goal-cfg.ball_reference_advance_limit_m,
                  "matched ball condition did not ramp to goal then extend while stalled");
      }
    }
    for(auto axis : {BallAxis::X,BallAxis::Y}) {
      for(double sign : {-1.,1.}) {
        for(bool eit : {false,true}) {
          Config cfg;cfg.task=Task::Ball;cfg.ball_axis=axis;cfg.roll_distance=sign*.10;
          cfg.ready_pad_position.y()=-.10;cfg.workspace_min.y()=-.32;
          cfg.approach_contact_confirm_s=.001;cfg.approach_contact_travel_m=.000001;cfg.preload_time=.05;
          auto s=state();s.ee_position.y()=-.10;
          const int index=axis==BallAxis::Y ? 1 : 0;
          const double initial=index==1 ? cfg.initial_ball_y : cfg.initial_ball_x;
          const V3 initial_face=s.ee_position+s.ee_rotation*cfg.ee_to_pad;
          Core core(cfg);core.activate(s);core.set_condition(eit);
          double now=1.;uint64_t sequence=1;
          auto advance=[&](bool touching,Request request=Request::None) {
            auto contact=electrical(now,sequence++,touching);
            const V3 face=s.ee_position+s.ee_rotation*cfg.ee_to_pad;
            V3 ball=face;ball[index]=initial+.5*(face[index]-initial_face[index]);
            const V3 local=s.ee_rotation.transpose()*(ball-face);
            contact.x=local.x();contact.y=local.y();
            auto out=core.step(s,contact,now,.001,true,request);now+=.001;return out;
          };
          advance(false,Request::Start);advance(true);s.ee_position.z()-=.00001;
          Output out;
          for(int i=0;i<1000;++i) {
            out=advance(true);
            require(out.fault==Fault::None,"selected ball axis/direction faulted");
            if(out.mode==Mode::Preload || out.mode==Mode::Interact)
              s.ee_position[index]+=std::clamp(out.requested_force[index]*.00001,-.00001,.00001);
          }
          require(out.mode==Mode::Interact && out.ball_axis==axis &&
                      std::abs(out.goal-initial-cfg.roll_distance)<1e-12 &&
                      sign*(out.progress-initial)>.001 &&
                      sign*(out.ball_drive_reference-initial)>0. &&
                      std::abs(out.requested_force[index])<=
                          std::min(cfg.goal_force_limit,cfg.force_xy_limit)+1e-12,
                  "ball ramp, observed travel, or lateral force bound is wrong");
          require(std::abs(out.reference[1-index]-initial_face[1-index])<1e-12,
                  "ball motion changed the unselected Base axis");
        }
      }
    }
    {
      Config cfg;cfg.task=Task::Ball;cfg.ball_axis=BallAxis::Y;cfg.ready_pad_position.y()=-.10;
      cfg.roll_distance=-.10;
      cfg.workspace_min.y()=-.18;bool rejected=false;
      try{cfg.validate();}catch(const std::invalid_argument&){rejected=true;}
      require(rejected,"sliding ball goal outside workspace was accepted");
      cfg.workspace_min.y()=-.25;cfg.validate();
      cfg.ball_baseline_motion_ratio=.5;rejected=false;
      try{cfg.validate();}catch(const std::invalid_argument&){rejected=true;}
      require(rejected,"historical rolling ratio outside workspace was accepted");
      cfg.workspace_min.y()=-.32;cfg.validate();
    }
    std::cout << "Core geometry, matched-condition, rebase and fault tests passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
