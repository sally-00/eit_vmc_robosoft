#pragma once
#include "franka_eit_vmc/core.hpp"
#include <algorithm>
#include <limits>
namespace franka_eit_vmc {
struct BallNormalStep {
  double position, velocity, force, acceleration, used_velocity;
  bool at_stop;
};
// Upward-positive, semi-implicit mass integration. Contact blocks descent only.
// The reaction is opposite the bounded virtual command, never measured load.
inline BallNormalStep ball_normal_step(const Config& c, double z, double v,
    double point_z, double point_v, bool brake, bool valid, double dt, double origin,
    double workspace_floor = -std::numeric_limits<double>::infinity(),
    double force_limit = -1., double virtual_preload = -1.) {
  if (!valid) return {z, 0., 0., 0., 0., false};
  if (brake) v = std::max(0., v);
  const double limit = force_limit >= 0. ? force_limit : c.normal_force_limit;
  const double preload = virtual_preload >= 0. ? virtual_preload : c.preload;
  const double force = std::clamp(c.normal_k*(z-point_z) + c.normal_d*(v-point_v),
                                  -limit, 0.);
  // The commanded spring force is supported only while contact is present.
  // During recovery an unloaded pad must not lift the virtual carriage.
  const double acc = (-preload - c.ball_normal_damping*v - (brake ? force : 0.)) /
                     c.ball_normal_mass;
  double next_v = std::clamp(v+dt*acc, -c.ball_normal_speed, c.ball_normal_speed);
  if (brake) next_v = std::max(0., next_v);
  const double trial = z+dt*next_v;
  const double next_z = std::clamp(trial,
      std::max(origin-c.ball_normal_travel,workspace_floor),origin+c.ball_normal_up_travel);
  const bool stop = next_z != trial;
  if (stop) next_v = 0.;
  return {next_z, next_v, force, acc, v, stop};
}
}  // namespace franka_eit_vmc

namespace franka_eit_vmc {
inline double ball_drive_force(const Config& c, double goal, double estimate, double estimate_v,
    double point, double velocity, double hold, double age, bool driving, double previous,
    double dt) {
  const double u = std::clamp(age / c.ball_goal_ramp_s, 0., 1.);
  const double desired = driving
      ? u*u*(3.-2.*u)*(c.goal_k*(goal-estimate)-c.ball_goal_d*estimate_v)
      : c.ball_pause_k*(hold-point)-c.ball_pause_d*velocity;
  const double cap = std::min(c.goal_force_limit, c.force_xy_limit);
  const double slew = driving ? c.ball_force_slew : c.ball_pause_force_slew;
  return previous + std::clamp(std::clamp(desired,-cap,cap)-previous,
                              -slew*dt,slew*dt);
}
}
