#pragma once
#include "franka_eit_vmc/core.hpp"

namespace franka_eit_vmc {
struct ArcReference { V3 position; V3 velocity; };
ArcReference inward_arc(const V3& upright_surface, double physical_length, double depth,
                        double angle, double rate);
struct SliderAlignment {
  V3 centre, force, moment;
  Eigen::Vector2d rate;
  double energy{0.};
  bool at_stop{false};
};
// Four symmetric links share ONE massless, frictionless planar guide. Normal
// translation and orientation are constrained to the palm; tangential force
// balance determines guide velocity. All quantities are in the robot Base frame.
SliderAlignment slider_alignment(const V3& face, const Eigen::Matrix3d& palm_rotation,
    const V3& face_velocity, const V3& omega, const V3& target, const V3& target_velocity,
    const Eigen::Matrix3d& target_rotation, const V3& target_omega,
    const Eigen::Vector2d& slider, double half_span, double spring_k, double spring_d,
    double radius);
}  // namespace franka_eit_vmc
