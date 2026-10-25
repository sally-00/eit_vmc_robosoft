#include "franka_eit_vmc/joystick_mechanism.hpp"
#include <cmath>

namespace franka_eit_vmc {
ArcReference inward_arc(const V3& anchor, double length, double depth, double theta, double rate) {
  const double radius = length - depth;
  return {anchor + V3(radius * std::sin(theta), 0., radius * std::cos(theta) - length),
          radius * rate * V3(std::cos(theta), 0., -std::sin(theta))};
}
SliderAlignment slider_alignment(const V3& face, const Eigen::Matrix3d& R,
    const V3& face_velocity, const V3& omega, const V3& target, const V3& target_velocity,
    const Eigen::Matrix3d& Q, const V3& target_omega, const Eigen::Vector2d& slider,
    double a, double k, double d, double radius) {
  SliderAlignment out;
  const V3 arm = R * V3(slider.x(), slider.y(), 0.);
  out.centre = face + arm;
  const V3 material_velocity = face_velocity + omega.cross(arm);
  out.rate = R.leftCols<2>().transpose() *
      (target_velocity - material_velocity + (k / d) * (target - out.centre));
  const double norm = slider.norm();
  if (norm >= radius - 1e-10 && norm > 0. && slider.dot(out.rate) > 0.) {
    out.rate -= slider * slider.dot(out.rate) / (norm * norm);
    out.at_stop = true;
  }
  const V3 centre_velocity = material_velocity + R.leftCols<2>() * out.rate;
  out.force.setZero(); out.moment.setZero();
  for (const V3& local : {V3(a,0.,0.), V3(-a,0.,0.), V3(0.,a,0.), V3(0.,-a,0.)}) {
    const V3 actual_arm = R * local, target_arm = Q * local;
    const V3 extension = target + target_arm - out.centre - actual_arm;
    const V3 relative_velocity = target_velocity + target_omega.cross(target_arm) -
                                centre_velocity - omega.cross(actual_arm);
    const V3 force = k * extension + d * relative_velocity;
    out.force += force;
    out.moment += actual_arm.cross(force);
    out.energy += .5 * k * extension.squaredNorm();
  }
  return out;
}
}  // namespace franka_eit_vmc
