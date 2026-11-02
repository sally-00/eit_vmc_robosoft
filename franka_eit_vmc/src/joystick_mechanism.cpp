#include "franka_eit_vmc/joystick_mechanism.hpp"
#include <algorithm>
#include <cmath>

namespace franka_eit_vmc {
SpringForce saturating_spring(const V3& extension, double stiffness, double force_limit) {
  SpringForce out;
  const double length = extension.norm();
  const double elastic_norm = stiffness * length;
  out.scale = elastic_norm > force_limit ? force_limit / elastic_norm : 1.;
  out.force = out.scale * stiffness * extension;
  out.energy = elastic_norm > force_limit
      ? force_limit * (length - .5 * force_limit / stiffness)
      : .5 * stiffness * length * length;
  return out;
}
ShaftGeometry shaft_geometry(double angle_x, double angle_y, double rate_x, double rate_y) {
  const V3 unnormalised(std::tan(angle_x), std::tan(angle_y), 1.);
  const double length = unnormalised.norm();
  const V3 axis = unnormalised / length;
  const V3 unnormalised_rate(rate_x / std::pow(std::cos(angle_x), 2),
                             rate_y / std::pow(std::cos(angle_y), 2), 0.);
  const V3 axis_rate = (unnormalised_rate - axis * axis.dot(unnormalised_rate)) / length;
  const V3 angular_velocity = axis.cross(axis_rate);
  const Eigen::Matrix3d rotation =
      Eigen::Quaterniond::FromTwoVectors(V3::UnitZ(), axis).toRotationMatrix();
  return {axis, axis_rate, angular_velocity, rotation};
}
ArcReference inward_arc(const V3& anchor, double length, double depth, const ShaftGeometry& shaft) {
  const double radius = length - depth;
  return {anchor + radius * shaft.axis - length * V3::UnitZ(), radius * shaft.axis_rate};
}
ArcReference inward_arc(const V3& anchor, double length, double depth, double theta, double rate) {
  const double radius = length - depth;
  return {anchor + V3(radius * std::sin(theta), 0., radius * std::cos(theta) - length),
          radius * rate * V3(std::cos(theta), 0., -std::sin(theta))};
}
SliderAlignment slider_alignment(const V3& face, const Eigen::Matrix3d& R,
    const V3& face_velocity, const V3& omega, const V3& target, const V3& target_velocity,
    const Eigen::Matrix3d& Q, const V3& target_omega, const Eigen::Vector2d& slider,
    double a, double k, double d, double radius, double spring_force_limit) {
  SliderAlignment out;
  const V3 arm = R * V3(slider.x(), slider.y(), 0.);
  out.centre = face + arm;
  const V3 material_velocity = face_velocity + omega.cross(arm);
  const std::array<V3,4> local{V3(a,0.,0.), V3(-a,0.,0.), V3(0.,a,0.), V3(0.,-a,0.)};
  std::array<V3,4> actual_arms, target_arms, elastic;
  V3 elastic_sum = V3::Zero();
  for (size_t i=0;i<local.size();++i) {
    actual_arms[i] = R * local[i];
    target_arms[i] = Q * local[i];
    const auto spring = saturating_spring(
        target + target_arms[i] - out.centre - actual_arms[i], k, spring_force_limit);
    elastic[i] = spring.force;
    elastic_sum += spring.force;
    out.energy += spring.energy;
    out.min_spring_scale = std::min(out.min_spring_scale, spring.scale);
  }
  // Saturated springs no longer sum to 4*k*centre_error. Solve tangential
  // balance using their actual forces; the four linear dampers contribute 4*d.
  out.rate = R.leftCols<2>().transpose() *
      (target_velocity - material_velocity + elastic_sum / (4. * d));
  const double norm = slider.norm();
  if (norm >= radius - 1e-10 && norm > 0. && slider.dot(out.rate) > 0.) {
    out.rate -= slider * slider.dot(out.rate) / (norm * norm);
    out.at_stop = true;
  }
  const V3 centre_velocity = material_velocity + R.leftCols<2>() * out.rate;
  out.force.setZero(); out.moment.setZero();
  for (size_t i=0;i<local.size();++i) {
    const V3 relative_velocity = target_velocity + target_omega.cross(target_arms[i]) -
                                centre_velocity - omega.cross(actual_arms[i]);
    const V3 force = elastic[i] + d * relative_velocity;
    out.force += force;
    out.moment += actual_arms[i].cross(force);
  }
  return out;
}
}  // namespace franka_eit_vmc
