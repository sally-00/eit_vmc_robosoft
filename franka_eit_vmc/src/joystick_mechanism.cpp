#include "franka_eit_vmc/joystick_mechanism.hpp"
#include <cmath>

namespace franka_eit_vmc {
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
