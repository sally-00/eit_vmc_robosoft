#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <franka/lowpass_filter.h>
#include <franka/rate_limiting.h>

#include "franka_eit_vmc/core.hpp"

using namespace franka_eit_vmc;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

// Apply the actual SDK functions in the order used by ControlLoop::spinControl.
V7 sdk_step(const V7& request, const V7& previous) {
  std::array<double, 7> filtered{}, old{};
  for (int i = 0; i < 7; ++i) {
    old[i] = previous[i];
    filtered[i] = franka::lowpassFilter(.001, request[i], previous[i],
                                      franka::kDefaultCutoffFrequency);
  }
  const auto applied = franka::limitRate(franka::kMaxTorqueRate, filtered, old);
  V7 result;
  for (int i = 0; i < 7; ++i) {
    require(std::abs(applied[i] - previous[i]) <= franka::kMaxTorqueRate[i] * .001 + 1e-12,
            "SDK torque-rate protection was exceeded");
    result[i] = applied[i];
  }
  return result;
}

Sample sample() {
  Sample s;
  s.q << 0., -.785, 0., -2.356, 0., 1.57, .785;
  s.ee_position << .45, 0., .49;
  s.ee_rotation = Eigen::AngleAxisd(3.141592653589793, V3::UnitX()).toRotationMatrix();
  s.jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  return s;
}

int main() {
  try {
    Core core{Config{}};
    auto s = sample();core.activate(s);
    double now = 1.;
    auto advance = [&](Request request = Request::None) {
      const auto out = core.step(s, Contact{}, now, .001, true, request);
      now += .001;
      if (!out.finished) s.previous_torque = sdk_step(out.torque, s.previous_torque);
      return out;
    };
    // A synthetic bias demand isolates the command path from the robot dynamics.
    s.coriolis.setConstant(10.);
    for (int i = 0; i < 25; ++i) {
      const auto out = advance();
      require((out.torque - s.coriolis).norm() < 1e-12,
              "normal torque was limited again before the SDK");
    }
    require(s.previous_torque.minCoeff() > 9.9,
            "torque step retained the former slow package-limit/filter feedback");
    s.coriolis.setConstant(-10.);
    for (int i = 0; i < 35; ++i) {
      const auto out = advance();
      require((out.torque - s.coriolis).norm() < 1e-12,
              "torque reversal was delayed before reaching the SDK");
    }
    require(s.previous_torque.maxCoeff() < -9.9, "SDK torque reversal did not settle");

    for (bool fault : {false, true}) {
      core.activate(s);
      s.previous_torque.setConstant(8.);
      s.dq.setZero();
      if (fault) s.dq[0] = 1.;
      auto out = advance(fault ? Request::None : Request::Stop);
      require(out.mode == (fault ? Mode::Fault : Mode::Stopping) &&
                  out.torque.isZero(0.) && !out.finished,
              "stop/fault finished before the SDK-applied torque ramped down");
      int steps = 1;
      while (!out.finished && steps < 100) {out = advance();++steps;}
      require(out.finished && s.previous_torque.cwiseAbs().maxCoeff() < 1e-4,
              "stop/fault did not wait for the filtered applied torque to reach zero");
    }
    std::cout << "SDK torque step, reversal and bounded stop/fault tests passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
