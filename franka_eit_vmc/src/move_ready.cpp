#include "franka_eit_vmc/runner.hpp"
// Finite Cartesian ready motion, preserving the captured/saved joint posture in null space.
int main(int argc, char** argv) {
  return franka_eit_vmc::run_controller(argc, argv, franka_eit_vmc::Task::Ready);
}
