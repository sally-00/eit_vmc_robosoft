#include "franka_eit_vmc/runner.hpp"
// EIT/robot-motion progress feedback; centre attachment in the proof-of-concept comparison.
int main(int argc, char** argv) {
  return franka_eit_vmc::run_controller(argc, argv, franka_eit_vmc::Task::Ball);
}
