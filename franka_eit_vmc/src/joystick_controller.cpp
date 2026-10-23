#include "franka_eit_vmc/runner.hpp"
// EIT/centre attachment at contact; identical prescribed arc and orientation gains.
int main(int argc, char** argv) {
  return franka_eit_vmc::run_controller(argc, argv, franka_eit_vmc::Task::Joystick);
}
