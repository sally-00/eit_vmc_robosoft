#include "franka_eit_vmc/runner.hpp"
// EIT/centre task attachment; legacy or measured-shaft inward-arc mechanism.
int main(int argc, char** argv) {
  return franka_eit_vmc::run_controller(argc, argv, franka_eit_vmc::Task::Joystick);
}
