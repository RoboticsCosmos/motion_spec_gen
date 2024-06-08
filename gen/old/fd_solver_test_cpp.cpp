extern "C"
{
#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"
}
#include <array>
#include <controllers/pid_controller.hpp>
#include <filesystem>
#include <iostream>
#include <kinova_mediator/mediator.hpp>
#include <motion_spec_utils/math_utils.hpp>
#include <motion_spec_utils/solver_utils.hpp>
#include <motion_spec_utils/utils.hpp>
#include <string>

int main(int argc, char *argv[])
{
  Manipulator<kinova_mediator> kinova_right;
  kinova_right.base_frame = "kinova_right_base_link";
  kinova_right.tool_frame = "kinova_right_bracelet_link";
  kinova_right.mediator = nullptr;
  kinova_right.state = new ManipulatorState();

  Manipulator<kinova_mediator> kinova_left;
  kinova_left.base_frame = "kinova_left_base_link";
  kinova_left.tool_frame = "kinova_left_bracelet_link";
  kinova_left.mediator = nullptr;
  kinova_left.state = new ManipulatorState();

  KeloBaseConfig kelo_base_config;
  kelo_base_config.nWheels = 4;
  kelo_base_config.index_to_EtherCAT = new int[4]{3, 5, 7, 9};
  kelo_base_config.radius = 0.052;
  kelo_base_config.castor_offset = 0.01;
  kelo_base_config.half_wheel_distance = 0.0275;
  kelo_base_config.wheel_coordinates =
      new double[8]{0.175, 0.1605, -0.175, 0.1605, -0.175, -0.1605, 0.175, -0.1605};
  kelo_base_config.pivot_angles_deviation = new double[4]{-2.5, -1.25, -2.14, 1.49};

  MobileBase<Robile> freddy_base;
  Robile robile;
  robile.ethercat_config = new EthercatConfig();
  robile.kelo_base_config = &kelo_base_config;

  freddy_base.mediator = &robile;
  freddy_base.state = new MobileBaseState();

  // get current file path
  std::filesystem::path path = __FILE__;

  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  Freddy freddy = {&kinova_left, &kinova_right, &freddy_base};

  initialize_robot_sim(robot_urdf, &freddy);

  double platform_force[3] = {0.0, 0.0, 10.0};
  double wheel_torques[8]{};

  base_fd_solver(&freddy, platform_force, wheel_torques);

  printf("->Wheel torques: ");
  for (size_t i = 0; i < 8; i++)
  {
    printf("%f ", wheel_torques[i]);
  }

  free_robot_data(&freddy);

  return 0;
}