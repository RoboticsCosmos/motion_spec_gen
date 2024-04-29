#include <array>
#include <controllers/pid_controller.hpp>
#include <filesystem>
#include <iostream>
#include <kinova_mediator/mediator.hpp>
#include <motion_spec_utils/math_utils.hpp>
#include <motion_spec_utils/solver_utils.hpp>
#include <motion_spec_utils/utils.hpp>
#include <string>

#include "kelo_motion_control/mediator.h"

int main()
{
  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  // set the base and tool links
  std::string base_link = "base_link";
  std::string tool_link_1 = "kinova_left_bracelet_link";
  std::string tool_link_2 = "kinova_right_bracelet_link";

  // initialize the chain
  KDL::Chain kinova_left_chain;
  initialize_robot_chain(robot_urdf, base_link, tool_link_1, kinova_left_chain);

  KDL::Chain kinova_right_chain;
  initialize_robot_chain(robot_urdf, base_link, tool_link_2, kinova_right_chain);

  // Initialize the robot structs
  Manipulator kinova_right_state;
  MobileBase freddy_base_state;
  Manipulator kinova_left_state;

  initialize_manipulator_state(kinova_right_chain.getNrOfJoints(),
                               kinova_right_chain.getNrOfSegments(), & kinova_right_state);
  initialize_mobile_base_state(&freddy_base_state);
  initialize_manipulator_state(kinova_left_chain.getNrOfJoints(),
                               kinova_left_chain.getNrOfSegments(), & kinova_left_state);

  // Initialize the robot connections
  // Initialize the Manipulator connections
  kinova_mediator *kinova_right_mediator = new kinova_mediator();
  kinova_right_mediator->initialize(0, 0, 0.0);
  kinova_right_mediator->set_control_mode(2);
  // Initialize the MobileBase connections
  KeloBaseConfig freddy_base_config;
  EthercatConfig freddy_base_ethercat_config;
  initialize_kelo_base(&freddy_base_config, &freddy_base_ethercat_config);
  int result = 0;
  establish_kelo_base_connection(&freddy_base_ethercat_config, "eno1", &result);
  if (result != 0)
  {
    printf("Failed to establish connection to KeloBase\n");
    exit(1);
  }
  // Initialize the Manipulator connections
  kinova_mediator *kinova_left_mediator = new kinova_mediator();
  kinova_left_mediator->initialize(0, 0, 0.0);
  kinova_left_mediator->set_control_mode(2);

  // initialize variables

  while (true)
  {
    // Get the robot structs with the data from robots
    get_manipulator_data(&kinova_right_state, kinova_right_mediator);
    get_kelo_base_state(&freddy_base_config, &freddy_base_ethercat_config,
                        freddy_base_state.pivot_angles);
    get_manipulator_data(&kinova_left_state, kinova_left_mediator);

    // controllers

    // embed maps

    // solvers
  }

  return 0;
}