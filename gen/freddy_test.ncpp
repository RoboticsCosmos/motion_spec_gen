#include <array>
#include <controllers/pid_controller.hpp>
#include <filesystem>
#include <iostream>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/kinfam_io.hpp>
#include <motion_spec_utils/math_utils.hpp>
#include <motion_spec_utils/solver_utils.hpp>
#include <motion_spec_utils/utils.hpp>
#include <string>

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

  return 0;
}
