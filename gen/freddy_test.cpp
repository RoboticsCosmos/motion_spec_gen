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
  std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  // set the base and tool links
  std::string base_link = "base_link";
  std::string tool_link_1 = "kinova_left_bracelet_link";
  std::string tool_link_2 = "kinova_right_bracelet_link";

  // initialize the chain
  KDL::Chain robot_chain_1;
  initialize_robot_chain(robot_urdf, base_link, tool_link_1, robot_chain_1);

  KDL::Chain robot_chain_2;
  initialize_robot_chain(robot_urdf, base_link, tool_link_2, robot_chain_2);

  // print chain info

  // print number of joints and segments
  std::cout << "Arm 1: " << std::endl;
  std::cout << "Number of joints: " << robot_chain_1.getNrOfJoints() << std::endl;
  std::cout << "Number of segments: " << robot_chain_1.getNrOfSegments() << std::endl;

  for (int i = 0; i < robot_chain_1.getNrOfSegments(); i++)
  {
    std::cout << robot_chain_1.getSegment(i).getName() << std::endl;
  }

  std::cout << std::endl;


  
  return 0;
}
