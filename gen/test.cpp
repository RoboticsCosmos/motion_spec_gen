#include <array>
#include <controllers/pid_controller.hpp>
#include <filesystem>
#include <iostream>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/kinfam_io.hpp>
#include <motion_spec_utils/math_utils.hpp>
#include <motion_spec_utils/solver_utils.hpp>
#include "motion_spec_utils/robot_structs.hpp"
#include <motion_spec_utils/utils.hpp>
#include <string>

int main()
{
  // initialize the robot state
  Manipulator rob;
  // double init_q[7] = {0.0, 0.26, 0.0, 2.26, 0.0, -0.95, -1.57};
  // double init_q[14]{};

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

  initialize_robot_state(robot_chain_1.getNrOfJoints(), robot_chain_1.getNrOfSegments(), &rob);

  double *ext_wrench[11];

  for (size_t i = 0; i < 11; i++)
  {
    ext_wrench[i] = new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  double move_arm_down_vel_twist_achd_solver_output_torques[7]{};

  // achd_solver_fext(&rob, &robot_chain_1, ext_wrench,
  //                    move_arm_down_vel_twist_achd_solver_output_torques);

  double beta[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int move_arm_down_vel_twist_achd_solver_nc = 6;
  double move_arm_down_vel_twist_achd_solver_root_acceleration[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double *move_arm_down_vel_twist_achd_solver_alpha[6] = {
      new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, new double[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0}, new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double move_arm_down_vel_twist_achd_solver_predicted_accelerations[7]{};
  double move_arm_down_vel_twist_achd_solver_feed_forward_torques[7]{};
  achd_solver(&rob, &robot_chain_1, move_arm_down_vel_twist_achd_solver_nc,
                move_arm_down_vel_twist_achd_solver_root_acceleration,
                move_arm_down_vel_twist_achd_solver_alpha, beta, ext_wrench,
                move_arm_down_vel_twist_achd_solver_feed_forward_torques,
                move_arm_down_vel_twist_achd_solver_predicted_accelerations,
                move_arm_down_vel_twist_achd_solver_output_torques);

  // std::cout << "Arm 2: " << std::endl;
  // std::cout << "Number of joints: " << robot_chain_2.getNrOfJoints() << std::endl;
  // std::cout << "Number of segments: " << robot_chain_2.getNrOfSegments() << std::endl;

  // for (int i = 0; i < robot_chain_2.getNrOfSegments(); i++)
  // {
  //   std::cout << robot_chain_2.getSegment(i).getName() << std::endl;
  // }

  // kinova_left_base_link

  // KDL::Wrench wrench_at_left_base;
  // wrench_at_left_base.force = KDL::Vector(-5, 0, 0);
  // wrench_at_left_base.torque = KDL::Vector(0, 0, 0);

  // std::cout << "Wrench at left arm base link: " << std::endl;
  // std::cout << wrench_at_left_base.force(0) << " " << wrench_at_left_base.force(1) << " "
  //           << wrench_at_left_base.force(2) << " " << wrench_at_left_base.torque(0) << " "
  //           << wrench_at_left_base.torque(1) << " " << wrench_at_left_base.torque(2) << std::endl;

  // // transform the wrench to the base link
  // KDL::Frame base_to_left_base;

  // // compute the transform
  // KDL::ChainFkSolverPos_recursive fk_solver_1(robot_chain_1);

  // // initialize the joint array
  // KDL::JntArray init_q(robot_chain_2.getNrOfJoints());

  // // get the transform from the base link to the left base link
  // fk_solver_1.JntToCart(init_q, base_to_left_base, 3);

  // std::cout << "robile_base_to_left_arm_base: " << base_to_left_base.p(0) << " "
  //           << base_to_left_base.p(1) << " " << base_to_left_base.p(2) << std::endl;

  // KDL::Frame left_base_to_base = base_to_left_base.Inverse();

  // std::cout << "left_arm_base_to_robile_base: " << left_base_to_base.p(0) << " "
  //           << left_base_to_base.p(1) << " " << left_base_to_base.p(2) << std::endl;

  // // transform the wrench
  // KDL::Wrench wrench_at_base_link = left_base_to_base * wrench_at_left_base;

  // std::cout << "Wrench at robile base link: " << std::endl;
  // std::cout << wrench_at_base_link.force(0) << " " << wrench_at_base_link.force(1) << " "
  //           << wrench_at_base_link.force(2) << " " << wrench_at_base_link.torque(0) << " "
  //           << wrench_at_base_link.torque(1) << " " << wrench_at_base_link.torque(2) << std::endl;

  return 0;
}
