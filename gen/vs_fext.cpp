#include <array>
#include <controllers/pid_controller.hpp>
#include <filesystem>
#include <iostream>
#include <motion_spec_utils/math_utils.hpp>
#include <motion_spec_utils/solver_utils.hpp>
#include <motion_spec_utils/utils.hpp>
#include <string>

#include "kdl/chainidsolver_recursive_newton_euler.hpp"

int main()
{
  // initialize the robot state
  Manipulator rob;
  // double init_q[7] = {0.0, 0.26, 0.0, 2.26, 0.0, -0.95, -1.57};
  double init_q[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  initialize_robot_state(7, 7, init_q, &rob);

  double dt = 0.001;

  // initialize variables
  double bracelet_link_vel_twist[6]{};
  int move_arm_down_vel_twist_achd_solver_nc = 6;
  int move_arm_down_vel_twist_achd_solver_nj = 7;
  int move_arm_down_vel_twist_achd_solver_ns = 8;
  std::string bracelet_link = "bracelet_link";

  double move_arm_down_vel_twist_achd_solver_feed_forward_torques[7] = {0.0, 0.0, 0.0, 0.0,
                                                                        0.0, 0.0, 0.0};
  double *move_arm_down_vel_twist_achd_solver_alpha[6] = {
      new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, new double[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0}, new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double move_arm_down_vel_twist_achd_solver_predicted_accelerations[7]{};
  double move_arm_down_vel_twist_achd_solver_output_torques[7]{};
  double move_arm_down_vel_twist_monitor_post_bracelet_link_vel_twist_sp[6] = {0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0};
  double move_arm_down_vel_twist_achd_solver_root_acceleration[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "gen3_robotiq_2f_85.urdf").string();

  // set the base and tool links
  std::string base_link = "base_link";
  std::string tool_link = "bracelet_link";

  // initialize the chain
  KDL::Chain robot_chain;
  initialize_robot_chain(robot_urdf, base_link, tool_link, robot_chain);

  int count = 0;

  while (true && count < 3)
  {
    std::cout << std::endl;
    std ::cout << "---------------------->count: " << count << std::endl;
    count++;

    double command_accelerations[7]{};

    add(move_arm_down_vel_twist_achd_solver_predicted_accelerations, command_accelerations,
        command_accelerations, 7);
    updateQandQdot(command_accelerations, dt, &rob);

    // solvers
    // solver beta
    double beta[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // solver ext_wrench
    double *ext_wrench[7];

    for (size_t i = 0; i < 6; i++)
    {
      ext_wrench[i] = new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    ext_wrench[6] = new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    achd_solver(&rob, &robot_chain, move_arm_down_vel_twist_achd_solver_nc,
                move_arm_down_vel_twist_achd_solver_root_acceleration,
                move_arm_down_vel_twist_achd_solver_alpha, beta, ext_wrench,
                move_arm_down_vel_twist_achd_solver_feed_forward_torques,
                move_arm_down_vel_twist_achd_solver_predicted_accelerations,
                move_arm_down_vel_twist_achd_solver_output_torques);

    achd_solver_fext(&rob, &robot_chain, ext_wrench,
                     move_arm_down_vel_twist_achd_solver_output_torques);

    rne_solver(&rob, &robot_chain, move_arm_down_vel_twist_achd_solver_root_acceleration,
               ext_wrench, move_arm_down_vel_twist_achd_solver_output_torques);
  }

  return 0;
}
