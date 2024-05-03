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
  std::string base_link = "kinova_left_base_link";
  std::string tool_link_1 = "kinova_left_bracelet_link";

  // initialize the chain
  KDL::Chain kinova_left_chain;
  initialize_robot_chain(robot_urdf, base_link, tool_link_1, kinova_left_chain);

  double achd_solver_feed_forward_torques[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double *achd_solver_alpha[6] = {new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                  new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                                  new double[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                                  new double[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                                  new double[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                                  new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double achd_solver_predicted_accelerations[7]{};
  double achd_solver_output_torques[7]{};
  double achd_solver_root_acceleration[6] = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};

  Manipulator rob;
  // double init_q[7] = {0.0, 0.26, 0.0, 2.26, 0.0, -0.95, -1.57};
  double init_q[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  initialize_robot_state(7, kinova_left_chain.getNrOfSegments(), init_q, &rob);

  int count = 0;

  while (true && count < 50)
  {
    std::cout << std::endl;
    std ::cout << "---------------------->count: " << count << std::endl;
    count++;

    double command_accelerations[7]{};

    add(achd_solver_predicted_accelerations, command_accelerations, command_accelerations,
        7);
    updateQandQdot(command_accelerations, 0.001, &rob);

    // print bracelet_link velocity twist
    std::cout << "bracelet_link: ";
    for (size_t i = 0; i < 6; i++)
    {
      std::cout << rob.s[6][i] << " ";
    }
    std::cout << std::endl;

    // solvers
    // solver beta
    double beta[6] = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
    // solver ext_wrench
    double *ext_wrench[7];

    for (size_t i = 0; i < kinova_left_chain.getNrOfSegments(); i++)
    {
      ext_wrench[i] = new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    achd_solver(&rob, &kinova_left_chain, 6, achd_solver_root_acceleration,
                achd_solver_alpha, beta, ext_wrench, achd_solver_feed_forward_torques,
                achd_solver_predicted_accelerations, achd_solver_output_torques);

    // print achd_solver_output_torques
    std::cout << "achd_solver_output_torques: ";
    for (size_t i = 0; i < 7; i++)
    {
      std::cout << achd_solver_output_torques[i] << " ";
    }
  }

  return 0;
}
