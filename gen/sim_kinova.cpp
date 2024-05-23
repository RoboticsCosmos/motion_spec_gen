extern "C"
{
#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"
#include "kelo_motion_control/mediator.h"
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
#include <chrono>

int main()
{
  Manipulator<kinova_mediator> kinova_left;
  kinova_left.base_frame = "kinova_left_base_link";
  kinova_left.tool_frame = "kinova_left_bracelet_link";
  kinova_left.mediator = nullptr;
  kinova_left.state = new ManipulatorState();

  Manipulator<kinova_mediator> kinova_right;
  kinova_right.base_frame = "kinova_right_base_link";
  kinova_right.tool_frame = "kinova_right_bracelet_link";
  kinova_right.mediator = nullptr;
  kinova_right.state = new ManipulatorState();

  std::string base_link = "base_link";

  // get current file path
  std::filesystem::path path = __FILE__;

  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  KDL::Tree tree;

  // load the robot urdf
  if (!kdl_parser::treeFromFile(robot_urdf, tree))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    exit(1);
  }

  // left arm
  if (!tree.getChain(kinova_left.base_frame, kinova_left.tool_frame, kinova_left.chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    exit(1);
  }

  // right arm
  if (!tree.getChain(kinova_right.base_frame, kinova_right.tool_frame,
                     kinova_right.chain))
  {
    std::cerr << "Failed to get chain from KDL tree" << std::endl;
    exit(1);
  }

  initialize_manipulator_state(kinova_left.chain.getNrOfJoints(),
                               kinova_left.chain.getNrOfSegments(), kinova_left.state);

  initialize_manipulator_state(kinova_right.chain.getNrOfJoints(),
                               kinova_right.chain.getNrOfSegments(), kinova_right.state);

  double init_angles[7] = {1.97791, -0.219453, 6.07101, -2.09586,
                           3.03115, -1.47373,  3.07873};

  double init_angles_right[7] = {3.63247, 0.0280151, 1.22534, -1.87552,
                                 3.52289, -1.28949,  5.98681};

  for (size_t i = 0; i < kinova_left.chain.getNrOfJoints(); i++)
  {
    kinova_left.state->q[i] = init_angles[i];
  }

  for (size_t i = 0; i < kinova_right.chain.getNrOfJoints(); i++)
  {
    kinova_right.state->q[i] = init_angles_right[i];
  }

  double achd_solver_kinova_nc = 6;
  double achd_solver_kinova_left_root_acceleration[6] = {-9.6, 0.99, 1.4, 0.0, 0.0, 0.0};
  double *achd_solver_kinova_left_alpha[6] = {
      new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double achd_solver_kinova_left_beta[6]{};
  for (size_t i = 0; i < 6; i++)
  {
    achd_solver_kinova_left_beta[i] = -achd_solver_kinova_left_root_acceleration[i];
  }
  double achd_solver_kinova_left_feed_forward_torques[7]{};

  double achd_solver_kinova_left_output_torques[7]{};
  double achd_solver_kinova_left_predicted_accelerations[7]{};

  // kinova right
  double achd_solver_kinova_right_root_acceleration[6] = {-9.685, -1.033, 1.324,
                                                          0.0,    0.0,    0.0};
  double achd_solver_kinova_right_output_acceleration_energy[6]{};
  double *achd_solver_kinova_right_alpha[6] = {
      new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double achd_solver_kinova_right_beta[6]{};
  for (size_t i = 0; i < 6; i++)
  {
    achd_solver_kinova_right_beta[i] = -achd_solver_kinova_right_root_acceleration[i];
  }
  double achd_solver_kinova_right_feed_forward_torques[7]{};

  double achd_solver_kinova_right_output_torques[7]{};
  double achd_solver_kinova_right_predicted_accelerations[7]{};

  const double desired_frequency = 900.0;  // Hz
  const auto desired_period =
      std::chrono::duration<double>(1.0 / desired_frequency);  // s

  int count = 0;
  while (count < 1000)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    count++;
    printf("\n--->count: %d\n", count);
    // Get the robot structs with the data from robots
    update_manipulator_state(kinova_left.state, kinova_left.tool_frame, &tree);
    update_manipulator_state(kinova_right.state, kinova_right.tool_frame, &tree);

    // left arm
    double *achd_solver_kinova_left_alpha_transf[6];
    for (size_t i = 0; i < achd_solver_kinova_nc; i++)
    {
      achd_solver_kinova_left_alpha_transf[i] = new double[6]{};
    }
    transform_alpha(&kinova_left, &tree, base_link, kinova_left.base_frame,
                    achd_solver_kinova_left_alpha, achd_solver_kinova_nc,
                    achd_solver_kinova_left_alpha_transf);
    achd_solver_manipulator(&kinova_left, achd_solver_kinova_nc,
                            achd_solver_kinova_left_root_acceleration,
                            achd_solver_kinova_left_alpha_transf, achd_solver_kinova_left_beta,
                            achd_solver_kinova_left_feed_forward_torques,
                            achd_solver_kinova_left_predicted_accelerations,
                            achd_solver_kinova_left_output_torques);

    printf("[achd ] left torques: ");
    for (size_t i = 0; i < 7; i++)
    {
      std::cout << achd_solver_kinova_left_output_torques[i] << " ";
    }
    std::cout << std::endl;

    double **rne_ext_wrench = new double *[7];
    for (size_t i = 0; i < 7; i++)
    {
      rne_ext_wrench[i] = new double[6]{};
    }

    // rne solver
    rne_solver_manipulator(&kinova_left, achd_solver_kinova_left_root_acceleration,
                           rne_ext_wrench, achd_solver_kinova_left_output_torques);

    printf("[rne  ] left torques: ");
    for (size_t i = 0; i < 7; i++)
    {
      std::cout << achd_solver_kinova_left_output_torques[i] << " ";
    }
    std::cout << std::endl << std::endl;

    // right arm
    double *achd_solver_kinova_right_alpha_transf[6];
    for (size_t i = 0; i < achd_solver_kinova_nc; i++)
    {
      achd_solver_kinova_right_alpha_transf[i] = new double[6]{};
    }
    transform_alpha(&kinova_right, &tree, base_link, kinova_right.base_frame,
                    achd_solver_kinova_right_alpha, achd_solver_kinova_nc,
                    achd_solver_kinova_right_alpha_transf);
    achd_solver_manipulator(&kinova_right, achd_solver_kinova_nc,
                            achd_solver_kinova_right_root_acceleration,
                            achd_solver_kinova_right_alpha_transf, achd_solver_kinova_right_beta,
                            achd_solver_kinova_right_feed_forward_torques,
                            achd_solver_kinova_right_predicted_accelerations,
                            achd_solver_kinova_right_output_torques);

    printf("[achd ] right torques: ");
    for (size_t i = 0; i < 7; i++)
    {
      std::cout << achd_solver_kinova_right_output_torques[i] << " ";
    }
    std::cout << std::endl;

    double **rne_ext_wrench_right = new double *[7];
    for (size_t i = 0; i < 7; i++)
    {
      rne_ext_wrench_right[i] = new double[6]{};
    }

    // rne
    rne_solver_manipulator(&kinova_right, achd_solver_kinova_right_root_acceleration,
                           rne_ext_wrench_right, achd_solver_kinova_right_output_torques);

    printf("[rne  ] right torques : ");
    for (size_t i = 0; i < 7; i++)
    {
      std::cout << achd_solver_kinova_right_output_torques[i] << " ";
    }
    std::cout << std::endl;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration<double>(end_time - start_time);

    // if the elapsed time is less than the desired period, busy wait
    while (elapsed_time < desired_period)
    {
      end_time = std::chrono::high_resolution_clock::now();
      elapsed_time = std::chrono::duration<double>(end_time - start_time);
    }
  }

  free_manipulator(&kinova_left);

  return 0;
}