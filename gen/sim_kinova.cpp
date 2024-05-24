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

  MobileBase<Robile> freddy_base;
  freddy_base.mediator = nullptr;
  freddy_base.state = new MobileBaseState();

  std::string base_link = "base_link";

  Freddy robot = {&kinova_left, &kinova_right, &freddy_base};

  // get current file path
  std::filesystem::path path = __FILE__;

  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  initialize_robot_sim(robot_urdf, &robot);

  double init_angles_left[7] = {1.97791, -0.219453, 6.07101, -2.09586,
                                3.03115, -1.47373,  3.07873};

  // double init_angles_right[7] = {3.63247, 0.0280151, 1.22534, -1.87552,
  //                                3.52289, -1.28949,  5.98681};

  // double init_angles_right[7] = {6.23973, 1.61685,   3.18572, -0.064486,
  //                                1.99243, 0.0948623, 2.64145};

  // double init_angles_right[7] = {0.645591, 0.306755, 3.94353, -2.04943,
  //                                0.138919, 1.56452, 2.82834};

  double init_angles_right[7] = {4.10969, -0.859412, 0.347875, -1.66477,
                                 6.05833, 1.49967,   3.14005};

  for (size_t i = 0; i < kinova_right.chain.getNrOfJoints(); i++)
  {
    kinova_right.state->q[i] = init_angles_right[i];
  }

  // kinova right
  double achd_solver_kinova_right_root_acceleration[6] = {-9.685, -1.033, 1.324,
                                                          0.0,    0.0,    0.0};
  double achd_solver_base_root_acceleration[6] = {0., 0.0, -9.81, 0.0, 0.0, 0.0};
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
    achd_solver_kinova_right_beta[i] = -achd_solver_base_root_acceleration[i];
  }
  double achd_solver_kinova_right_feed_forward_torques[7]{};

  double achd_solver_kinova_right_output_torques[7]{};
  double achd_solver_kinova_right_predicted_accelerations[7]{};
  int achd_solver_kinova_nc = 6;

  const double desired_frequency = 900.0;  // Hz
  const auto desired_period =
      std::chrono::duration<double>(1.0 / desired_frequency);  // s

  int count = 0;
  while (count < 25)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    count++;
    printf("\n------------------------------------------\n> count: %d\n", count);
    // Get the robot structs with the data from robots
    get_robot_data_sim(&robot, nullptr, achd_solver_kinova_right_predicted_accelerations,
                       0.01);
    print_robot_data(&robot);

    // right arm
    std::cout << "\n> [right arm]" << std::endl;
    double *achd_solver_kinova_right_alpha_transf[6];
    for (size_t i = 0; i < achd_solver_kinova_nc; i++)
    {
      achd_solver_kinova_right_alpha_transf[i] = new double[6]{};
    }
    transform_alpha(&robot, base_link, kinova_right.base_frame,
                    achd_solver_kinova_right_alpha, achd_solver_kinova_nc,
                    achd_solver_kinova_right_alpha_transf);
    achd_solver(&robot, kinova_right.base_frame, kinova_right.tool_frame,
                achd_solver_kinova_nc, achd_solver_kinova_right_root_acceleration,
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
    rne_solver(&robot, kinova_right.base_frame, kinova_right.tool_frame,
               achd_solver_kinova_right_root_acceleration, rne_ext_wrench_right,
               achd_solver_kinova_right_output_torques);

    printf("[rne  ] right torques: ");
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