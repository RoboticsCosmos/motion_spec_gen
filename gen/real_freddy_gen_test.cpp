extern "C"
{
#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"
#include "kelo_motion_control/mediator.h"
}
#include <array>
#include <filesystem>
#include <iostream>
#include <string>
#include <chrono>
#include <controllers/pid_controller.hpp>
#include <kinova_mediator/mediator.hpp>
#include <motion_spec_utils/math_utils.hpp>
#include <motion_spec_utils/solver_utils.hpp>
#include <motion_spec_utils/utils.hpp>

int main()
{
  Manipulator<kinova_mediator> kinova_right;
  kinova_right.base_frame = "kinova_right_base_link";
  kinova_right.tool_frame = "kinova_right_bracelet_link";
  kinova_right.mediator = new kinova_mediator();
  kinova_right.state = new ManipulatorState();

  Manipulator<kinova_mediator> kinova_left;
  kinova_left.base_frame = "kinova_left_base_link";
  kinova_left.tool_frame = "kinova_left_bracelet_link";
  kinova_left.mediator = new kinova_mediator();
  kinova_left.state = new ManipulatorState();

  KeloBaseConfig kelo_base_config;
  kelo_base_config.nWheels = 4;
  kelo_base_config.index_to_EtherCAT = new int[4]{6, 7, 3, 4};
  kelo_base_config.radius = 0.052;
  kelo_base_config.castor_offset = 0.01;
  kelo_base_config.half_wheel_distance = 0.0275;
  kelo_base_config.wheel_coordinates =
      new double[8]{0.175, 0.1605, -0.175, 0.1605, -0.175, -0.1605, 0.175, -0.1605};
  kelo_base_config.pivot_angles_deviation = new double[4]{5.310, 5.533, 1.563, 1.625};

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

  char *interface = "eno1";
  initialize_robot(robot_urdf, interface, &freddy);

  // initialize variables
  int achd_solver_kinova_left_nc = 6;
  int achd_solver_kinova_left_nj = 7;
  std::string kinova_left_base_link = "kinova_left_base_link";
  std::string kinova_left_bracelet_link = "kinova_left_bracelet_link";
  double achd_solver_kinova_left_root_acceleration[6] = {-9.6, 0.99, 1.4,
                                                         0.0,   0.0,   0.0};
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
  int achd_solver_kinova_right_nc = 6;
  int achd_solver_kinova_right_nj = 7;
  std::string kinova_right_base_link = "kinova_right_base_link";
  std::string kinova_right_bracelet_link = "kinova_right_bracelet_link";
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

  int count = 0;
  std::cout << std::endl;

  bool kinova_left_first_time = true;
  bool kinova_right_first_time = true;

  const double desired_frequency = 900.0;  // Hz
  const auto desired_period =
      std::chrono::duration<double>(1.0 / desired_frequency);  // s

  while (true)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    count++;
    std::cout << "count: " << count << std::endl;
    // Get the robot structs with the data from robots
    get_robot_data(&freddy);
    print_robot_data(&freddy);
    // std::cout << std::endl;

    // solvers
    // printf("kinova right\n");
    achd_solver(&freddy, kinova_right_base_link, kinova_right_bracelet_link,
                achd_solver_kinova_right_nc, achd_solver_kinova_right_root_acceleration,
                achd_solver_kinova_right_alpha, achd_solver_kinova_right_beta,
                achd_solver_kinova_right_feed_forward_torques,
                achd_solver_kinova_right_predicted_accelerations,
                achd_solver_kinova_right_output_torques);
    // printf("right output torques: ");
    // for (size_t i = 0; i < 7; i++)
    // {
    //   std::cout << achd_solver_kinova_right_output_torques[i] << " ";
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;

    // double **rne_ext_wrench_right = new double *[7];
    // for (size_t i = 0; i < 7; i++)
    // {
    //   rne_ext_wrench_right[i] = new double[6]{};
    // }

    // // rne solver
    // rne_solver(&freddy, kinova_right_base_link, kinova_right_bracelet_link,
    //            achd_solver_kinova_right_root_acceleration, rne_ext_wrench_right,
    //            achd_solver_kinova_right_output_torques);

    // achd_solver
    // printf("kinova left\n");
    achd_solver(&freddy, kinova_left_base_link, kinova_left_bracelet_link,
                achd_solver_kinova_left_nc, achd_solver_kinova_left_root_acceleration,
                achd_solver_kinova_left_alpha, achd_solver_kinova_left_beta,
                achd_solver_kinova_left_feed_forward_torques,
                achd_solver_kinova_left_predicted_accelerations,
                achd_solver_kinova_left_output_torques);

    printf("achd left output torques: ");
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
    rne_solver(&freddy, kinova_left_base_link, kinova_left_bracelet_link,
               achd_solver_kinova_left_root_acceleration, rne_ext_wrench,
               achd_solver_kinova_left_output_torques);

    // Command the torques to the robots
    KDL::JntArray kinova_right_jnt_tau(7);
    cap_and_convert_torques(achd_solver_kinova_right_output_torques, 7, kinova_right_jnt_tau);

    // if (kinova_right_first_time)
    // {
    //   freddy.kinova_right->mediator->set_control_mode(2);
    //   kinova_right_first_time = false;
    // }
    // set_manipulator_torques(&freddy, kinova_right_base_link,
    //                         &kinova_right_jnt_tau);

    KDL::JntArray kinova_left_jnt_tau(7);
    cap_and_convert_torques(achd_solver_kinova_left_output_torques, 7, kinova_left_jnt_tau);

    std::cout << "kinova left torques: " << kinova_left_jnt_tau << std::endl;

    if (kinova_left_first_time)
    {
      freddy.kinova_left->mediator->set_control_mode(2);
      kinova_left_first_time = false;
    }
    set_manipulator_torques(&freddy, kinova_left_base_link,
                            &kinova_left_jnt_tau);

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

  free_robot_data(&freddy);

  return 0;
}