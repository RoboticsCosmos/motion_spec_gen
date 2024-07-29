#include "kelo_motion_control/mediator.h"
#include <array>
#include <string>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <controllers/pid_controller.hpp>
#include "controllers/abag.h"
#include <motion_spec_utils/utils.hpp>
#include <motion_spec_utils/math_utils.hpp>
#include <motion_spec_utils/solver_utils.hpp>
#include <csignal>

#include <unsupported/Eigen/MatrixFunctions>

volatile sig_atomic_t flag = 0;

void handle_signal(int sig)
{
  flag = 1;
  printf("Caught signal %d\n", sig);
}

int main(int argc, char **argv)
{
  // handle signals
  struct sigaction sa;
  sa.sa_handler = handle_signal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;

  for (int i = 1; i < NSIG; ++i)
  {
    if (sigaction(i, &sa, NULL) == -1)
    {
      perror("sigaction");
    }
  }

  // read the platform force from the command line
  double pf[3] = {0.0, 0.0, 0.0};
  if (argc == 4)
  {
    pf[0] = std::stod(argv[1]);
    pf[1] = std::stod(argv[2]);
    pf[2] = std::stod(argv[3]);
  }
  else
  {
    printf(
        "Usage: ./freddy_base_align <platform_force_x> <platform_force_y> <platform_torque_z>\n");
    exit(1);
  }

  // Initialize the robot structs
  KeloBaseConfig *kelo_base_config = new KeloBaseConfig();
  kelo_base_config->nWheels = 4;
  int index_to_EtherCAT[4] = {6, 7, 3, 4};
  kelo_base_config->index_to_EtherCAT = index_to_EtherCAT;
  kelo_base_config->radius = 0.115 / 2;
  kelo_base_config->castor_offset = 0.01;
  kelo_base_config->half_wheel_distance = 0.0775 / 2;
  double wheel_coordinates[8] = {0.188, 0.2075, -0.188, 0.2075, -0.188, -0.2075, 0.188, -0.2075};
  kelo_base_config->wheel_coordinates = wheel_coordinates;
  double pivot_angles_deviation[4] = {5.310, 5.533, 1.563, 1.625};
  kelo_base_config->pivot_angles_deviation = pivot_angles_deviation;

  EthercatConfig *ethercat_config = new EthercatConfig();

  MobileBase<Robile> freddy_base;
  Robile robile;
  robile.ethercat_config = ethercat_config;
  robile.kelo_base_config = kelo_base_config;

  freddy_base.mediator = &robile;
  freddy_base.state = new MobileBaseState();

  Freddy robot = {nullptr, nullptr, &freddy_base};

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy_corrected_base.urdf").string();

  char ethernet_interface[100] = "eno1";
  initialize_robot(&robot, robot_urdf, ethernet_interface);

  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s
  double control_loop_timestep = desired_period.count();                               // s
  double *control_loop_dt = &control_loop_timestep;                                    // s

  // ABAG parameters
  const double alpha_parameter[8]          = { 0.950000, 0.950000, 0.950000, 0.950000, 0.950000, 0.950000, 0.950000, 0.950000 };
  const double bias_threshold_parameter[8] = { 0.000407, 0.000407, 0.000407, 0.000407, 0.000407, 0.000407, 0.000407, 0.000407 };
  const double bias_step_parameter[8]      = { 0.000400, 0.000400, 0.000400, 0.000400, 0.000400, 0.000400, 0.000400, 0.000400 };
  const double gain_threshold_parameter[8] = { 0.550000, 0.550000, 0.550000, 0.550000, 0.550000, 0.550000, 0.550000, 0.550000 };
  const double gain_step_parameter[8]      = { 0.003000, 0.003000, 0.003000, 0.003000, 0.003000, 0.003000, 0.003000, 0.003000 };
  double desired_state[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double tube[8]          = { 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15 };
  double ctrl_error[8]    = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double abag_command[8]  = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  abagState_t abag_state[8];
  for (int i = 0; i < 8; i++)
  {
      initialize_abagState(&abag_state[i]);
  }

  int count = 0;

  while (true)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (flag)
    {
      printf("Exiting somewhat cleanly...\n");
      free_robot_data(&robot);
      exit(0);
    }

    count++;
    // printf("\n");
    printf("count: %d\n", count);

    // update_base_state(robot.mobile_base->mediator->kelo_base_config,
    //                   robot.mobile_base->mediator->ethercat_config);
    get_robot_data(&robot, *control_loop_dt);

    // solver
    double platform_force[3] = {pf[0], pf[1], pf[2]};  // [N], [N], [Nm]

    // compute the weights for the platform force
    double platform_weights[2];
    platform_weights[0] = abs(platform_force[2]) < 1e-6
                              ? 1.0
                              : sqrt(pow(platform_force[0], 2) + pow(platform_force[1], 2)) /
                                    (sqrt(pow(platform_force[0], 2) + pow(platform_force[1], 2) +
                                          pow(platform_force[2], 2)));

    platform_weights[1] = 1.0 - platform_weights[0];

    double lin_offsets[robot.mobile_base->mediator->kelo_base_config->nWheels];
    double ang_offsets[robot.mobile_base->mediator->kelo_base_config->nWheels];
    get_pivot_alignment_offsets(&robot, platform_force, lin_offsets, ang_offsets);

    Eigen::Vector2d lin_pf = Eigen::Vector2d(platform_force[0], platform_force[1]);

    double lin_force_weight = lin_pf.norm() == 0.0 ? 0.0 : platform_weights[0];
    double moment_weight = platform_force[2] == 0.0 ? 0.0 : platform_weights[1];

    double lin_signal_w1, lin_signal_w2, lin_signal_w3, lin_signal_w4 = 0.0;
    double ang_signal_w1, ang_signal_w2, ang_signal_w3, ang_signal_w4 = 0.0;

    // ABAG
    ctrl_error[0] = lin_offsets[0];
    if (fabs(ctrl_error[0]) < tube[0]) ctrl_error[0] = 0.0;
    abag_sched(&abag_state[0], &ctrl_error[0],
                   &abag_command[0], &alpha_parameter[0],
                   &bias_threshold_parameter[0], &bias_step_parameter[0],
                   &gain_threshold_parameter[0], &gain_step_parameter[0]);
    lin_signal_w1 = abag_command[0];

    ctrl_error[1] = lin_offsets[1];
    if (fabs(ctrl_error[1]) < tube[1]) ctrl_error[1] = 0.0;
    abag_sched(&abag_state[1], &ctrl_error[1],
                   &abag_command[1], &alpha_parameter[1],
                   &bias_threshold_parameter[1], &bias_step_parameter[1],
                   &gain_threshold_parameter[1], &gain_step_parameter[1]);
    lin_signal_w2 = abag_command[1];

    ctrl_error[2] = lin_offsets[2];
    if (fabs(ctrl_error[2]) < tube[2]) ctrl_error[2] = 0.0;
    abag_sched(&abag_state[2], &ctrl_error[2],
                   &abag_command[2], &alpha_parameter[2],
                   &bias_threshold_parameter[2], &bias_step_parameter[2],
                   &gain_threshold_parameter[2], &gain_step_parameter[2]);
    lin_signal_w3 = abag_command[2];

    ctrl_error[3] = lin_offsets[3];
    if (fabs(ctrl_error[3]) < tube[3]) ctrl_error[3] = 0.0;
    abag_sched(&abag_state[3], &ctrl_error[3],
                   &abag_command[3], &alpha_parameter[3],
                   &bias_threshold_parameter[3], &bias_step_parameter[3],
                   &gain_threshold_parameter[3], &gain_step_parameter[3]);
    lin_signal_w4 = abag_command[3];

    ctrl_error[4] = ang_offsets[0];
    if (fabs(ctrl_error[4]) < tube[4]) ctrl_error[4] = 0.0;
    abag_sched(&abag_state[4], &ctrl_error[4],
                   &abag_command[4], &alpha_parameter[4],
                   &bias_threshold_parameter[4], &bias_step_parameter[4],
                   &gain_threshold_parameter[4], &gain_step_parameter[4]);
    ang_signal_w1 = abag_command[4];

    ctrl_error[5] = ang_offsets[1];
    if (fabs(ctrl_error[5]) < tube[5]) ctrl_error[5] = 0.0;
    abag_sched(&abag_state[5], &ctrl_error[5],
                   &abag_command[5], &alpha_parameter[5],
                   &bias_threshold_parameter[5], &bias_step_parameter[5],
                   &gain_threshold_parameter[5], &gain_step_parameter[5]);
    ang_signal_w2 = abag_command[5];

    ctrl_error[6] = ang_offsets[2];
    if (fabs(ctrl_error[6]) < tube[6]) ctrl_error[6] = 0.0;
    abag_sched(&abag_state[6], &ctrl_error[6],
                   &abag_command[6], &alpha_parameter[6],
                   &bias_threshold_parameter[6], &bias_step_parameter[6],
                   &gain_threshold_parameter[6], &gain_step_parameter[6]);
    ang_signal_w3 = abag_command[6];

    ctrl_error[7] = ang_offsets[3];
    if (fabs(ctrl_error[7]) < tube[7]) ctrl_error[7] = 0.0;
    abag_sched(&abag_state[7], &ctrl_error[7],
                   &abag_command[7], &alpha_parameter[7],
                   &bias_threshold_parameter[7], &bias_step_parameter[7],
                   &gain_threshold_parameter[7], &gain_step_parameter[7]);
    ang_signal_w4 = abag_command[7];

    double lin_signals[4] = {lin_signal_w1, lin_signal_w2, lin_signal_w3, lin_signal_w4};
    double ang_signals[4] = {ang_signal_w1, ang_signal_w2, ang_signal_w3, ang_signal_w4};

    double alignment_taus[robot.mobile_base->mediator->kelo_base_config->nWheels];
    for (size_t i = 0; i < robot.mobile_base->mediator->kelo_base_config->nWheels; i++)
    {
      alignment_taus[i] = lin_signals[i] * lin_force_weight + ang_signals[i] * moment_weight;
    }

    double tau_wheel_ref[robot.mobile_base->mediator->kelo_base_config->nWheels * 2];
    for (size_t i = 0; i < robot.mobile_base->mediator->kelo_base_config->nWheels; i++)
    {
      tau_wheel_ref[2 * i] = alignment_taus[i] * 2.0;
      tau_wheel_ref[2 * i + 1] = -alignment_taus[i] * 2.0;
    }

    double tau_wheel_c[8]{};
    for (size_t i = 0; i < 8; i++)
    {
      tau_wheel_c[i] = tau_wheel_ref[i];
    }

    // set torques
    double tau_limit = 4.0;
    for (size_t i = 0; i < 8; i++)
    {
      if (tau_wheel_c[i] > tau_limit)
      {
        tau_wheel_c[i] = tau_limit;
      }
      else if (tau_wheel_c[i] < -tau_limit)
      {
        tau_wheel_c[i] = -tau_limit;
      }
    }

    printf("torques: ");
    print_array(tau_wheel_c, 8);

    printf("\n");

    if (count > 2)
    {
      set_mobile_base_torques(&robot, tau_wheel_c);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration<double>(end_time - start_time);

    // if the elapsed time is less than the desired period, busy wait
    while (elapsed_time < desired_period)
    {
      end_time = std::chrono::high_resolution_clock::now();
      elapsed_time = std::chrono::duration<double>(end_time - start_time);
    }
    control_loop_timestep = elapsed_time.count();
    // std::cout << "control loop timestep: " << control_loop_timestep << std::endl;
  }

  free_robot_data(&robot);

  return 0;
}