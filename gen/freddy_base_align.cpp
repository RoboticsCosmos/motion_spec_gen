extern "C"
{
#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"
#include "kelo_motion_control/mediator.h"
}
#include <array>
#include <string>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <controllers/pid_controller.hpp>
#include <motion_spec_utils/utils.hpp>
#include <motion_spec_utils/math_utils.hpp>
#include <motion_spec_utils/solver_utils.hpp>
#include <kinova_mediator/mediator.hpp>
#include <csignal>

#include <unsupported/Eigen/MatrixFunctions>

#include "kdl/treefksolverpos_recursive.hpp"

volatile sig_atomic_t flag = 0;

void handle_signal(int sig)
{
  flag = 1;
  printf("Caught signal %d\n", sig);
}

int main()
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

  // Initialize the robot structs

  KeloBaseConfig kelo_base_config;
  kelo_base_config.nWheels = 4;
  int index_to_EtherCAT[4] = {6, 7, 3, 4};
  kelo_base_config.index_to_EtherCAT = index_to_EtherCAT;
  kelo_base_config.radius = 0.115 / 2;
  kelo_base_config.castor_offset = 0.01;
  kelo_base_config.half_wheel_distance = 0.0775 / 2;
  double wheel_coordinates[8] = {0.188, 0.2075, -0.188, 0.2075, -0.188, -0.2075, 0.188, -0.2075};
  kelo_base_config.wheel_coordinates = wheel_coordinates;
  double pivot_angles_deviation[4] = {5.310, 5.533, 1.563, 1.625};
  kelo_base_config.pivot_angles_deviation = pivot_angles_deviation;

  MobileBase<Robile> freddy_base;
  Robile robile;
  robile.ethercat_config = new EthercatConfig();
  robile.kelo_base_config = &kelo_base_config;

  freddy_base.mediator = &robile;
  freddy_base.state = new MobileBaseState();

  Freddy robot = {nullptr, nullptr, &freddy_base};

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy_corrected_base.urdf").string();

  char ethernet_interface[100] = "eno1";
  initialize_robot(robot_urdf, ethernet_interface, &robot);

  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s
  double control_loop_timestep = desired_period.count();                               // s
  double *control_loop_dt = &control_loop_timestep;                                    // s

  // pid controller variables
  double kp = 5.0;
  double ki = 2.5;
  double kd = 1.5;

  // double wheel_lin_prev_error[4]{};
  // double wheel_lin_error_sum[4]{};
  // double wheel_ang_prev_error[4]{};
  // double wheel_ang_error_sum[4]{};

  double w2_lin_prev_error = 0.0;
  double w2_lin_error_sum = 0.0;
  double w2_ang_prev_error = 0.0;
  double w2_ang_error_sum = 0.0;

  get_robot_data(&robot, *control_loop_dt);

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
    printf("\n");
    // printf("count: %d\n", count);

    update_base_state(robot.mobile_base->mediator->kelo_base_config,
                      robot.mobile_base->mediator->ethercat_config);
    get_robot_data(&robot, *control_loop_dt);
    std::cout << std::endl;

    // solver
    double platform_force[3] = {-50.0, -50.0, 0.0};  // [N], [N], [Nm]

    std::cout << "platform force: ";
    print_array(platform_force, 3);

    double platform_weights[2] = {1.0, 1.0};

    double lin_offsets[robot.mobile_base->mediator->kelo_base_config->nWheels];
    double ang_offsets[robot.mobile_base->mediator->kelo_base_config->nWheels];
    get_pivot_alignment_offsets(&robot, platform_force, lin_offsets, ang_offsets);

    double lin_signals[4]{};
    double ang_signals[4]{};

    // pidController(lin_offsets[0], kp, ki, kd, control_loop_timestep, wheel_lin_error_sum[0],
    //               2.0, wheel_lin_prev_error[0], lin_signals[0]);
    // pidController(lin_offsets[1], kp, ki, kd, control_loop_timestep, wheel_lin_error_sum[1],
    //               2.0, wheel_lin_prev_error[1], lin_signals[1]);
    // pidController(lin_offsets[2], kp, ki, kd, control_loop_timestep, wheel_lin_error_sum[2],
    //               2.0, wheel_lin_prev_error[2], lin_signals[2]);
    // pidController(lin_offsets[3], kp, ki, kd, control_loop_timestep, wheel_lin_error_sum[3],
    //               2.0, wheel_lin_prev_error[3], lin_signals[3]);

    // pidController(ang_offsets[0], kp, ki, kd, control_loop_timestep, wheel_ang_error_sum[0],
    //               2.0, wheel_ang_prev_error[0], ang_signals[0]);
    // pidController(ang_offsets[1], kp, ki, kd, control_loop_timestep, wheel_ang_error_sum[1],
    //               2.0, wheel_ang_prev_error[1], ang_signals[1]);  
    // pidController(ang_offsets[2], kp, ki, kd, control_loop_timestep, wheel_ang_error_sum[2],
    //               2.0, wheel_ang_prev_error[2], ang_signals[2]);
    // pidController(ang_offsets[3], kp, ki, kd, control_loop_timestep, wheel_ang_error_sum[3],
    //               2.0, wheel_ang_prev_error[3], ang_signals[3]);

    pidController(lin_offsets[1], kp, ki, kd, control_loop_timestep, w2_lin_error_sum,
                  5.0, w2_lin_prev_error, lin_signals[1]);
    pidController(ang_offsets[1], kp, ki, kd, control_loop_timestep, w2_ang_error_sum,
                  5.0, w2_ang_prev_error, ang_signals[1]);

    std::cout << "lin signals: ";
    print_array(lin_signals, 4);
    std::cout << "ang signals: ";
    print_array(ang_signals, 4);

    double tau_wheel_c[8]{};
    // base_fd_solver_with_alignment(&robot, platform_force, lin_offsets, ang_offsets,
    //                               platform_weights, tau_wheel_c);
    base_fd_solver_with_alignment(&robot, platform_force, lin_signals, ang_signals,
                                  platform_weights, tau_wheel_c);

    // set torques
    double tau_wheel_c1[8]{};
    tau_wheel_c1[2] = tau_wheel_c[2];
    tau_wheel_c1[3] = tau_wheel_c[3];

    std::cout << "torques: ";
    print_array(tau_wheel_c1, 8);

    for (size_t i = 0; i < 8; i++)
    {
      if (tau_wheel_c1[i] > 5.0)
      {
        tau_wheel_c1[i] = 5.0;
      }
      else if (tau_wheel_c1[i] < -5.0)
      {
        tau_wheel_c1[i] = -5.0;
      }
    }

    set_mobile_base_torques(&robot, tau_wheel_c1);

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