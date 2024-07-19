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
  Manipulator<kinova_mediator> kinova_right;
  kinova_right.base_frame = "kinova_right_base_link";
  kinova_right.tool_frame = "kinova_right_bracelet_link";
  kinova_right.mediator = new kinova_mediator();
  kinova_right.state = new ManipulatorState();
  bool kinova_right_torque_control_mode_set = false;

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

  Manipulator<kinova_mediator> kinova_left;
  kinova_left.base_frame = "kinova_left_base_link";
  kinova_left.tool_frame = "kinova_left_bracelet_link";
  kinova_left.mediator = new kinova_mediator();
  kinova_left.state = new ManipulatorState();
  bool kinova_left_torque_control_mode_set = false;

  Freddy robot = {&kinova_left, &kinova_right, &freddy_base};
  // Freddy robot = {nullptr, nullptr, &freddy_base};

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  char ethernet_interface[100] = "eno1";
  initialize_robot(robot_urdf, ethernet_interface, &robot);

  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s
  double control_loop_timestep = desired_period.count();                               // s
  double *control_loop_dt = &control_loop_timestep;                                    // s

  // initialize variables
  double kr_achd_solver_root_acceleration[6] = {-9.685, -1.033, 1.34, 0.0, 0.0, 0.0};
  double kl_achd_solver_root_acceleration[6] = {-9.6, 0.92, 1.4, 0.0, 0.0, 0.0};

  double fd_solver_robile_output_torques[8]{};

  update_base_state(robot.mobile_base->mediator->kelo_base_config,
                    robot.mobile_base->mediator->ethercat_config);
  get_robot_data(&robot, *control_loop_dt);

  int count = 0;

  while (true)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (flag)
    {
      free_robot_data(&robot);
      printf("Exiting somewhat cleanly...\n");
      exit(0);
    }

    count++;
    printf("\n");
    // printf("count: %d\n", count);

    update_base_state(robot.mobile_base->mediator->kelo_base_config,
                    robot.mobile_base->mediator->ethercat_config);
    get_robot_data(&robot, *control_loop_dt);

    print_robot_data(&robot);

    // base_fd_solver
    double plat_force[3] = {300.0, 0.0, 0.0};
    std::cout << "plat_force: ";
    print_array(plat_force, 3);

    double lin_offsets[robot.mobile_base->mediator->kelo_base_config->nWheels];
    double ang_offsets[robot.mobile_base->mediator->kelo_base_config->nWheels];
    get_pivot_alignment_offsets(&robot, plat_force, lin_offsets, ang_offsets);
    double platform_weights[2] = {2.0, 1.0};
    base_fd_solver_with_alignment(&robot, plat_force, lin_offsets, ang_offsets,
                                  platform_weights, fd_solver_robile_output_torques);

    // init torques
    double kr_rne_ext_wrench[7][6]{};
    double kr_rne_output_torques[7]{};

    double kl_rne_ext_wrench[7][6]{};
    double kl_rne_output_torques[7]{};

    // rne
    // rne_solver(&robot, kinova_right.base_frame, kinova_right.tool_frame,
    //           kr_achd_solver_root_acceleration, kr_rne_ext_wrench, kr_rne_output_torques);

    // rne_solver(&robot, kinova_left.base_frame, kinova_left.tool_frame,
    //           kl_achd_solver_root_acceleration, kl_rne_ext_wrench, kl_rne_output_torques);

    // cap torques
    KDL::JntArray kinova_right_cmd_tau_kdl(7);
    // cap_and_convert_manipulator_torques(kr_rne_output_torques, 7, kinova_right_cmd_tau_kdl);

    KDL::JntArray kinova_left_cmd_tau_kdl(7);
    // cap_and_convert_manipulator_torques(kl_rne_output_torques, 7, kinova_left_cmd_tau_kdl);

    for (size_t i = 0; i < 8; i++)
    {
      if (fd_solver_robile_output_torques[i] > 5.0)
      {
        fd_solver_robile_output_torques[i] = 5.0;
      }
      else if (fd_solver_robile_output_torques[i] < -5.0)
      {
        fd_solver_robile_output_torques[i] = -5.0;
      }
    }

    // set torques
    set_mobile_base_torques(&robot, fd_solver_robile_output_torques);
    set_manipulator_torques(&robot, kinova_left.base_frame, &kinova_left_cmd_tau_kdl);
    set_manipulator_torques(&robot, kinova_right.base_frame, &kinova_right_cmd_tau_kdl);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration<double>(end_time - start_time);

    // if the elapsed time is less than the desired period, busy wait
    while (elapsed_time < desired_period)
    {
      end_time = std::chrono::high_resolution_clock::now();
      elapsed_time = std::chrono::duration<double>(end_time - start_time);
    }
    control_loop_timestep = elapsed_time.count();
    std::cout << "control loop timestep: " << control_loop_timestep << std::endl;
  }

  free_robot_data(&robot);

  return 0;
}