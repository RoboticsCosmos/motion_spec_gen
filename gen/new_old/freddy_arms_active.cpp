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

#include "motion_spec_utils/log_structs.hpp"

volatile sig_atomic_t flag = 0;

void handle_signal(int sig)
{
  flag = 1;
  // printf("Caught signal %d\n", sig);
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

  Freddy robot = {&kinova_left, &kinova_right, nullptr};

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  char ethernet_interface[100] = "eno1";
  initialize_robot(&robot, robot_urdf, ethernet_interface);

  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s
  double control_loop_timestep = desired_period.count();                               // s
  double *control_loop_dt = &control_loop_timestep;                                    // s

  // log structs
  std::string log_dir = "../../logs/data/freddy_arms_active";
  char log_dir_name[100];
  get_new_folder_name(log_dir.c_str(), log_dir_name);
  std::filesystem::create_directories(log_dir_name);
  std::filesystem::permissions(log_dir_name, std::filesystem::perms::all);

  LogManipulatorVoltageCurrentDataVector kr_log_data_vec("kinova_right", log_dir_name);
  LogManipulatorVoltageCurrentDataVector kl_log_data_vec("kinova_left", log_dir_name);


  // explicitly referesh the robot data
  robot.kinova_left->mediator->refresh_feedback();
  robot.kinova_right->mediator->refresh_feedback();
  // update_base_state(robot.mobile_base->mediator->kelo_base_config,
  //                   robot.mobile_base->mediator->ethercat_config);
  get_robot_data(&robot, *control_loop_dt);

  // get voltage and current data
  double kr_base_voltage, kr_base_current = 0.0;
  double kl_base_voltage, kl_base_current = 0.0;
  double kr_actuator_voltages[7]{};
  double kr_actuator_currents[7]{};
  double kl_actuator_voltages[7]{};
  double kl_actuator_currents[7]{};
  robot.kinova_right->mediator->get_arm_voltage(kr_base_voltage, kr_actuator_voltages);
  robot.kinova_right->mediator->get_arm_current(kr_base_current, kr_actuator_currents);
  robot.kinova_left->mediator->get_arm_voltage(kl_base_voltage, kl_actuator_voltages);
  robot.kinova_left->mediator->get_arm_current(kl_base_current, kl_actuator_currents);

  // log the data
  kr_log_data_vec.addVoltageCurrentData(kr_base_voltage, kr_base_current, kr_actuator_voltages,
                                        kr_actuator_currents);
  kl_log_data_vec.addVoltageCurrentData(kl_base_voltage, kl_base_current, kl_actuator_voltages,
                                        kl_actuator_currents);

  int count = 0;

  while (true)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (flag)
    {
      kr_log_data_vec.writeToOpenFile();
      kl_log_data_vec.writeToOpenFile();

      free_robot_data(&robot);
      printf("Exiting somewhat cleanly...\n");
      exit(0);
    }

    count++;
    // printf("\n");
    printf("count: %d\n", count);

    // update_base_state(robot.mobile_base->mediator->kelo_base_config,
    //                   robot.mobile_base->mediator->ethercat_config);
    get_robot_data(&robot, *control_loop_dt);

    robot.kinova_right->mediator->get_arm_voltage(kr_base_voltage, kr_actuator_voltages);
    robot.kinova_right->mediator->get_arm_current(kr_base_current, kr_actuator_currents);
    robot.kinova_left->mediator->get_arm_voltage(kl_base_voltage, kl_actuator_voltages);
    robot.kinova_left->mediator->get_arm_current(kl_base_current, kl_actuator_currents);

    // get_kelo_wheel_voltages_and_currents(
    //     robot.mobile_base->mediator->kelo_base_config,
    //     robot.mobile_base->mediator->ethercat_config, base_bus_voltages, base_wheel_voltages,
    //     base_wheel_currents);

    kr_log_data_vec.addVoltageCurrentData(kr_base_voltage, kr_base_current, kr_actuator_voltages,
                                          kr_actuator_currents);
    kl_log_data_vec.addVoltageCurrentData(kl_base_voltage, kl_base_current, kl_actuator_voltages,
                                          kl_actuator_currents);
    // base_log_data_vec.addVoltageCurrentData(base_bus_voltages, base_wheel_voltages,
    //                                         base_wheel_currents);

    

    double kl_achd_solver_root_acceleration[6] = {-9.6, 0.98, 1.42, 0.0, 0.0, 0.0};
    double kl_rne_ext_wrench[7][6]{};
    double kl_rne_output_torques[7]{};
    rne_solver(&robot, robot.kinova_left->base_frame, robot.kinova_left->tool_frame,
               kl_achd_solver_root_acceleration, kl_rne_ext_wrench, kl_rne_output_torques);

    double kr_achd_solver_root_acceleration[6] = {-9.685, -1.033, 1.324, 0.0, 0.0, 0.0};
    double kr_rne_ext_wrench[7][6]{};
    double kr_rne_output_torques[7]{};
    rne_solver(&robot, robot.kinova_right->base_frame, robot.kinova_right->tool_frame,
               kr_achd_solver_root_acceleration, kr_rne_ext_wrench, kr_rne_output_torques);

    KDL::JntArray kinova_right_cmd_tau_kdl1(7);
    cap_and_convert_manipulator_torques(kr_rne_output_torques, 7, kinova_right_cmd_tau_kdl1);

    KDL::JntArray kinova_left_cmd_tau_kdl1(7);
    cap_and_convert_manipulator_torques(kl_rne_output_torques, 7, kinova_left_cmd_tau_kdl1);

    // set torques
    // set_mobile_base_torques(&robot, fd_solver_robile_output_torques);
    // set_manipulator_torques(&robot, kinova_left_base_link, &kinova_left_cmd_tau_kdl);
    // set_manipulator_torques(&robot, kinova_right_base_link, &kinova_right_cmd_tau_kdl);

    set_manipulator_torques(&robot, kinova_left.base_frame, &kinova_left_cmd_tau_kdl1);
    set_manipulator_torques(&robot, kinova_right.base_frame, &kinova_right_cmd_tau_kdl1);

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