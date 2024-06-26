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

int main()
{
  // Initialize the robot structs
  Manipulator<kinova_mediator> kinova_right;
  kinova_right.base_frame = "kinova_right_base_link";
  kinova_right.tool_frame = "kinova_right_bracelet_link";
  kinova_right.mediator = nullptr;
  kinova_right.state = new ManipulatorState();
  bool kinova_right_torque_control_mode_set = false;

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
  robile.ethercat_config = nullptr;
  robile.kelo_base_config = &kelo_base_config;

  freddy_base.mediator = &robile;
  freddy_base.state = new MobileBaseState();

  Manipulator<kinova_mediator> kinova_left;
  kinova_left.base_frame = "base_link";
  kinova_left.tool_frame = "bracelet_link";
  kinova_left.mediator = new kinova_mediator();
  kinova_left.state = new ManipulatorState();
  bool kinova_left_torque_control_mode_set = false;

  Freddy robot = {&kinova_left, &kinova_right, &freddy_base};

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "gen3_robotiq_2f_85.urdf").string();

  char *ethernet_interface = "eno1";
  initialize_robot(robot_urdf, ethernet_interface, &robot);

  // initialize variables
  double kinova_right_bracelet_base_ang_vel_y_pid_controller_kd = 0.0;
  double achd_solver_kinova_left_output_torques[7]{};
  double kinova_right_bracelet_base_lin_vel_z_pid_controller_signal = 0.0;
  double kinova_right_bracelet_base_ang_vel_x_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_ang_vel_z_pid_controller_prev_error = 0.0;
  double
      kinova_right_bracelet_base_ang_vel_x_embed_map_achd_solver_kinova_right_output_acceleration_energy
          [6]{};
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_error_sum = 0.0;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_signal = 0.0;
  double *achd_solver_kinova_left_alpha[6] = {
      new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double kinova_left_bracelet_base_ang_vel_x_pid_controller_error_sum = 0.0;
  double kinova_right_bracelet_base_ang_vel_y_pid_controller_kp = 20.0;
  double kinova_right_bracelet_base_vel_vector_lin_x[6] = {1, 0, 0, 0, 0, 0};
  double kinova_right_bracelet_base_ang_vel_y_pid_controller_ki = 0.0;
  std::string base_link = "base_link";
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_error_sum = 0.0;
  double
      kinova_left_bracelet_base_ang_vel_x_embed_map_achd_solver_kinova_left_output_acceleration_energy
          [6]{};
  double kinova_right_bracelet_base_ang_vel_z_pid_controller_ki = 0.0;
  double
      kinova_right_bracelet_base_lin_vel_z_embed_map_achd_solver_kinova_right_output_acceleration_energy
          [6]{};
  double kinova_left_bracelet_base_ang_vel_y_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_ang_vel_z_pid_controller_kp = 20.0;
  double arm_bracelet_link_ang_vel_reference_value = 0.0;
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_kd = 0.0;
  double achd_solver_kinova_right_predicted_accelerations[7]{};
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_ki = 0.0;
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_ang_vel_z_pid_controller_kd = 0.0;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_error_sum = 0.0;
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_kp = 20.0;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_kp = 20.0;
  double kinova_right_bracelet_base_ang_vel_y_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_vel_vector_ang_z[6] = {0, 0, 0, 0, 0, 1};
  double kinova_right_bracelet_base_vel_vector_ang_y[6] = {0, 0, 0, 0, 1, 0};
  double kinova_right_bracelet_base_vel_vector_ang_x[6] = {0, 0, 0, 1, 0, 0};
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_signal = 0.0;
  double kinova_left_bracelet_base_lin_vel_z_pid_controller_prev_error = 0.0;
  double kinova_left_bracelet_base_ang_vel_y_pid_controller_prev_error = 0.0;
  double kinova_left_bracelet_base_lin_vel_y_embed_map_vector[6] = {0.0, 1.0, 0.0,
                                                                    0.0, 0.0, 0.0};
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_prev_error = 0.0;
  double achd_solver_kinova_left_predicted_accelerations[7]{};
  double kinova_left_bracelet_base_ang_vel_z_pid_controller_kp = 20.0;
  double kinova_right_bracelet_base_lin_vel_x_embed_map_vector[6] = {1.0, 0.0, 0.0,
                                                                     0.0, 0.0, 0.0};
  double kinova_left_bracelet_base_lin_vel_z_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_kp = 20.0;
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_ki = 0.0;
  double kinova_left_bracelet_base_lin_vel_z_embed_map_vector[6] = {0.0, 0.0, 1.0,
                                                                    0.0, 0.0, 0.0};
  double kinova_left_bracelet_base_ang_vel_z_pid_controller_ki = 0.9;
  double achd_solver_kinova_left_feed_forward_torques[7]{};
  double kinova_left_bracelet_base_ang_vel_z_pid_controller_prev_error = 0.0;
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_prev_error = 0.0;
  double kinova_right_bracelet_base_ang_vel_z_pid_controller_error_sum = 0.0;
  double kinova_left_bracelet_base_ang_vel_z_pid_controller_kd = 0.0;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_prev_error = 0.0;
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_kd = 0.0;
  double kinova_left_bracelet_base_ang_vel_z_embed_map_vector[6] = {0.0, 0.0, 0.0,
                                                                    0.0, 0.0, 1.0};
  std::string kinova_right_base_link = "kinova_right_base_link";
  double kinova_right_bracelet_base_ang_vel_y_pid_controller_error_sum = 0.0;
  double achd_solver_kinova_right_root_acceleration[6] = {-9.685, -1.033, 1.324,
                                                          0.0,    0.0,    0.0};
  double kinova_right_bracelet_base_lin_vel_z_pid_controller_prev_error = 0.0;
  double kinova_right_bracelet_base_ang_vel_y_embed_map_vector[6] = {0.0, 0.0, 0.0,
                                                                     0.0, 1.0, 0.0};
  double kinova_right_bracelet_base_ang_vel_x_pid_controller_error_sum = 0.0;
  double kinova_left_bracelet_base_ang_vel_y_pid_controller_signal = 0.0;
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_prev_error = 0.0;
  double of_vel_qname_ang_y = 0.0;
  std::string kinova_left_bracelet_link = "bracelet_link";
  double of_vel_qname_ang_x = 0.0;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_ki = 0.9;
  double of_vel_qname_ang_z = 0.0;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_kd = 0.0;
  double
      kinova_left_bracelet_base_lin_vel_x_twist_embed_map_achd_solver_kinova_left_output_acceleration_energy
          [6]{};
  double kinova_left_bracelet_base_ang_vel_z_pid_controller_error_sum = 0.0;
  double kinova_left_bracelet_base_ang_vel_x_pid_controller_kp = 20.0;
  double kinova_right_bracelet_base_ang_vel_x_pid_controller_signal = 0.0;
  double kinova_left_bracelet_base_vel_vector_ang_x[6] = {0, 0, 0, 1, 0, 0};
  double kinova_right_bracelet_base_lin_vel_z_pid_controller_time_step = 1;
  double kinova_left_bracelet_base_lin_vel_x_twist_embed_map_vector[6] = {1.0, 0.0, 0.0,
                                                                          0.0, 0.0, 0.0};
  double kinova_left_bracelet_base_ang_vel_x_pid_controller_kd = 0.0;
  double kinova_right_bracelet_base_ang_vel_y_pid_controller_prev_error = 0.0;
  double kinova_right_bracelet_base_ang_vel_z_pid_controller_time_step = 1;
  double kinova_left_bracelet_base_vel_vector_ang_z[6] = {0, 0, 0, 0, 0, 1};
  double
      kinova_left_bracelet_base_ang_vel_y_embed_map_achd_solver_kinova_left_output_acceleration_energy
          [6]{};
  double kinova_left_bracelet_base_ang_vel_x_pid_controller_ki = 0.9;
  double kinova_left_bracelet_base_vel_vector_ang_y[6] = {0, 0, 0, 0, 1, 0};
  int achd_solver_kinova_right_nj = 7;
  double kinova_right_bracelet_base_ang_vel_z_embed_map_vector[6] = {0.0, 0.0, 0.0,
                                                                     0.0, 0.0, 1.0};
  double kinova_right_bracelet_base_ang_vel_y_pid_controller_signal = 0.0;
  double arm_bracelet_link_lin_vel_reference_value = 0.0;
  double
      kinova_right_bracelet_base_lin_vel_x_embed_map_achd_solver_kinova_right_output_acceleration_energy
          [6]{};
  int achd_solver_kinova_right_nc = 6;
  double kinova_right_bracelet_base_ang_vel_z_pid_controller_signal = 0.0;
  double kinova_left_bracelet_base_ang_vel_z_pid_controller_time_step = 1;
  double achd_solver_kinova_right_output_torques[7]{};
  int achd_solver_kinova_left_nc = 6;
  double
      kinova_right_bracelet_base_ang_vel_z_embed_map_achd_solver_kinova_right_output_acceleration_energy
          [6]{};
  std::string kinova_left_base_link = "base_link";
  int achd_solver_kinova_left_nj = 7;
  std::string kinova_right_bracelet_link = "kinova_right_bracelet_link";
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_error_sum = 0.0;
  double kinova_left_bracelet_base_lin_vel_z_pid_controller_error_sum = 0.0;
  double kinova_left_bracelet_base_ang_vel_y_pid_controller_ki = 0.9;
  double kinova_left_bracelet_base_ang_vel_y_pid_controller_kd = 0.0;
  double kinova_left_bracelet_base_ang_vel_y_pid_controller_kp = 20.0;
  double kinova_right_bracelet_base_ang_vel_x_embed_map_vector[6] = {0.0, 0.0, 0.0,
                                                                     1.0, 0.0, 0.0};
  double kinova_right_bracelet_base_lin_vel_z_pid_controller_error_sum = 0.0;
  double kinova_left_bracelet_base_ang_vel_x_pid_controller_prev_error = 0.0;
  double kinova_right_bracelet_base_lin_vel_z_pid_controller_kd = 0.0;
  double kinova_left_bracelet_base_ang_vel_x_pid_controller_signal = 0.0;
  double kinova_right_bracelet_base_lin_vel_y_twist_embed_map_vector[6] = {0.0, 1.0, 0.0,
                                                                           0.0, 0.0, 0.0};
  double *achd_solver_kinova_right_alpha[6] = {
      new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double kinova_right_bracelet_base_lin_vel_z_pid_controller_ki = 0.;
  double kinova_left_bracelet_base_lin_vel_z_pid_controller_ki = 0.9;
  double kinova_left_bracelet_base_ang_vel_y_embed_map_vector[6] = {0.0, 0.0, 0.0,
                                                                    0.0, 1.0, 0.0};
  double kinova_left_bracelet_base_ang_vel_x_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_lin_vel_z_pid_controller_kp = 20.0;
  double kinova_left_bracelet_base_lin_vel_z_pid_controller_kd = 0.0;
  double kinova_left_bracelet_base_ang_vel_y_pid_controller_error_sum = 0.0;
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_kp = 20.0;
  double achd_solver_kinova_left_root_acceleration[6] = {0., 0., -9.81, 0.0, 0.0, 0.0};
  double
      kinova_left_bracelet_base_lin_vel_z_embed_map_achd_solver_kinova_left_output_acceleration_energy
          [6]{};
  double kinova_left_bracelet_base_ang_vel_x_embed_map_vector[6] = {0.0, 0.0, 0.0,
                                                                    1.0, 0.0, 0.0};
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_signal = 0.0;
  double
      kinova_left_bracelet_base_ang_vel_z_embed_map_achd_solver_kinova_left_output_acceleration_energy
          [6]{};
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_ki = 0.9;
  double kinova_right_bracelet_base_vel_vector_lin_z[6] = {0, 0, 1, 0, 0, 0};
  double kinova_right_bracelet_base_vel_vector_lin_y[6] = {0, 1, 0, 0, 0, 0};
  double
      kinova_left_bracelet_base_lin_vel_y_embed_map_achd_solver_kinova_left_output_acceleration_energy
          [6]{};
  double kinova_right_bracelet_base_ang_vel_x_pid_controller_kd = 0.0;
  double kinova_left_bracelet_base_lin_vel_z_pid_controller_kp = 20.0;
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_kd = 0.0;
  double of_vel_qname_lin_x = 0.0;
  double of_vel_qname_lin_y = 0.0;
  double kinova_right_bracelet_base_ang_vel_x_pid_controller_ki = 0.0;
  double achd_solver_kinova_right_feed_forward_torques[7]{};
  double of_vel_qname_lin_z = 0.0;
  double kinova_right_bracelet_base_lin_vel_z_embed_map_vector[6] = {0.0, 0.0, 1.0,
                                                                     0.0, 0.0, 0.0};
  double kinova_right_bracelet_base_ang_vel_x_pid_controller_kp = 20.0;
  double kinova_left_bracelet_base_lin_vel_z_pid_controller_signal = 0.0;
  double
      kinova_right_bracelet_base_ang_vel_y_embed_map_achd_solver_kinova_right_output_acceleration_energy
          [6]{};
  double kinova_left_bracelet_base_vel_vector_lin_z[6] = {0, 0, 1, 0, 0, 0};
  double
      kinova_right_bracelet_base_lin_vel_y_twist_embed_map_achd_solver_kinova_right_output_acceleration_energy
          [6]{};
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_signal = 0.0;
  double kinova_right_bracelet_base_ang_vel_x_pid_controller_prev_error = 0.0;
  double kinova_left_bracelet_base_vel_vector_lin_x[6] = {1, 0, 0, 0, 0, 0};
  double kinova_left_bracelet_base_vel_vector_lin_y[6] = {0, 1, 0, 0, 0, 0};
  double kinova_left_bracelet_base_ang_vel_z_pid_controller_signal = 0.0;

  int count = 0;

  const double desired_frequency = 900.0;  // Hz
  const auto desired_period =
      std::chrono::duration<double>(1.0 / desired_frequency);  // s

  while (true)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    count++;
    std::cout << "count: " << count << std::endl;

    get_robot_data(&robot);

    // print_robot_data(&robot);

    // controllers
    // pid controller
    getLinkVelocity(kinova_left_bracelet_link, base_link, base_link,
                    kinova_left_bracelet_base_vel_vector_lin_y, &robot,
                    of_vel_qname_lin_y);
    double kinova_left_bracelet_base_lin_vel_y_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_lin_y, arm_bracelet_link_lin_vel_reference_value,
                         kinova_left_bracelet_base_lin_vel_y_pid_controller_error);
    pidController(kinova_left_bracelet_base_lin_vel_y_pid_controller_error,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_kp,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_ki,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_kd,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_time_step,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_error_sum,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_prev_error,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_signal);

    // // pid controller
    // getLinkVelocity(kinova_right_bracelet_link, base_link, base_link,
    //                 kinova_right_bracelet_base_vel_vector_lin_x, &robot,
    //                 of_vel_qname_lin_x);
    // double kinova_right_bracelet_base_lin_vel_x_pid_controller_error = 0;
    // computeEqualityError(of_vel_qname_lin_x, arm_bracelet_link_lin_vel_reference_value,
    //                      kinova_right_bracelet_base_lin_vel_x_pid_controller_error);
    // pidController(kinova_right_bracelet_base_lin_vel_x_pid_controller_error,
    //               kinova_right_bracelet_base_lin_vel_x_pid_controller_kp,
    //               kinova_right_bracelet_base_lin_vel_x_pid_controller_ki,
    //               kinova_right_bracelet_base_lin_vel_x_pid_controller_kd,
    //               kinova_right_bracelet_base_lin_vel_x_pid_controller_time_step,
    //               kinova_right_bracelet_base_lin_vel_x_pid_controller_error_sum,
    //               kinova_right_bracelet_base_lin_vel_x_pid_controller_prev_error,
    //               kinova_right_bracelet_base_lin_vel_x_pid_controller_signal);

    // // pid controller
    // getLinkVelocity(kinova_right_bracelet_link, base_link, base_link,
    //                 kinova_right_bracelet_base_vel_vector_lin_y, &robot,
    //                 of_vel_qname_lin_y);
    // double kinova_right_bracelet_base_lin_vel_y_pid_controller_error = 0;
    // computeEqualityError(of_vel_qname_lin_y, arm_bracelet_link_lin_vel_reference_value,
    //                      kinova_right_bracelet_base_lin_vel_y_pid_controller_error);
    // pidController(kinova_right_bracelet_base_lin_vel_y_pid_controller_error,
    //               kinova_right_bracelet_base_lin_vel_y_pid_controller_kp,
    //               kinova_right_bracelet_base_lin_vel_y_pid_controller_ki,
    //               kinova_right_bracelet_base_lin_vel_y_pid_controller_kd,
    //               kinova_right_bracelet_base_lin_vel_y_pid_controller_time_step,
    //               kinova_right_bracelet_base_lin_vel_y_pid_controller_error_sum,
    //               kinova_right_bracelet_base_lin_vel_y_pid_controller_prev_error,
    //               kinova_right_bracelet_base_lin_vel_y_pid_controller_signal);

    // pid controller
    getLinkVelocity(kinova_left_bracelet_link, base_link, base_link,
                    kinova_left_bracelet_base_vel_vector_ang_x, &robot,
                    of_vel_qname_ang_x);
    double kinova_left_bracelet_base_ang_vel_x_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_ang_x, arm_bracelet_link_ang_vel_reference_value,
                         kinova_left_bracelet_base_ang_vel_x_pid_controller_error);
    pidController(kinova_left_bracelet_base_ang_vel_x_pid_controller_error,
                  kinova_left_bracelet_base_ang_vel_x_pid_controller_kp,
                  kinova_left_bracelet_base_ang_vel_x_pid_controller_ki,
                  kinova_left_bracelet_base_ang_vel_x_pid_controller_kd,
                  kinova_left_bracelet_base_ang_vel_x_pid_controller_time_step,
                  kinova_left_bracelet_base_ang_vel_x_pid_controller_error_sum,
                  kinova_left_bracelet_base_ang_vel_x_pid_controller_prev_error,
                  kinova_left_bracelet_base_ang_vel_x_pid_controller_signal);

    // pid controller
    getLinkVelocity(kinova_left_bracelet_link, base_link, base_link,
                    kinova_left_bracelet_base_vel_vector_ang_z, &robot,
                    of_vel_qname_ang_z);
    double kinova_left_bracelet_base_ang_vel_z_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_ang_z, arm_bracelet_link_ang_vel_reference_value,
                         kinova_left_bracelet_base_ang_vel_z_pid_controller_error);
    pidController(kinova_left_bracelet_base_ang_vel_z_pid_controller_error,
                  kinova_left_bracelet_base_ang_vel_z_pid_controller_kp,
                  kinova_left_bracelet_base_ang_vel_z_pid_controller_ki,
                  kinova_left_bracelet_base_ang_vel_z_pid_controller_kd,
                  kinova_left_bracelet_base_ang_vel_z_pid_controller_time_step,
                  kinova_left_bracelet_base_ang_vel_z_pid_controller_error_sum,
                  kinova_left_bracelet_base_ang_vel_z_pid_controller_prev_error,
                  kinova_left_bracelet_base_ang_vel_z_pid_controller_signal);

    // pid controller
    getLinkVelocity(kinova_left_bracelet_link, base_link, base_link,
                    kinova_left_bracelet_base_vel_vector_ang_y, &robot,
                    of_vel_qname_ang_y);
    double kinova_left_bracelet_base_ang_vel_y_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_ang_y, arm_bracelet_link_ang_vel_reference_value,
                         kinova_left_bracelet_base_ang_vel_y_pid_controller_error);
    pidController(kinova_left_bracelet_base_ang_vel_y_pid_controller_error,
                  kinova_left_bracelet_base_ang_vel_y_pid_controller_kp,
                  kinova_left_bracelet_base_ang_vel_y_pid_controller_ki,
                  kinova_left_bracelet_base_ang_vel_y_pid_controller_kd,
                  kinova_left_bracelet_base_ang_vel_y_pid_controller_time_step,
                  kinova_left_bracelet_base_ang_vel_y_pid_controller_error_sum,
                  kinova_left_bracelet_base_ang_vel_y_pid_controller_prev_error,
                  kinova_left_bracelet_base_ang_vel_y_pid_controller_signal);

    // // pid controller
    // getLinkVelocity(kinova_right_bracelet_link, base_link, base_link,
    //                 kinova_right_bracelet_base_vel_vector_ang_y, &robot,
    //                 of_vel_qname_ang_y);
    // double kinova_right_bracelet_base_ang_vel_y_pid_controller_error = 0;
    // computeEqualityError(of_vel_qname_ang_y, arm_bracelet_link_ang_vel_reference_value,
    //                      kinova_right_bracelet_base_ang_vel_y_pid_controller_error);
    // pidController(kinova_right_bracelet_base_ang_vel_y_pid_controller_error,
    //               kinova_right_bracelet_base_ang_vel_y_pid_controller_kp,
    //               kinova_right_bracelet_base_ang_vel_y_pid_controller_ki,
    //               kinova_right_bracelet_base_ang_vel_y_pid_controller_kd,
    //               kinova_right_bracelet_base_ang_vel_y_pid_controller_time_step,
    //               kinova_right_bracelet_base_ang_vel_y_pid_controller_error_sum,
    //               kinova_right_bracelet_base_ang_vel_y_pid_controller_prev_error,
    //               kinova_right_bracelet_base_ang_vel_y_pid_controller_signal);

    // // pid controller
    // getLinkVelocity(kinova_right_bracelet_link, base_link, base_link,
    //                 kinova_right_bracelet_base_vel_vector_lin_z, &robot,
    //                 of_vel_qname_lin_z);
    // double kinova_right_bracelet_base_lin_vel_z_pid_controller_error = 0;
    // computeEqualityError(of_vel_qname_lin_z, arm_bracelet_link_lin_vel_reference_value,
    //                      kinova_right_bracelet_base_lin_vel_z_pid_controller_error);
    // pidController(kinova_right_bracelet_base_lin_vel_z_pid_controller_error,
    //               kinova_right_bracelet_base_lin_vel_z_pid_controller_kp,
    //               kinova_right_bracelet_base_lin_vel_z_pid_controller_ki,
    //               kinova_right_bracelet_base_lin_vel_z_pid_controller_kd,
    //               kinova_right_bracelet_base_lin_vel_z_pid_controller_time_step,
    //               kinova_right_bracelet_base_lin_vel_z_pid_controller_error_sum,
    //               kinova_right_bracelet_base_lin_vel_z_pid_controller_prev_error,
    //               kinova_right_bracelet_base_lin_vel_z_pid_controller_signal);

    // // pid controller
    // getLinkVelocity(kinova_right_bracelet_link, base_link, base_link,
    //                 kinova_right_bracelet_base_vel_vector_ang_z, &robot,
    //                 of_vel_qname_ang_z);
    // double kinova_right_bracelet_base_ang_vel_z_pid_controller_error = 0;
    // computeEqualityError(of_vel_qname_ang_z, arm_bracelet_link_ang_vel_reference_value,
    //                      kinova_right_bracelet_base_ang_vel_z_pid_controller_error);
    // pidController(kinova_right_bracelet_base_ang_vel_z_pid_controller_error,
    //               kinova_right_bracelet_base_ang_vel_z_pid_controller_kp,
    //               kinova_right_bracelet_base_ang_vel_z_pid_controller_ki,
    //               kinova_right_bracelet_base_ang_vel_z_pid_controller_kd,
    //               kinova_right_bracelet_base_ang_vel_z_pid_controller_time_step,
    //               kinova_right_bracelet_base_ang_vel_z_pid_controller_error_sum,
    //               kinova_right_bracelet_base_ang_vel_z_pid_controller_prev_error,
    //               kinova_right_bracelet_base_ang_vel_z_pid_controller_signal);

    // pid controller
    getLinkVelocity(kinova_left_bracelet_link, base_link, base_link,
                    kinova_left_bracelet_base_vel_vector_lin_z, &robot,
                    of_vel_qname_lin_z);
    double kinova_left_bracelet_base_lin_vel_z_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_lin_z, arm_bracelet_link_lin_vel_reference_value,
                         kinova_left_bracelet_base_lin_vel_z_pid_controller_error);
    pidController(kinova_left_bracelet_base_lin_vel_z_pid_controller_error,
                  kinova_left_bracelet_base_lin_vel_z_pid_controller_kp,
                  kinova_left_bracelet_base_lin_vel_z_pid_controller_ki,
                  kinova_left_bracelet_base_lin_vel_z_pid_controller_kd,
                  kinova_left_bracelet_base_lin_vel_z_pid_controller_time_step,
                  kinova_left_bracelet_base_lin_vel_z_pid_controller_error_sum,
                  kinova_left_bracelet_base_lin_vel_z_pid_controller_prev_error,
                  kinova_left_bracelet_base_lin_vel_z_pid_controller_signal);

    // // pid controller
    // getLinkVelocity(kinova_right_bracelet_link, base_link, base_link,
    //                 kinova_right_bracelet_base_vel_vector_ang_x, &robot,
    //                 of_vel_qname_ang_x);
    // double kinova_right_bracelet_base_ang_vel_x_pid_controller_error = 0;
    // computeEqualityError(of_vel_qname_ang_x, arm_bracelet_link_ang_vel_reference_value,
    //                      kinova_right_bracelet_base_ang_vel_x_pid_controller_error);
    // pidController(kinova_right_bracelet_base_ang_vel_x_pid_controller_error,
    //               kinova_right_bracelet_base_ang_vel_x_pid_controller_kp,
    //               kinova_right_bracelet_base_ang_vel_x_pid_controller_ki,
    //               kinova_right_bracelet_base_ang_vel_x_pid_controller_kd,
    //               kinova_right_bracelet_base_ang_vel_x_pid_controller_time_step,
    //               kinova_right_bracelet_base_ang_vel_x_pid_controller_error_sum,
    //               kinova_right_bracelet_base_ang_vel_x_pid_controller_prev_error,
    //               kinova_right_bracelet_base_ang_vel_x_pid_controller_signal);

    // pid controller
    getLinkVelocity(kinova_left_bracelet_link, base_link, base_link,
                    kinova_left_bracelet_base_vel_vector_lin_x, &robot,
                    of_vel_qname_lin_x);
    double kinova_left_bracelet_base_lin_vel_x_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_lin_x, arm_bracelet_link_lin_vel_reference_value,
                         kinova_left_bracelet_base_lin_vel_x_pid_controller_error);
    pidController(kinova_left_bracelet_base_lin_vel_x_pid_controller_error,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_kp,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_ki,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_kd,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_time_step,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_error_sum,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_prev_error,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_signal);

    // // kinova_left_signals
    // std::cout << "kinova_left_signals: ";
    // std::cout << kinova_left_bracelet_base_lin_vel_x_pid_controller_signal << " "
    //           << kinova_left_bracelet_base_lin_vel_y_pid_controller_signal << " "
    //           << kinova_left_bracelet_base_lin_vel_z_pid_controller_signal << " "
    //           << kinova_left_bracelet_base_ang_vel_x_pid_controller_signal << " "
    //           << kinova_left_bracelet_base_ang_vel_y_pid_controller_signal << " "
    //           << kinova_left_bracelet_base_ang_vel_z_pid_controller_signal <<
    //           std::endl;

    // embed maps
    // double
    // kinova_right_bracelet_base_lin_vel_x_embed_map_achd_solver_kinova_right_output_acceleration_energy[6]{};
    // double
    // kinova_right_bracelet_base_lin_vel_y_twist_embed_map_achd_solver_kinova_right_output_acceleration_energy[6]{};
    // double
    // kinova_right_bracelet_base_lin_vel_z_embed_map_achd_solver_kinova_right_output_acceleration_energy[6]{};
    // double
    // kinova_right_bracelet_base_ang_vel_x_embed_map_achd_solver_kinova_right_output_acceleration_energy[6]{};
    // double
    // kinova_right_bracelet_base_ang_vel_y_embed_map_achd_solver_kinova_right_output_acceleration_energy[6]{};
    // double
    // kinova_right_bracelet_base_ang_vel_z_embed_map_achd_solver_kinova_right_output_acceleration_energy[6]{};

    // for (size_t i = 0;
    //      i < sizeof(kinova_right_bracelet_base_lin_vel_x_embed_map_vector) /
    //              sizeof(kinova_right_bracelet_base_lin_vel_x_embed_map_vector[0]);
    //      i++)
    // {
    //   if (kinova_right_bracelet_base_lin_vel_x_embed_map_vector[i] != 0.0)
    //   {
    //     kinova_right_bracelet_base_lin_vel_x_embed_map_achd_solver_kinova_right_output_acceleration_energy
    //         [i] += kinova_right_bracelet_base_lin_vel_x_pid_controller_signal;
    //   }
    // }
    // for (size_t i = 0;
    //      i < sizeof(kinova_right_bracelet_base_lin_vel_y_twist_embed_map_vector) /
    //              sizeof(kinova_right_bracelet_base_lin_vel_y_twist_embed_map_vector[0]);
    //      i++)
    // {
    //   if (kinova_right_bracelet_base_lin_vel_y_twist_embed_map_vector[i] != 0.0)
    //   {
    //     kinova_right_bracelet_base_lin_vel_y_twist_embed_map_achd_solver_kinova_right_output_acceleration_energy
    //         [i] += kinova_right_bracelet_base_lin_vel_y_pid_controller_signal;
    //   }
    // }
    // for (size_t i = 0;
    //      i < sizeof(kinova_right_bracelet_base_lin_vel_z_embed_map_vector) /
    //              sizeof(kinova_right_bracelet_base_lin_vel_z_embed_map_vector[0]);
    //      i++)
    // {
    //   if (kinova_right_bracelet_base_lin_vel_z_embed_map_vector[i] != 0.0)
    //   {
    //     kinova_right_bracelet_base_lin_vel_z_embed_map_achd_solver_kinova_right_output_acceleration_energy
    //         [i] += kinova_right_bracelet_base_lin_vel_z_pid_controller_signal;
    //   }
    // }
    // for (size_t i = 0;
    //      i < sizeof(kinova_right_bracelet_base_ang_vel_x_embed_map_vector) /
    //              sizeof(kinova_right_bracelet_base_ang_vel_x_embed_map_vector[0]);
    //      i++)
    // {
    //   if (kinova_right_bracelet_base_ang_vel_x_embed_map_vector[i] != 0.0)
    //   {
    //     kinova_right_bracelet_base_ang_vel_x_embed_map_achd_solver_kinova_right_output_acceleration_energy
    //         [i] += kinova_right_bracelet_base_ang_vel_x_pid_controller_signal;
    //   }
    // }
    // for (size_t i = 0;
    //      i < sizeof(kinova_right_bracelet_base_ang_vel_y_embed_map_vector) /
    //              sizeof(kinova_right_bracelet_base_ang_vel_y_embed_map_vector[0]);
    //      i++)
    // {
    //   if (kinova_right_bracelet_base_ang_vel_y_embed_map_vector[i] != 0.0)
    //   {
    //     kinova_right_bracelet_base_ang_vel_y_embed_map_achd_solver_kinova_right_output_acceleration_energy
    //         [i] += kinova_right_bracelet_base_ang_vel_y_pid_controller_signal;
    //   }
    // }
    // for (size_t i = 0;
    //      i < sizeof(kinova_right_bracelet_base_ang_vel_z_embed_map_vector) /
    //              sizeof(kinova_right_bracelet_base_ang_vel_z_embed_map_vector[0]);
    //      i++)
    // {
    //   if (kinova_right_bracelet_base_ang_vel_z_embed_map_vector[i] != 0.0)
    //   {
    //     kinova_right_bracelet_base_ang_vel_z_embed_map_achd_solver_kinova_right_output_acceleration_energy
    //         [i] += kinova_right_bracelet_base_ang_vel_z_pid_controller_signal;
    //   }
    // }

    double
        kinova_left_bracelet_base_lin_vel_x_twist_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [6]{};
    double
        kinova_left_bracelet_base_lin_vel_y_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [6]{};
    double
        kinova_left_bracelet_base_lin_vel_z_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [6]{};
    double
        kinova_left_bracelet_base_ang_vel_x_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [6]{};
    double
        kinova_left_bracelet_base_ang_vel_y_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [6]{};
    double
        kinova_left_bracelet_base_ang_vel_z_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [6]{};

    for (size_t i = 0;
         i < sizeof(kinova_left_bracelet_base_lin_vel_x_twist_embed_map_vector) /
                 sizeof(kinova_left_bracelet_base_lin_vel_x_twist_embed_map_vector[0]);
         i++)
    {
      if (kinova_left_bracelet_base_lin_vel_x_twist_embed_map_vector[i] != 0.0)
      {
        kinova_left_bracelet_base_lin_vel_x_twist_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [i] += kinova_left_bracelet_base_lin_vel_x_pid_controller_signal;
      }
    }
    for (size_t i = 0;
         i < sizeof(kinova_left_bracelet_base_lin_vel_y_embed_map_vector) /
                 sizeof(kinova_left_bracelet_base_lin_vel_y_embed_map_vector[0]);
         i++)
    {
      if (kinova_left_bracelet_base_lin_vel_y_embed_map_vector[i] != 0.0)
      {
        kinova_left_bracelet_base_lin_vel_y_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [i] += kinova_left_bracelet_base_lin_vel_y_pid_controller_signal;
      }
    }
    for (size_t i = 0;
         i < sizeof(kinova_left_bracelet_base_lin_vel_z_embed_map_vector) /
                 sizeof(kinova_left_bracelet_base_lin_vel_z_embed_map_vector[0]);
         i++)
    {
      if (kinova_left_bracelet_base_lin_vel_z_embed_map_vector[i] != 0.0)
      {
        kinova_left_bracelet_base_lin_vel_z_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [i] += kinova_left_bracelet_base_lin_vel_z_pid_controller_signal;
      }
    }
    for (size_t i = 0;
         i < sizeof(kinova_left_bracelet_base_ang_vel_x_embed_map_vector) /
                 sizeof(kinova_left_bracelet_base_ang_vel_x_embed_map_vector[0]);
         i++)
    {
      if (kinova_left_bracelet_base_ang_vel_x_embed_map_vector[i] != 0.0)
      {
        kinova_left_bracelet_base_ang_vel_x_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [i] += kinova_left_bracelet_base_ang_vel_x_pid_controller_signal;
      }
    }
    for (size_t i = 0;
         i < sizeof(kinova_left_bracelet_base_ang_vel_y_embed_map_vector) /
                 sizeof(kinova_left_bracelet_base_ang_vel_y_embed_map_vector[0]);
         i++)
    {
      if (kinova_left_bracelet_base_ang_vel_y_embed_map_vector[i] != 0.0)
      {
        kinova_left_bracelet_base_ang_vel_y_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [i] += kinova_left_bracelet_base_ang_vel_y_pid_controller_signal;
      }
    }
    for (size_t i = 0;
         i < sizeof(kinova_left_bracelet_base_ang_vel_z_embed_map_vector) /
                 sizeof(kinova_left_bracelet_base_ang_vel_z_embed_map_vector[0]);
         i++)
    {
      if (kinova_left_bracelet_base_ang_vel_z_embed_map_vector[i] != 0.0)
      {
        kinova_left_bracelet_base_ang_vel_z_embed_map_achd_solver_kinova_left_output_acceleration_energy
            [i] += kinova_left_bracelet_base_ang_vel_z_pid_controller_signal;
      }
    }

    // solvers
    // achd_solver
    // double achd_solver_kinova_right_beta[6]{};

    // add(kinova_right_bracelet_base_lin_vel_x_embed_map_achd_solver_kinova_right_output_acceleration_energy,
    //     achd_solver_kinova_right_beta, achd_solver_kinova_right_beta, 6);
    // add(kinova_right_bracelet_base_lin_vel_y_twist_embed_map_achd_solver_kinova_right_output_acceleration_energy,
    //     achd_solver_kinova_right_beta, achd_solver_kinova_right_beta, 6);
    // add(kinova_right_bracelet_base_lin_vel_z_embed_map_achd_solver_kinova_right_output_acceleration_energy,
    //     achd_solver_kinova_right_beta, achd_solver_kinova_right_beta, 6);
    // add(kinova_right_bracelet_base_ang_vel_x_embed_map_achd_solver_kinova_right_output_acceleration_energy,
    //     achd_solver_kinova_right_beta, achd_solver_kinova_right_beta, 6);
    // add(kinova_right_bracelet_base_ang_vel_y_embed_map_achd_solver_kinova_right_output_acceleration_energy,
    //     achd_solver_kinova_right_beta, achd_solver_kinova_right_beta, 6);
    // add(kinova_right_bracelet_base_ang_vel_z_embed_map_achd_solver_kinova_right_output_acceleration_energy,
    //     achd_solver_kinova_right_beta, achd_solver_kinova_right_beta, 6);
    // double *achd_solver_kinova_right_alpha_transf[achd_solver_kinova_right_nc];
    // for (size_t i = 0; i < achd_solver_kinova_right_nc; i++)
    // {
    //   achd_solver_kinova_right_alpha_transf[i] = new double[6]{};
    // }
    // transform_alpha_beta(&robot, base_link, kinova_right_base_link,
    //                      achd_solver_kinova_right_alpha, achd_solver_kinova_right_beta,
    //                      achd_solver_kinova_right_nc,
    //                      achd_solver_kinova_right_alpha_transf,
    //                      achd_solver_kinova_right_beta);
    // for (size_t i = 0; i < 6; i++)
    // {
    //   achd_solver_kinova_right_beta[i] +=
    //   -achd_solver_kinova_right_root_acceleration[i];
    // }
    // achd_solver(&robot, kinova_right_base_link, kinova_right_bracelet_link,
    //             achd_solver_kinova_right_nc,
    //             achd_solver_kinova_right_root_acceleration,
    //             achd_solver_kinova_right_alpha_transf, achd_solver_kinova_right_beta,
    //             achd_solver_kinova_right_feed_forward_torques,
    //             achd_solver_kinova_right_predicted_accelerations,
    //             achd_solver_kinova_right_output_torques);

    // achd_solver
    double achd_solver_kinova_left_beta[6]{};
    add(kinova_left_bracelet_base_lin_vel_x_twist_embed_map_achd_solver_kinova_left_output_acceleration_energy,
        achd_solver_kinova_left_beta, achd_solver_kinova_left_beta, 6);
    add(kinova_left_bracelet_base_lin_vel_y_embed_map_achd_solver_kinova_left_output_acceleration_energy,
        achd_solver_kinova_left_beta, achd_solver_kinova_left_beta, 6);
    add(kinova_left_bracelet_base_lin_vel_z_embed_map_achd_solver_kinova_left_output_acceleration_energy,
        achd_solver_kinova_left_beta, achd_solver_kinova_left_beta, 6);
    add(kinova_left_bracelet_base_ang_vel_x_embed_map_achd_solver_kinova_left_output_acceleration_energy,
        achd_solver_kinova_left_beta, achd_solver_kinova_left_beta, 6);
    add(kinova_left_bracelet_base_ang_vel_y_embed_map_achd_solver_kinova_left_output_acceleration_energy,
        achd_solver_kinova_left_beta, achd_solver_kinova_left_beta, 6);
    add(kinova_left_bracelet_base_ang_vel_z_embed_map_achd_solver_kinova_left_output_acceleration_energy,
        achd_solver_kinova_left_beta, achd_solver_kinova_left_beta, 6);

    std::cout << "control signal: ";
    for (size_t i = 0; i < 6; i++)
    {
      std::cout << achd_solver_kinova_left_beta[i] << " ";
    }
    std::cout << std::endl;

    // double **achd_solver_kinova_left_alpha_trasnf = new double *[6];
    // for (size_t i = 0; i < 6; i++)
    // {
    //   achd_solver_kinova_left_alpha_trasnf[i] = new double[6];
    // }
    // double *achd_solver_kinova_left_beta_transf = new double[6];
    // transform_alpha_beta(&robot, base_link, kinova_left_base_link,
    //                      achd_solver_kinova_left_alpha, achd_solver_kinova_left_beta,
    //                      achd_solver_kinova_left_nc,
    //                      achd_solver_kinova_left_alpha_trasnf,
    //                      achd_solver_kinova_left_beta_transf);
    for (size_t i = 0; i < 6; i++)
    {
      achd_solver_kinova_left_beta[i] += -achd_solver_kinova_left_root_acceleration[i];
    }
    std::cout << "control signal (g): ";
    for (size_t i = 0; i < 6; i++)
    {
      std::cout << achd_solver_kinova_left_beta[i] << " ";
    }
    std::cout << std::endl;
    achd_solver(&robot, kinova_left_base_link, kinova_left_bracelet_link,
                achd_solver_kinova_left_nc, achd_solver_kinova_left_root_acceleration,
                achd_solver_kinova_left_alpha, achd_solver_kinova_left_beta,
                achd_solver_kinova_left_feed_forward_torques,
                achd_solver_kinova_left_predicted_accelerations,
                achd_solver_kinova_left_output_torques);

    std::cout << "[achd] kinova left torques: ";
    for (size_t i = 0; i < 7; i++)
    {
      std::cout << achd_solver_kinova_left_output_torques[i] << " ";
    }
    std::cout << std::endl;

    // Command the torques to the robots
    // double kinova_right_cmd_tau[7]{};
    // add(achd_solver_kinova_right_output_torques, kinova_right_cmd_tau,
    //     kinova_right_cmd_tau, 7);
    // KDL::JntArray kinova_right_cmd_tau_kdl(7);
    // cap_and_convert_torques(kinova_right_cmd_tau, 7, kinova_right_cmd_tau_kdl);

    double kinova_left_cmd_tau[7]{};
    add(achd_solver_kinova_left_output_torques, kinova_left_cmd_tau, kinova_left_cmd_tau,
        7);
    KDL::JntArray kinova_left_cmd_tau_kdl(7);
    cap_and_convert_torques(kinova_left_cmd_tau, 7, kinova_left_cmd_tau_kdl);

    // std::cout << "kinova_left_cmd_tau_kdl: " << kinova_left_cmd_tau_kdl << std::endl;

    // if (!kinova_right_torque_control_mode_set)
    // {
    //   robot.kinova_right->mediator->set_control_mode(2);
    //   kinova_right_torque_control_mode_set = true;
    // }
    // set_manipulator_torques(&robot, kinova_right_base_link, &kinova_right_cmd_tau_kdl);

    if (!kinova_left_torque_control_mode_set)
    {
      robot.kinova_left->mediator->set_control_mode(2, nullptr);
      kinova_left_torque_control_mode_set = true;
    }
    set_manipulator_torques(&robot, kinova_left_base_link, &kinova_left_cmd_tau_kdl);

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

  free_robot_data(&robot);

  return 0;
}