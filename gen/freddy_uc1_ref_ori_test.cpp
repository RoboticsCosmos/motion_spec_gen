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
  Manipulator<kinova_mediator> kl;
  kl.base_frame = "kinova_left_base_link";
  kl.tool_frame = "kinova_left_bracelet_link";
  kl.mediator = new kinova_mediator();
  kl.state = new ManipulatorState();
  bool kl_torque_control_mode_set = false;

  double kl_rne_init_taus[7]{};
  Manipulator<kinova_mediator> kr;
  kr.base_frame = "kinova_right_base_link";
  kr.tool_frame = "kinova_right_bracelet_link";
  kr.mediator = new kinova_mediator();
  kr.state = new ManipulatorState();
  bool kr_torque_control_mode_set = false;

  double kr_rne_init_taus[7]{};
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

  Freddy robot = {&kl, &kr, &freddy_base};

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  char *ethernet_interface = "eno1";
  initialize_robot(robot_urdf, ethernet_interface, &robot);

  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s

  double control_loop_dt = 1.0 / desired_frequency;

  // initialize variables
  double kr_bracelet_table_contact_force_lin_z_vector_z[6] = {0, 0, 1};
  double kl_elbow_base_base_distance_z_embed_map_vector[3] = {0.0, 0.0, 1.0};
  double kr_bl_orientation_ang_x_pid_controller_signal = 0.0;
  double *kr_achd_solver_alpha[5] = {
      new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0}, new double[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, new double[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double kr_bl_position_lin_z_pid_controller_signal = 0.0;
  double kl_bl_position_coord_lin_z_initial = 0.0;
  double kl_bl_position_coord_lin_z_vector[3]{};
  std::string kinova_right_half_arm_2_link_origin_point = "kinova_right_half_arm_2_link";
  double kl_elbow_base_z_distance_reference_value = 0.9;
  double kl_bl_position_lin_z_twist_embed_map_vector[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  double kr_bracelet_table_contact_force_pid_controller_signal = 0.0;
  double kl_bl_position_lin_z_pid_controller_prev_error = 0.0;
  double kl_bl_orientation_ang_z_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  double kr_bl_orientation_ang_y_pid_controller_time_step = control_loop_dt;
  double kl_bl_position_lin_y_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
  double kr_bl_orientation_coord_ang_z = 0.0;
  double kl_bl_orientation_ang_z_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
  double kr_bl_orientation_ang_x_pid_controller_error_sum = 0.0;
  double kr_bl_orientation_coord_ang_y = 0.0;
  double kr_bl_orientation_coord_ang_x = 0.0;
  std::string kl_bracelet_link = "kl_bracelet_link";
  double kl_achd_solver_root_acceleration[6] = {-9.6, 0.99, 1.4, 0.0, 0.0, 0.0};
  double kl_bl_position_lin_z_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
  double kr_achd_solver_feed_forward_torques[7]{};
  double kl_bl_orientation_ang_y_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
  double kl_achd_solver_feed_forward_torques[7]{};
  double kr_bl_position_coord_lin_y_initial = 0.0;
  double kl_bl_position_lin_z_pid_controller_signal = 0.0;
  double kl_bracelet_table_contact_force_lin_z = 0.0;
  double kl_elbow_base_base_distance_z_impedance_controller_signal = 0.0;
  double kl_bl_orientation_ang_z_pid_controller_error_sum = 0.0;
  double kl_bl_orientation_ang_x_pid_controller_time_step = control_loop_dt;
  double kr_bl_orientation_coord_ang_y_vector[3]{};
  std::string kinova_right_base_link = "kinova_right_base_link";
  double kr_bracelet_table_contact_force_pid_controller_error_sum = 0.0;
  double kr_bl_position_coord_lin_z_vector[3]{};
  double kl_bl_orientation_coord_ang_z_initial = 0.0;
  double kr_bl_orientation_ang_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
  double kr_bl_orientation_coord_ang_y_initial_vector[3]{};
  double kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench[6]{};
  std::string kinova_left_bracelet_link_origin_point = "kinova_left_bracelet_link";
  double kr_bl_position_lin_y_pid_controller_prev_error = 0.0;
  std::string kr_bracelet_link = "kr_bracelet_link";
  double kl_bracelet_table_contact_force_embed_map_vector[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  double kr_bl_orientation_ang_y_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  double kr_bracelet_table_contact_force_lin_z = 0.0;
  double kl_achd_solver_predicted_accelerations[7]{};
  double kl_achd_solver_fext_output_torques[7]{};
  double kr_bl_position_lin_z_pid_controller_time_step = control_loop_dt;
  double kr_elbow_base_z_distance_reference_value = 0.9;
  int kl_achd_solver_nc = 5;
  double kl_bl_orientation_ang_z_pid_controller_signal = 0.0;
  double kr_bl_position_lin_y_twist_embed_map_vector[6] = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  double kl_bl_orientation_coord_ang_y_initial = 0.0;
  double kl_bracelet_table_contact_force_pid_controller_time_step = control_loop_dt;
  double kr_bl_orientation = 0.0;
  double kr_bl_position_coord_lin_z_initial_vector[3]{};
  int kr_achd_solver_nc = 5;
  double kr_bl_position_lin_z_pid_controller_error_sum = 0.0;
  int kl_achd_solver_fext_nj = 7;
  int kr_achd_solver_nj = 7;
  double kr_elbow_base_base_distance_z_impedance_controller_signal = 0.0;
  int kl_achd_solver_nj = 7;
  double kl_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench[6]{};
  double kl_bl_orientation_coord_ang_y_vector[3]{};
  double kl_elbow_base_distance_coord_lin_z_axis[6] = {0, 0, 1};
  double kl_bl_orientation_ang_z_pid_controller_time_step = control_loop_dt;
  double kr_bl_position_coord_lin_y = 0.0;
  double kr_bl_position_coord_lin_z = 0.0;
  double kl_bl_orientation_ang_x_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
  double kr_bl_orientation_ang_y_pid_controller_prev_error = 0.0;
  double kl_elbow_base_distance_coord_lin_z = 0.0;
  double arm_table_contact_force_reference_value = -10.0;
  double kl_bl_position_lin_y_pid_controller_signal = 0.0;
  double kl_bl_orientation_ang_y_pid_controller_prev_error = 0.0;
  double kr_bl_orientation_ang_x_pid_controller_prev_error = 0.0;
  double kr_bl_orientation_coord_ang_y_initial = 0.0;
  double kl_bl_position_lin_z_pid_controller_kp = 10.0;
  double kl_bl_orientation_ang_x_pid_controller_prev_error = 0.0;
  double kr_bl_orientation_ang_z_pid_controller_signal = 0.0;
  double kr_bl_orientation_coord_ang_x_initial_vector[3]{};
  double kr_bl_orientation_ang_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
  double kl_bl_orientation_ang_x_pid_controller_kd = 2.5;
  double kr_bracelet_table_contact_force_pid_controller_kd = 0.0;
  double kl_bl_position_lin_z_pid_controller_ki = 0.9;
  double kr_elbow_base_distance_coord_lin_z_axis[6] = {0, 0, 1};
  double kl_bl_orientation_ang_x_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  double kl_bracelet_table_contact_force_pid_controller_kd = 0.0;
  double kl_bl_position_lin_z_pid_controller_time_step = control_loop_dt;
  double kl_bl_orientation_coord_ang_x_initial_vector[3]{};
  double kl_bl_orientation_ang_x_pid_controller_ki = 0.9;
  double kr_bracelet_table_contact_force_pid_controller_ki = 0.9;
  double kl_bl_position_lin_z_pid_controller_kd = 1.5;
  double kl_bracelet_table_contact_force_pid_controller_kp = 20.0;
  double kl_bl_position_lin_z_pid_controller_error_sum = 0.0;
  double kr_bl_orientation_ang_z_pid_controller_prev_error = 0.0;
  double kl_bracelet_table_contact_force_pid_controller_error_sum = 0.0;
  double kr_bracelet_table_contact_force_pid_controller_prev_error = 0.0;
  double kr_bracelet_table_contact_force_pid_controller_kp = 20.0;
  double kl_bracelet_table_contact_force_pid_controller_ki = 0.9;
  double kr_bl_orientation_ang_x_pid_controller_kp = 20.0;
  double kl_elbow_base_base_distance_z_impedance_controller_stiffness_diag_mat[1] = {100.0};
  double kr_bl_orientation_ang_x_pid_controller_kd = 2.5;
  double kr_bl_orientation_ang_x_pid_controller_ki = 0.9;
  double kl_bl_orientation_ang_z_pid_controller_prev_error = 0.0;
  double kl_bl_orientation_ang_y_pid_controller_kp = 20.0;
  double kr_bracelet_table_contact_force_pid_controller_time_step = control_loop_dt;
  double kl_bl_orientation_ang_z_pid_controller_ki = 0.9;
  double kl_bl_orientation_ang_z_pid_controller_kd = 2.5;
  double kl_bl_orientation_ang_x_pid_controller_kp = 20.0;
  int kr_achd_solver_fext_nj = 7;
  double kr_bl_orientation_coord_ang_z_initial = 0.0;
  double kr_bl_position_lin_y_pid_controller_signal = 0.0;
  double kl_bl_position_coord_lin_z_initial_vector[3]{};
  double kr_bl_position_coord_lin_z_initial = 0.0;
  std::string base_link = "base_link";
  double kr_bl_orientation_coord_ang_z_vector[3]{};
  double kr_bl_orientation_coord_ang_x_vector[3]{};
  double kl_bl_orientation_coord_ang_z = 0.0;
  double kl_bl_orientation_coord_ang_y = 0.0;
  std::string kinova_right_bracelet_link_origin_point = "kinova_right_bracelet_link";
  double kl_bl_orientation_coord_ang_x = 0.0;
  double kl_bl_position_lin_y_twist_embed_map_vector[6] = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  double kr_bl_orientation_ang_x_pid_controller_time_step = control_loop_dt;
  double kr_bl_position_coord_lin_y_vector[3]{};
  double kr_bl_orientation_ang_y_pid_controller_error_sum = 0.0;
  double kr_elbow_base_base_distance_z_embed_map_vector[3] = {0.0, 0.0, 1.0};
  double kr_bl_position_lin_y_pid_controller_time_step = control_loop_dt;
  double kl_bl_orientation_ang_x_pid_controller_error_sum = 0.0;
  double kl_bl_position_coord_lin_y_initial_vector[3]{};
  double kr_bl_position_coord_lin_y_initial_vector[3]{};
  double kr_bl_position_lin_y_pid_controller_kp = 10.0;
  double kr_bl_position_lin_y_pid_controller_ki = 0.9;
  std::string table = "table";
  double kl_bracelet_table_contact_force_lin_z_vector_z[6] = {0, 0, 1};
  double kr_bl_position_lin_z_pid_controller_prev_error = 0.0;
  double kr_bl_orientation_ang_z_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  double kr_bl_position_lin_y_pid_controller_kd = 1.0;
  double kr_bl_position_lin_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
  double kl_bl_orientation_coord_ang_z_vector[3]{};
  double kr_bl_orientation_ang_y_pid_controller_signal = 0.0;
  double kl_bl_position_coord_lin_y_vector[3]{};
  double kl_bl_orientation_coord_ang_y_initial_vector[3]{};
  double kl_bl_orientation_ang_y_pid_controller_kd = 2.5;
  std::string kinova_left_bracelet_link = "kinova_left_bracelet_link";
  double kl_bracelet_table_contact_force_pid_controller_prev_error = 0.0;
  double kl_achd_solver_output_torques[7]{};
  double kl_bl_position_coord_lin_y_initial = 0.0;
  double kl_bracelet_table_contact_force_pid_controller_signal = 0.0;
  double kl_bl_orientation_ang_y_pid_controller_ki = 0.9;
  std::string kinova_left_half_arm_2_link_origin_point = "kinova_left_half_arm_2_link";
  double *kl_achd_solver_alpha[5] = {
      new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0}, new double[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, new double[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double kr_bl_orientation_ang_y_pid_controller_ki = 0.9;
  double kr_bl_orientation_ang_y_pid_controller_kd = 2.5;
  double kr_bl_orientation_ang_y_pid_controller_kp = 20.0;
  double kr_bl_orientation_ang_z_pid_controller_kd = 2.5;
  double kl_bl_orientation_ang_y_pid_controller_time_step = control_loop_dt;
  double kl_bl_position_lin_y_pid_controller_ki = 0.9;
  double kr_bl_orientation_coord_ang_x_initial = 0.0;
  double kr_bl_orientation_ang_z_pid_controller_ki = 0.9;
  double kr_achd_solver_fext_output_torques[7]{};
  double kr_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench[6]{};
  double kl_bl_orientation_ang_y_pid_controller_error_sum = 0.0;
  double kl_bl_position_lin_y_pid_controller_kd = 1.5;
  double kr_achd_solver_output_torques[7]{};
  double kr_bl_orientation_ang_z_pid_controller_kp = 20.0;
  double kl_bl_orientation_ang_y_pid_controller_signal = 0.0;
  double kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench[6]{};
  double kr_bl_position_lin_y_pid_controller_error_sum = 0.0;
  double kr_bl_orientation_ang_x_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  double kr_bl_orientation_ang_z_pid_controller_time_step = control_loop_dt;
  std::string kinova_left_base_link = "kinova_left_base_link";
  double kl_bl_position_lin_y_pid_controller_kp = 10.0;
  std::string kinova_right_bracelet_link = "kinova_right_bracelet_link";
  double kl_bl_orientation_ang_x_pid_controller_signal = 0.0;
  double kr_bl_position_lin_z_twist_embed_map_vector[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  double kr_bl_orientation_ang_z_pid_controller_error_sum = 0.0;
  double kl_bl_orientation = 0.0;
  double kr_achd_solver_predicted_accelerations[7]{};
  double kl_bl_position_lin_y_pid_controller_prev_error = 0.0;
  double kr_elbow_base_distance_coord_lin_z = 0.0;
  double kl_bl_orientation_coord_ang_z_initial_vector[3]{};
  double kr_bl_position_lin_z_pid_controller_ki = 0.9;
  double kl_bl_orientation_ang_y_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  double kr_achd_solver_root_acceleration[6] = {-9.685, -1.033, 1.324, 0.0, 0.0, 0.0};
  double kr_bl_position_lin_z_pid_controller_kp = 10.0;
  double kr_elbow_base_base_distance_z_impedance_controller_stiffness_diag_mat[1] = {100.0};
  double kl_bl_orientation_coord_ang_x_initial = 0.0;
  double kr_bl_orientation_ang_x_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
  double kr_bracelet_table_contact_force_embed_map_vector[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  double kl_bl_position_coord_lin_y = 0.0;
  double kl_bl_position_coord_lin_z = 0.0;
  double kr_bl_position_lin_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
  std::string kl_half_arm_2_link = "kl_half_arm_2_link";
  std::string kr_half_arm_2_link = "kr_half_arm_2_link";
  double kl_bl_orientation_ang_z_pid_controller_kp = 20.0;
  double kl_bl_orientation_coord_ang_x_vector[3]{};
  double kl_bl_position_lin_y_pid_controller_time_step = control_loop_dt;
  double kl_bl_position_lin_y_pid_controller_error_sum = 0.0;
  std::string base_link_origin_point = "base_link";
  double kr_bl_position_lin_z_pid_controller_kd = 1.0;
  double kr_bl_orientation_coord_ang_z_initial_vector[3]{};

  get_robot_data(&robot, control_loop_dt);

  // initial taus for manipulators during control mode switch
  double **kl_rne_ext_wrenches = new double *[7];
  init_2d_array(kl_rne_ext_wrenches, 7, 6);

  rne_solver(&robot, kl.base_frame, kl.tool_frame, kl_rne_solver_root_acc, kl_rne_ext_wrenches,
             kl_rne_init_taus);
  double **kr_rne_ext_wrenches = new double *[7];
  init_2d_array(kr_rne_ext_wrenches, 7, 6);

  rne_solver(&robot, kr.base_frame, kr.tool_frame, kr_rne_solver_root_acc, kr_rne_ext_wrenches,
             kr_rne_init_taus);

  // update compute variables
  getLinkPosition(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                  kr_bl_position_coord_lin_y_initial_vector, &robot,
                  kr_bl_position_coord_lin_y_initial);
  getLinkQuaternion(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kl_bl_orientation_coord_ang_x_initial_vector, &robot,
                    kl_bl_orientation_coord_ang_x_initial);
  getLinkQuaternion(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kr_bl_orientation_coord_ang_x_initial_vector, &robot,
                    kr_bl_orientation_coord_ang_x_initial);
  getLinkQuaternion(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kl_bl_orientation_coord_ang_y_initial_vector, &robot,
                    kl_bl_orientation_coord_ang_y_initial);
  getLinkQuaternion(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kl_bl_orientation_coord_ang_z_initial_vector, &robot,
                    kl_bl_orientation_coord_ang_z_initial);
  getLinkQuaternion(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kr_bl_orientation_coord_ang_z_initial_vector, &robot,
                    kr_bl_orientation_coord_ang_z_initial);
  getLinkQuaternion(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kr_bl_orientation_coord_ang_y_initial_vector, &robot,
                    kr_bl_orientation_coord_ang_y_initial);
  getLinkPosition(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                  kl_bl_position_coord_lin_y_initial_vector, &robot,
                  kl_bl_position_coord_lin_y_initial);
  getLinkPosition(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                  kl_bl_position_coord_lin_z_initial_vector, &robot,
                  kl_bl_position_coord_lin_z_initial);
  getLinkPosition(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                  kr_bl_position_coord_lin_z_initial_vector, &robot,
                  kr_bl_position_coord_lin_z_initial);

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
    // printf("count: %d\n", count);

    get_robot_data(&robot, control_loop_dt);

    // controllers
    // pid controller
    getLinkQuaternion(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                      kl_bl_orientation_coord_ang_x_vector, &robot, kl_bl_orientation_coord_ang_x);
    double kl_bl_orientation_ang_x_pid_controller_error = 0;
    computeEqualityError(kl_bl_orientation_coord_ang_x, kl_bl_orientation_coord_ang_x_initial,
                         kl_bl_orientation_ang_x_pid_controller_error);
    pidController(
        kl_bl_orientation_ang_x_pid_controller_error, kl_bl_orientation_ang_x_pid_controller_kp,
        kl_bl_orientation_ang_x_pid_controller_ki, kl_bl_orientation_ang_x_pid_controller_kd,
        kl_bl_orientation_ang_x_pid_controller_time_step,
        kl_bl_orientation_ang_x_pid_controller_error_sum,
        kl_bl_orientation_ang_x_pid_controller_prev_error,
        kl_bl_orientation_ang_x_pid_controller_signal);

    // pid controller
    getLinkQuaternion(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                      kr_bl_orientation_coord_ang_y_vector, &robot, kr_bl_orientation_coord_ang_y);
    double kr_bl_orientation_ang_y_pid_controller_error = 0;
    computeEqualityError(kr_bl_orientation_coord_ang_y, kr_bl_orientation_coord_ang_y_initial,
                         kr_bl_orientation_ang_y_pid_controller_error);
    pidController(
        kr_bl_orientation_ang_y_pid_controller_error, kr_bl_orientation_ang_y_pid_controller_kp,
        kr_bl_orientation_ang_y_pid_controller_ki, kr_bl_orientation_ang_y_pid_controller_kd,
        kr_bl_orientation_ang_y_pid_controller_time_step,
        kr_bl_orientation_ang_y_pid_controller_error_sum,
        kr_bl_orientation_ang_y_pid_controller_prev_error,
        kr_bl_orientation_ang_y_pid_controller_signal);

    // impedance controller
    double kl_elbow_base_base_distance_z_impedance_controller_stiffness_error = 0;
    computeDistance1D(
        new std::string[2]{kinova_left_half_arm_2_link_origin_point, base_link_origin_point},
        kl_elbow_base_distance_coord_lin_z_axis, base_link, &robot,
        kl_elbow_base_distance_coord_lin_z);
    computeEqualityError(kl_elbow_base_distance_coord_lin_z,
                         kl_elbow_base_z_distance_reference_value,
                         kl_elbow_base_base_distance_z_impedance_controller_stiffness_error);
    impedanceController(kl_elbow_base_base_distance_z_impedance_controller_stiffness_error, 0.0,
                        kl_elbow_base_base_distance_z_impedance_controller_stiffness_diag_mat,
                        new double[1]{0.0},
                        kl_elbow_base_base_distance_z_impedance_controller_signal);

    // pid controller
    getLinkForce(kl_bracelet_link, table, kl_bracelet_link,
                 kl_bracelet_table_contact_force_lin_z_vector_z, &robot,
                 kl_bracelet_table_contact_force_lin_z);
    double kl_bracelet_table_contact_force_pid_controller_error = 0;
    computeEqualityError(kl_bracelet_table_contact_force_lin_z,
                         arm_table_contact_force_reference_value,
                         kl_bracelet_table_contact_force_pid_controller_error);
    pidController(kl_bracelet_table_contact_force_pid_controller_error,
                  kl_bracelet_table_contact_force_pid_controller_kp,
                  kl_bracelet_table_contact_force_pid_controller_ki,
                  kl_bracelet_table_contact_force_pid_controller_kd,
                  kl_bracelet_table_contact_force_pid_controller_time_step,
                  kl_bracelet_table_contact_force_pid_controller_error_sum,
                  kl_bracelet_table_contact_force_pid_controller_prev_error,
                  kl_bracelet_table_contact_force_pid_controller_signal);

    // pid controller
    getLinkPosition(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kr_bl_position_coord_lin_y_vector, &robot, kr_bl_position_coord_lin_y);
    double kr_bl_position_lin_y_pid_controller_error = 0;
    computeEqualityError(kr_bl_position_coord_lin_y, kr_bl_position_coord_lin_y_initial,
                         kr_bl_position_lin_y_pid_controller_error);
    pidController(kr_bl_position_lin_y_pid_controller_error,
                  kr_bl_position_lin_y_pid_controller_kp, kr_bl_position_lin_y_pid_controller_ki,
                  kr_bl_position_lin_y_pid_controller_kd,
                  kr_bl_position_lin_y_pid_controller_time_step,
                  kr_bl_position_lin_y_pid_controller_error_sum,
                  kr_bl_position_lin_y_pid_controller_prev_error,
                  kr_bl_position_lin_y_pid_controller_signal);

    // pid controller
    getLinkQuaternion(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                      kl_bl_orientation_coord_ang_z_vector, &robot, kl_bl_orientation_coord_ang_z);
    double kl_bl_orientation_ang_z_pid_controller_error = 0;
    computeEqualityError(kl_bl_orientation_coord_ang_z, kl_bl_orientation_coord_ang_z_initial,
                         kl_bl_orientation_ang_z_pid_controller_error);
    pidController(
        kl_bl_orientation_ang_z_pid_controller_error, kl_bl_orientation_ang_z_pid_controller_kp,
        kl_bl_orientation_ang_z_pid_controller_ki, kl_bl_orientation_ang_z_pid_controller_kd,
        kl_bl_orientation_ang_z_pid_controller_time_step,
        kl_bl_orientation_ang_z_pid_controller_error_sum,
        kl_bl_orientation_ang_z_pid_controller_prev_error,
        kl_bl_orientation_ang_z_pid_controller_signal);

    // pid controller
    getLinkPosition(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kl_bl_position_coord_lin_z_vector, &robot, kl_bl_position_coord_lin_z);
    double kl_bl_position_lin_z_pid_controller_error = 0;
    computeEqualityError(kl_bl_position_coord_lin_z, kl_bl_position_coord_lin_z_initial,
                         kl_bl_position_lin_z_pid_controller_error);
    pidController(kl_bl_position_lin_z_pid_controller_error,
                  kl_bl_position_lin_z_pid_controller_kp, kl_bl_position_lin_z_pid_controller_ki,
                  kl_bl_position_lin_z_pid_controller_kd,
                  kl_bl_position_lin_z_pid_controller_time_step,
                  kl_bl_position_lin_z_pid_controller_error_sum,
                  kl_bl_position_lin_z_pid_controller_prev_error,
                  kl_bl_position_lin_z_pid_controller_signal);

    // pid controller
    getLinkQuaternion(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                      kr_bl_orientation_coord_ang_z_vector, &robot, kr_bl_orientation_coord_ang_z);
    double kr_bl_orientation_ang_z_pid_controller_error = 0;
    computeEqualityError(kr_bl_orientation_coord_ang_z, kr_bl_orientation_coord_ang_z_initial,
                         kr_bl_orientation_ang_z_pid_controller_error);
    pidController(
        kr_bl_orientation_ang_z_pid_controller_error, kr_bl_orientation_ang_z_pid_controller_kp,
        kr_bl_orientation_ang_z_pid_controller_ki, kr_bl_orientation_ang_z_pid_controller_kd,
        kr_bl_orientation_ang_z_pid_controller_time_step,
        kr_bl_orientation_ang_z_pid_controller_error_sum,
        kr_bl_orientation_ang_z_pid_controller_prev_error,
        kr_bl_orientation_ang_z_pid_controller_signal);

    // impedance controller
    double kr_elbow_base_base_distance_z_impedance_controller_stiffness_error = 0;
    computeDistance1D(
        new std::string[2]{kinova_right_half_arm_2_link_origin_point, base_link_origin_point},
        kr_elbow_base_distance_coord_lin_z_axis, base_link, &robot,
        kr_elbow_base_distance_coord_lin_z);
    computeEqualityError(kr_elbow_base_distance_coord_lin_z,
                         kr_elbow_base_z_distance_reference_value,
                         kr_elbow_base_base_distance_z_impedance_controller_stiffness_error);
    impedanceController(kr_elbow_base_base_distance_z_impedance_controller_stiffness_error, 0.0,
                        kr_elbow_base_base_distance_z_impedance_controller_stiffness_diag_mat,
                        new double[1]{0.0},
                        kr_elbow_base_base_distance_z_impedance_controller_signal);

    // pid controller
    getLinkQuaternion(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                      kl_bl_orientation_coord_ang_y_vector, &robot, kl_bl_orientation_coord_ang_y);
    double kl_bl_orientation_ang_y_pid_controller_error = 0;
    computeEqualityError(kl_bl_orientation_coord_ang_y, kl_bl_orientation_coord_ang_y_initial,
                         kl_bl_orientation_ang_y_pid_controller_error);
    pidController(
        kl_bl_orientation_ang_y_pid_controller_error, kl_bl_orientation_ang_y_pid_controller_kp,
        kl_bl_orientation_ang_y_pid_controller_ki, kl_bl_orientation_ang_y_pid_controller_kd,
        kl_bl_orientation_ang_y_pid_controller_time_step,
        kl_bl_orientation_ang_y_pid_controller_error_sum,
        kl_bl_orientation_ang_y_pid_controller_prev_error,
        kl_bl_orientation_ang_y_pid_controller_signal);

    // pid controller
    getLinkPosition(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kr_bl_position_coord_lin_z_vector, &robot, kr_bl_position_coord_lin_z);
    double kr_bl_position_lin_z_pid_controller_error = 0;
    computeEqualityError(kr_bl_position_coord_lin_z, kr_bl_position_coord_lin_z_initial,
                         kr_bl_position_lin_z_pid_controller_error);
    pidController(kr_bl_position_lin_z_pid_controller_error,
                  kr_bl_position_lin_z_pid_controller_kp, kr_bl_position_lin_z_pid_controller_ki,
                  kr_bl_position_lin_z_pid_controller_kd,
                  kr_bl_position_lin_z_pid_controller_time_step,
                  kr_bl_position_lin_z_pid_controller_error_sum,
                  kr_bl_position_lin_z_pid_controller_prev_error,
                  kr_bl_position_lin_z_pid_controller_signal);

    // pid controller
    getLinkQuaternion(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                      kr_bl_orientation_coord_ang_x_vector, &robot, kr_bl_orientation_coord_ang_x);
    double kr_bl_orientation_ang_x_pid_controller_error = 0;
    computeEqualityError(kr_bl_orientation_coord_ang_x, kr_bl_orientation_coord_ang_x_initial,
                         kr_bl_orientation_ang_x_pid_controller_error);
    pidController(
        kr_bl_orientation_ang_x_pid_controller_error, kr_bl_orientation_ang_x_pid_controller_kp,
        kr_bl_orientation_ang_x_pid_controller_ki, kr_bl_orientation_ang_x_pid_controller_kd,
        kr_bl_orientation_ang_x_pid_controller_time_step,
        kr_bl_orientation_ang_x_pid_controller_error_sum,
        kr_bl_orientation_ang_x_pid_controller_prev_error,
        kr_bl_orientation_ang_x_pid_controller_signal);

    // pid controller
    getLinkPosition(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kl_bl_position_coord_lin_y_vector, &robot, kl_bl_position_coord_lin_y);
    double kl_bl_position_lin_y_pid_controller_error = 0;
    computeEqualityError(kl_bl_position_coord_lin_y, kl_bl_position_coord_lin_y_initial,
                         kl_bl_position_lin_y_pid_controller_error);
    pidController(kl_bl_position_lin_y_pid_controller_error,
                  kl_bl_position_lin_y_pid_controller_kp, kl_bl_position_lin_y_pid_controller_ki,
                  kl_bl_position_lin_y_pid_controller_kd,
                  kl_bl_position_lin_y_pid_controller_time_step,
                  kl_bl_position_lin_y_pid_controller_error_sum,
                  kl_bl_position_lin_y_pid_controller_prev_error,
                  kl_bl_position_lin_y_pid_controller_signal);

    // pid controller
    getLinkForce(kr_bracelet_link, table, kr_bracelet_link,
                 kr_bracelet_table_contact_force_lin_z_vector_z, &robot,
                 kr_bracelet_table_contact_force_lin_z);
    double kr_bracelet_table_contact_force_pid_controller_error = 0;
    computeEqualityError(kr_bracelet_table_contact_force_lin_z,
                         arm_table_contact_force_reference_value,
                         kr_bracelet_table_contact_force_pid_controller_error);
    pidController(kr_bracelet_table_contact_force_pid_controller_error,
                  kr_bracelet_table_contact_force_pid_controller_kp,
                  kr_bracelet_table_contact_force_pid_controller_ki,
                  kr_bracelet_table_contact_force_pid_controller_kd,
                  kr_bracelet_table_contact_force_pid_controller_time_step,
                  kr_bracelet_table_contact_force_pid_controller_error_sum,
                  kr_bracelet_table_contact_force_pid_controller_prev_error,
                  kr_bracelet_table_contact_force_pid_controller_signal);

    // embed maps
    double
        kl_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench[6]{};
    double kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench[6]{};

    for (size_t i = 0; i < sizeof(kl_bracelet_table_contact_force_embed_map_vector) /
                               sizeof(kl_bracelet_table_contact_force_embed_map_vector[0]);
         i++)
    {
      if (kl_bracelet_table_contact_force_embed_map_vector[i] != 0.0)
      {
        kl_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench[i] +=
            kl_bracelet_table_contact_force_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(kl_elbow_base_base_distance_z_embed_map_vector) /
                               sizeof(kl_elbow_base_base_distance_z_embed_map_vector[0]);
         i++)
    {
      if (kl_elbow_base_base_distance_z_embed_map_vector[i] != 0.0)
      {
        kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench[i] +=
            kl_elbow_base_base_distance_z_impedance_controller_signal;
      }
    }
    double kl_bl_position_lin_y_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
    double kl_bl_position_lin_z_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
    double kl_bl_orientation_ang_x_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
    double kl_bl_orientation_ang_y_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
    double kl_bl_orientation_ang_z_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};

    for (size_t i = 0; i < sizeof(kl_bl_position_lin_y_twist_embed_map_vector) /
                               sizeof(kl_bl_position_lin_y_twist_embed_map_vector[0]);
         i++)
    {
      if (kl_bl_position_lin_y_twist_embed_map_vector[i] != 0.0)
      {
        kl_bl_position_lin_y_twist_embed_map_kl_achd_solver_output_acceleration_energy[i] +=
            kl_bl_position_lin_y_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(kl_bl_position_lin_z_twist_embed_map_vector) /
                               sizeof(kl_bl_position_lin_z_twist_embed_map_vector[0]);
         i++)
    {
      if (kl_bl_position_lin_z_twist_embed_map_vector[i] != 0.0)
      {
        kl_bl_position_lin_z_twist_embed_map_kl_achd_solver_output_acceleration_energy[i] +=
            kl_bl_position_lin_z_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(kl_bl_orientation_ang_x_twist_embed_map_vector) /
                               sizeof(kl_bl_orientation_ang_x_twist_embed_map_vector[0]);
         i++)
    {
      if (kl_bl_orientation_ang_x_twist_embed_map_vector[i] != 0.0)
      {
        kl_bl_orientation_ang_x_twist_embed_map_kl_achd_solver_output_acceleration_energy[i] +=
            kl_bl_orientation_ang_x_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(kl_bl_orientation_ang_y_twist_embed_map_vector) /
                               sizeof(kl_bl_orientation_ang_y_twist_embed_map_vector[0]);
         i++)
    {
      if (kl_bl_orientation_ang_y_twist_embed_map_vector[i] != 0.0)
      {
        kl_bl_orientation_ang_y_twist_embed_map_kl_achd_solver_output_acceleration_energy[i] +=
            kl_bl_orientation_ang_y_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(kl_bl_orientation_ang_z_twist_embed_map_vector) /
                               sizeof(kl_bl_orientation_ang_z_twist_embed_map_vector[0]);
         i++)
    {
      if (kl_bl_orientation_ang_z_twist_embed_map_vector[i] != 0.0)
      {
        kl_bl_orientation_ang_z_twist_embed_map_kl_achd_solver_output_acceleration_energy[i] +=
            kl_bl_orientation_ang_z_pid_controller_signal;
      }
    }
    double kr_bl_position_lin_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
    double kr_bl_position_lin_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
    double kr_bl_orientation_ang_x_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
    double kr_bl_orientation_ang_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
    double kr_bl_orientation_ang_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};

    for (size_t i = 0; i < sizeof(kr_bl_position_lin_y_twist_embed_map_vector) /
                               sizeof(kr_bl_position_lin_y_twist_embed_map_vector[0]);
         i++)
    {
      if (kr_bl_position_lin_y_twist_embed_map_vector[i] != 0.0)
      {
        kr_bl_position_lin_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] +=
            kr_bl_position_lin_y_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(kr_bl_position_lin_z_twist_embed_map_vector) /
                               sizeof(kr_bl_position_lin_z_twist_embed_map_vector[0]);
         i++)
    {
      if (kr_bl_position_lin_z_twist_embed_map_vector[i] != 0.0)
      {
        kr_bl_position_lin_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] +=
            kr_bl_position_lin_z_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(kr_bl_orientation_ang_x_twist_embed_map_vector) /
                               sizeof(kr_bl_orientation_ang_x_twist_embed_map_vector[0]);
         i++)
    {
      if (kr_bl_orientation_ang_x_twist_embed_map_vector[i] != 0.0)
      {
        kr_bl_orientation_ang_x_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] +=
            kr_bl_orientation_ang_x_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(kr_bl_orientation_ang_y_twist_embed_map_vector) /
                               sizeof(kr_bl_orientation_ang_y_twist_embed_map_vector[0]);
         i++)
    {
      if (kr_bl_orientation_ang_y_twist_embed_map_vector[i] != 0.0)
      {
        kr_bl_orientation_ang_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] +=
            kr_bl_orientation_ang_y_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(kr_bl_orientation_ang_z_twist_embed_map_vector) /
                               sizeof(kr_bl_orientation_ang_z_twist_embed_map_vector[0]);
         i++)
    {
      if (kr_bl_orientation_ang_z_twist_embed_map_vector[i] != 0.0)
      {
        kr_bl_orientation_ang_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] +=
            kr_bl_orientation_ang_z_pid_controller_signal;
      }
    }
    double
        kr_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench[6]{};
    double kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench[6]{};

    for (size_t i = 0; i < sizeof(kr_bracelet_table_contact_force_embed_map_vector) /
                               sizeof(kr_bracelet_table_contact_force_embed_map_vector[0]);
         i++)
    {
      if (kr_bracelet_table_contact_force_embed_map_vector[i] != 0.0)
      {
        kr_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench[i] +=
            kr_bracelet_table_contact_force_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(kr_elbow_base_base_distance_z_embed_map_vector) /
                               sizeof(kr_elbow_base_base_distance_z_embed_map_vector[0]);
         i++)
    {
      if (kr_elbow_base_base_distance_z_embed_map_vector[i] != 0.0)
      {
        kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench[i] +=
            kr_elbow_base_base_distance_z_impedance_controller_signal;
      }
    }

    // solvers
    // achd_solver_fext
    double *kl_achd_solver_fext_ext_wrenches[7];
    for (size_t i = 0; i < 7; i++)
    {
      kl_achd_solver_fext_ext_wrenches[i] = new double[6]{};
    }
    int link_id = -1;
    double
        kl_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench_transf
            [6]{};
    transform_wrench(
        &robot, kl_bracelet_link, kinova_left_base_link,
        kl_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench,
        kl_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench_transf);
    getLinkId(&robot, kinova_left_base_link, kinova_left_bracelet_link, kl_bracelet_link, link_id);
    kl_achd_solver_fext_ext_wrenches[link_id] =
        kl_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench_transf;
    double
        kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench_transf
            [6]{};
    transform_wrench(
        &robot, base_link, kinova_left_base_link,
        kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench,
        kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench_transf);
    getLinkId(&robot, kinova_left_base_link, kinova_left_bracelet_link, kl_half_arm_2_link,
              link_id);
    kl_achd_solver_fext_ext_wrenches[link_id] =
        kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench_transf;
    achd_solver_fext(&robot, kinova_left_base_link, kinova_left_bracelet_link,
                     kl_achd_solver_fext_ext_wrenches, kl_achd_solver_fext_output_torques);

    // achd_solver
    double kl_achd_solver_beta[6]{};
    add(kl_bl_position_lin_y_twist_embed_map_kl_achd_solver_output_acceleration_energy,
        kl_achd_solver_beta, kl_achd_solver_beta, 6);
    add(kl_bl_position_lin_z_twist_embed_map_kl_achd_solver_output_acceleration_energy,
        kl_achd_solver_beta, kl_achd_solver_beta, 6);
    add(kl_bl_orientation_ang_x_twist_embed_map_kl_achd_solver_output_acceleration_energy,
        kl_achd_solver_beta, kl_achd_solver_beta, 6);
    add(kl_bl_orientation_ang_y_twist_embed_map_kl_achd_solver_output_acceleration_energy,
        kl_achd_solver_beta, kl_achd_solver_beta, 6);
    add(kl_bl_orientation_ang_z_twist_embed_map_kl_achd_solver_output_acceleration_energy,
        kl_achd_solver_beta, kl_achd_solver_beta, 6);
    double *kl_achd_solver_alpha_transf[kl_achd_solver_nc];
    for (size_t i = 0; i < kl_achd_solver_nc; i++)
    {
      kl_achd_solver_alpha_transf[i] = new double[6]{};
    }
    transform_alpha(&robot, base_link, kinova_left_base_link, kl_achd_solver_alpha,
                    kl_achd_solver_nc, kl_achd_solver_alpha_transf);
    achd_solver(&robot, kinova_left_base_link, kinova_left_bracelet_link, kl_achd_solver_nc,
                kl_achd_solver_root_acceleration, kl_achd_solver_alpha_transf, kl_achd_solver_beta,
                kl_achd_solver_feed_forward_torques, kl_achd_solver_predicted_accelerations,
                kl_achd_solver_output_torques);

    // achd_solver
    double kr_achd_solver_beta[6]{};
    add(kr_bl_position_lin_y_twist_embed_map_kr_achd_solver_output_acceleration_energy,
        kr_achd_solver_beta, kr_achd_solver_beta, 6);
    add(kr_bl_position_lin_z_twist_embed_map_kr_achd_solver_output_acceleration_energy,
        kr_achd_solver_beta, kr_achd_solver_beta, 6);
    add(kr_bl_orientation_ang_x_twist_embed_map_kr_achd_solver_output_acceleration_energy,
        kr_achd_solver_beta, kr_achd_solver_beta, 6);
    add(kr_bl_orientation_ang_y_twist_embed_map_kr_achd_solver_output_acceleration_energy,
        kr_achd_solver_beta, kr_achd_solver_beta, 6);
    add(kr_bl_orientation_ang_z_twist_embed_map_kr_achd_solver_output_acceleration_energy,
        kr_achd_solver_beta, kr_achd_solver_beta, 6);
    double *kr_achd_solver_alpha_transf[kr_achd_solver_nc];
    for (size_t i = 0; i < kr_achd_solver_nc; i++)
    {
      kr_achd_solver_alpha_transf[i] = new double[6]{};
    }
    transform_alpha(&robot, base_link, kinova_right_base_link, kr_achd_solver_alpha,
                    kr_achd_solver_nc, kr_achd_solver_alpha_transf);
    achd_solver(&robot, kinova_right_base_link, kinova_right_bracelet_link, kr_achd_solver_nc,
                kr_achd_solver_root_acceleration, kr_achd_solver_alpha_transf, kr_achd_solver_beta,
                kr_achd_solver_feed_forward_torques, kr_achd_solver_predicted_accelerations,
                kr_achd_solver_output_torques);

    // achd_solver_fext
    double *kr_achd_solver_fext_ext_wrenches[7];
    for (size_t i = 0; i < 7; i++)
    {
      kr_achd_solver_fext_ext_wrenches[i] = new double[6]{};
    }
    int link_id = -1;
    double
        kr_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench_transf
            [6]{};
    transform_wrench(
        &robot, kr_bracelet_link, kinova_right_base_link,
        kr_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench,
        kr_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench_transf);
    getLinkId(&robot, kinova_right_base_link, kinova_right_bracelet_link, kr_bracelet_link,
              link_id);
    kr_achd_solver_fext_ext_wrenches[link_id] =
        kr_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench_transf;
    double
        kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench_transf
            [6]{};
    transform_wrench(
        &robot, base_link, kinova_right_base_link,
        kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench,
        kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench_transf);
    getLinkId(&robot, kinova_right_base_link, kinova_right_bracelet_link, kr_half_arm_2_link,
              link_id);
    kr_achd_solver_fext_ext_wrenches[link_id] =
        kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench_transf;
    achd_solver_fext(&robot, kinova_right_base_link, kinova_right_bracelet_link,
                     kr_achd_solver_fext_ext_wrenches, kr_achd_solver_fext_output_torques);

    // Command the torques to the robots
    double kl_cmd_tau[7]{};
    add(kl_achd_solver_output_torques, kl_cmd_tau, kl_cmd_tau, 7);
    add(kl_achd_solver_fext_output_torques, kl_cmd_tau, kl_cmd_tau, 7);
    KDL::JntArray kl_cmd_tau_kdl(7);
    cap_and_convert_torques(kl_cmd_tau, 7, kl_cmd_tau_kdl);
    if (!kl_torque_control_mode_set)
    {
      robot.kl->mediator->set_control_mode(2, kl_rne_init_taus);
      kl_torque_control_mode_set = true;
    }

    double kr_cmd_tau[7]{};
    add(kr_achd_solver_output_torques, kr_cmd_tau, kr_cmd_tau, 7);
    add(kr_achd_solver_fext_output_torques, kr_cmd_tau, kr_cmd_tau, 7);
    KDL::JntArray kr_cmd_tau_kdl(7);
    cap_and_convert_torques(kr_cmd_tau, 7, kr_cmd_tau_kdl);
    if (!kr_torque_control_mode_set)
    {
      robot.kr->mediator->set_control_mode(2, kr_rne_init_taus);
      kr_torque_control_mode_set = true;
    }

    set_manipulator_torques(&robot, kinova_left_base_link, &kl_cmd_tau_kdl);
    set_manipulator_torques(&robot, kinova_right_base_link, &kr_cmd_tau_kdl);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration<double>(end_time - start_time);

    // if the elapsed time is less than the desired period, busy wait
    while (elapsed_time < desired_period)
    {
      end_time = std::chrono::high_resolution_clock::now();
      elapsed_time = std::chrono::duration<double>(end_time - start_time);
    }

    control_loop_dt = elapsed_time.count();
  }

  free_robot_data(&robot);

  return 0;
}