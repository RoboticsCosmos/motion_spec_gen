#include <array>
#include <controllers/pid_controller.hpp>
#include <filesystem>
#include <iostream>
#include <kinova_mediator/mediator.hpp>
#include <motion_spec_utils/math_utils.hpp>
#include <motion_spec_utils/solver_utils.hpp>
#include <motion_spec_utils/utils.hpp>
#include <string>

#include "kelo_motion_control/mediator.h"

volatile sig_atomic_t flag = 0;

void handle_signal(int sig)
{
  flag = 1;
  printf("Caught signal %d\n", sig);
}

int main()
{
  // handle signals
  // struct sigaction sa;
  // sa.sa_handler = handle_signal;
  // sigemptyset(&sa.sa_mask);
  // sa.sa_flags = 0;

  // for (int i = 1; i < NSIG; ++i)
  // {
  //   if (sigaction(i, &sa, NULL) == -1)
  //   {
  //     perror("sigaction");
  //   }
  // }

  // Initialize the robot structs
  Manipulator<kinova_mediator> kinova_right;
  kinova_right.base_frame = "kinova_right_base_link";
  kinova_right.tool_frame = "kinova_right_bracelet_link";
  kinova_right.mediator = new kinova_mediator();
  kinova_right.state = new ManipulatorState();
  bool kinova_right_torque_control_mode_set = false;

  KeloBaseConfig kelo_base_config;
  kelo_base_config.nWheels = 4;
  kelo_base_config.index_to_EtherCAT = new int[4]{6, 7, 3, 4};
  kelo_base_config.radius = 0.115 / 2;
  kelo_base_config.castor_offset = 0.01;
  kelo_base_config.half_wheel_distance = 0.0775 / 2;
  kelo_base_config.wheel_coordinates =
      new double[8]{0.188, 0.2075, -0.188, 0.2075, -0.188, -0.2075, 0.188, -0.2075};
  kelo_base_config.pivot_angles_deviation = new double[4]{5.310, 5.533, 1.563, 1.625};

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

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  char *ethernet_interface = "eno1";
  initialize_robot(robot_urdf, ethernet_interface, &robot);

  // initialize variables
  double achd_solver_kinova_left_output_torques[7]{};
  double kinova_left_bracelet_table_contact_force_lin_z_vector_z[6] = {0, 0, 1};
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_error_sum;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_signal;
  double *achd_solver_kinova_left_alpha[2] = {new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                              new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0}};
  double kinova_right_bracelet_table_contact_force_pid_controller_error_sum;
  int achd_solver_fext_kinova_right_nj = 7;
  double kinova_right_bracelet_base_distance_impedance_controller_signal;
  double kinova_left_bracelet_table_contact_force_pid_controller_kp = 20.0;
  double kinova_right_bracelet_base_vel_vector_lin_x[6] = {1, 0, 0, 0, 0, 0};
  double kinova_left_bracelet_table_contact_force_pid_controller_ki = 0.9;
  double fd_solver_robile_root_acceleration[6] = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  double arm_bracelet_link_lin_xy_vel_reference_value = -10.0;
  std::string base_link = "base_link";
  double kinova_left_bracelet_table_contact_force_pid_controller_kd = 0.0;
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_error_sum;
  double kinova_right_bracelet_base_distance;
  double achd_solver_fext_kinova_left_output_external_wrench[6]{};
  int achd_solver_fext_kinova_right_ns = 8;
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_time_step = 1;
  double kinova_right_bracelet_table_contact_force_pid_controller_signal;
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_kd = 0.0;
  double achd_solver_kinova_right_predicted_accelerations[7]{};
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_ki = 0.9;
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_time_step = 1;
  double kinova_left_bracelet_table_contact_force_embed_map_vector[6] = {0.0, 0.0, 1.0,
                                                                         0.0, 0.0, 0.0};
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_error_sum;
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_kp = 20.0;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_kp = 20.0;
  double kinova_left_bracelet_base_distance;
  double kinova_left_bracelet_base_distance_impedance_controller_signal;
  double kinova_left_bracelet_table_contact_force_pid_controller_time_step = 1;
  int achd_solver_fext_kinova_left_nj = 7;
  double achd_solver_fext_kinova_right_root_acceleration[6] = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_signal;
  double kinova_left_bracelet_base_lin_vel_y_embed_map_vector[6] = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_prev_error;
  double achd_solver_kinova_left_predicted_accelerations[7]{};
  double kinova_right_bracelet_base_lin_vel_x_embed_map_vector[6] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_kp = 20.0;
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_ki = 0.9;
  double kinova_right_bracelet_table_contact_force_lin_z;
  double achd_solver_kinova_left_feed_forward_torques[7]{};
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_prev_error;
  double kinova_right_bracelet_table_contact_force_lin_z_vector_z[6] = {0, 0, 1};
  double kinova_left_bracelet_table_contact_force_pid_controller_error_sum;
  std::string table = "table";
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_prev_error;
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_kd = 0.0;
  std::string kinova_right_base_link = "kinova_right_base_link";
  double kinova_left_bracelet_base_distance_embed_map_vector[3] = {1.0, 1.0, 1.0};
  double achd_solver_kinova_right_root_acceleration[6] = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  double kinova_left_bracelet_base_distance_impedance_controller_stiffness_diag_mat[1] = {1.0};
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_prev_error;
  double kinova_right_bracelet_base_distance_embed_map_vector[3] = {1.0, 1.0, 1.0};
  std::string kinova_left_bracelet_link = "kinova_left_bracelet_link";
  double achd_solver_fext_kinova_right_output_torques[7]{};
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_ki = 0.9;
  int achd_solver_fext_kinova_left_ns = 8;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_kd = 0.0;
  double kinova_left_bracelet_base_lin_vel_x_twist_embed_map_vector[6] = {1.0, 0.0, 0.0,
                                                                          0.0, 0.0, 0.0};
  int achd_solver_kinova_right_ns = 8;
  double kinova_right_bracelet_table_contact_force_pid_controller_kp = 20.0;
  int achd_solver_kinova_right_nj = 7;
  double kinova_right_bracelet_table_contact_force_pid_controller_ki = 0.9;
  double kinova_right_bracelet_table_contact_force_pid_controller_kd = 0.0;
  int achd_solver_kinova_right_nc = 6;
  double arms_distance_reference_value = 0.5;
  double kinova_right_bracelet_table_contact_force_embed_map_vector[6] = {0.0, 0.0, 1.0,
                                                                          0.0, 0.0, 0.0};
  double achd_solver_kinova_right_output_torques[7]{};
  int achd_solver_kinova_left_nc = 6;
  std::string kinova_left_base_link = "kinova_left_base_link";
  double achd_solver_kinova_right_output_acceleration_energy[6]{};
  int achd_solver_kinova_left_nj = 7;
  std::string kinova_right_bracelet_link = "kinova_right_bracelet_link";
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_error_sum;
  double kinova_right_bracelet_table_contact_force_pid_controller_prev_error;
  int achd_solver_kinova_left_ns = 8;
  double achd_solver_fext_kinova_right_output_external_wrench[6]{};
  double kinova_left_bracelet_table_contact_force_pid_controller_signal;
  double kinova_left_bracelet_table_contact_force_lin_z;
  double kinova_right_bracelet_base_lin_vel_y_twist_embed_map_vector[6] = {0.0, 1.0, 0.0,
                                                                           0.0, 0.0, 0.0};
  double arm_table_contact_force_reference_value = -10.0;
  double *achd_solver_kinova_right_alpha[2] = {new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                               new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0}};
  double fd_solver_robile_output_external_wrench[6]{};
  double fd_solver_robile_output_external_wrench2[6]{};
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_kp = 20.0;
  double achd_solver_kinova_left_root_acceleration[6] = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_signal;
  double kinova_right_bracelet_base_distance_impedance_controller_stiffness_diag_mat[1] = {1.0};
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_ki = 0.9;
  double kinova_left_bracelet_table_contact_force_pid_controller_prev_error;
  double kinova_right_bracelet_base_vel_vector_lin_y[6] = {0, 1, 0, 0, 0, 0};
  double achd_solver_fext_kinova_left_root_acceleration[6] = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  double achd_solver_kinova_left_output_acceleration_energy[6]{};
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_kd = 0.0;
  double of_vel_qname_lin_x;
  double of_vel_qname_lin_y;
  double achd_solver_fext_kinova_left_output_torques[7]{};
  double achd_solver_kinova_right_feed_forward_torques[7]{};
  double fd_solver_robile_output_torques[8]{};
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_signal;
  double kinova_left_bracelet_base_vel_vector_lin_x[6] = {1, 0, 0, 0, 0, 0};
  double kinova_right_bracelet_table_contact_force_pid_controller_time_step = 1;
  double kinova_left_bracelet_base_vel_vector_lin_y[6] = {0, 1, 0, 0, 0, 0};

  double control_loop_dt = 0.001;

  get_robot_data(&robot, control_loop_dt);

  int count = 0;
  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s
  double angle_about_vec[6] = {0, 0, 0, 0, 0, 1};

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
    // Get the robot structs with the data from robots
    get_robot_data(&robot, control_loop_dt);

    double table_line[3]{};
    getLine(&robot, new std::string[2]{kinova_left_bracelet_link, kinova_right_bracelet_link}, 2,
            table_line);

    double base_line[3]{};
    getLine(&robot, new std::string[2]{kinova_left_base_link, kinova_right_base_link}, 2,
            base_line);


    double angle = 0.0;
    getAngleBetweenLines(&robot, table_line, base_line, angle_about_vec, angle);

    std::cout << "Angle: " << RAD2DEG(angle) << std::endl;

    // ----------- end of loop ------------
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration<double>(end_time - start_time);

    control_loop_dt = elapsed_time.count();

    // if the elapsed time is less than the desired period, busy wait
    while (elapsed_time < desired_period)
    {
      end_time = std::chrono::high_resolution_clock::now();
      elapsed_time = std::chrono::duration<double>(end_time - start_time);

      control_loop_dt = elapsed_time.count();
    }
  }
  -9.595, 0.866, 1.299

  free_robot_data(&robot);

  return 0;
}