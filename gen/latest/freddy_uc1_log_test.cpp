#include "kelo_motion_control/mediator.h"
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
  static int signal_caught = 0;
  if (!signal_caught)
  {
    signal_caught = 1;
    flag = 1;
    printf("Caught signal %d (%s)\n", sig, strsignal(sig));
  }
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

  // read the run description from the command line
  std::string run_description = "";
  if (argc > 1)
  {
    run_description = argv[1];
  }
  else
  {
    printf("Usage: ./freddy_uc1_log <run_description>\n");
    exit(1);
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

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  char ethernet_interface[100] = "eno1";
  initialize_robot(&robot, robot_urdf, ethernet_interface, false);

  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s
  double control_loop_timestep = desired_period.count();                               // s
  double *control_loop_dt = &control_loop_timestep;                                    // s

  // initialize variables
  double kl_elbow_base_base_distance_z_embed_map_vector[3] = {0.0, 0.0, 1.0};
  double kr_bl_orientation_ang_x_pid_controller_signal = 0.0;
  double kinova_left_bracelet_table_contact_force_lin_z_vector_z[6] = {0, 0, 1, 0, 0, 0};
  int kr_achd_solver_nc = 5;
  double kr_achd_solver_alpha[5][6] = {{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                                       {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                                       {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                                       {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                                       {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double kr_bl_position_lin_z_pid_controller_signal = 0.0;
  double kl_bl_position_coord_lin_z_initial = 0.0;
  double kinova_right_bracelet_table_contact_force_pid_controller_error_sum = 0.0;
  double kl_bl_position_coord_lin_z_vector[6] = {0, 0, 1, 0, 0, 0};
  double kinova_left_bracelet_table_contact_force_pid_controller_kp = 1.0;
  double kinova_left_bracelet_table_contact_force_pid_controller_ki = 0.0;
  double kinova_left_bracelet_table_contact_force_pid_controller_kd = 0.0;
  double kl_bl_position_lin_z_twist_embed_map_vector[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  double kl_bl_position_lin_z_pid_controller_prev_error = 0.0;
  double kl_bl_orientation_ang_z_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  double kl_bl_position_lin_y_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
  double kr_bl_orientation_coord_ang_z = 0.0;
  double kl_bl_orientation_ang_z_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
  double kr_bl_orientation_ang_x_pid_controller_error_sum = 0.0;
  double kr_bl_orientation_coord_ang_y = 0.0;
  double kr_bl_orientation_coord_ang_x = 0.0;
  double kinova_right_bracelet_table_contact_force_pid_controller_signal = 0.0;
  double kl_achd_solver_root_acceleration[6] = {-9.6, 0.92, 1.4, 0.0, 0.0, 0.0};
  double kl_bl_position_lin_z_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
  double kr_achd_solver_feed_forward_torques[7]{};
  double kl_bl_orientation_ang_y_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
  double kl_achd_solver_feed_forward_torques[7]{};
  double kr_bl_position_coord_lin_y_initial = 0.0;
  double kl_bl_position_lin_z_pid_controller_signal = 0.0;
  double kl_elbow_base_base_distance_z_impedance_controller_signal = 0.0;
  double kl_bl_orientation_ang_z_pid_controller_error_sum = 0.0;
  double kr_bl_orientation_coord_ang_y_vector[6] = {0, 0, 0, 0, 1, 0};
  std::string kinova_right_base_link = "kinova_right_base_link";
  double kr_bl_position_coord_lin_z_vector[6] = {0, 0, 1, 0, 0, 0};
  double kl_bl_orientation_coord_ang_z_initial = 0.0;
  double kr_bl_orientation_ang_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
  double kr_bl_orientation_coord_ang_y_initial_vector[6] = {0, 0, 0, 0, 1, 0};
  double kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench[6]{};
  std::string kinova_left_bracelet_link_origin_point = "kinova_left_bracelet_link";
  double kr_bl_position_lin_y_pid_controller_prev_error = 0.0;
  double kr_bl_orientation_ang_y_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  double
      kinova_left_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench
          [6]{};
  double kl_achd_solver_predicted_accelerations[7]{};
  double kl_achd_solver_fext_output_torques[7]{};
  int kl_achd_solver_nc = 5;
  double kl_bl_orientation_ang_z_pid_controller_signal = 0.0;
  double
      kinova_right_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench
          [6]{};
  double kr_bl_position_lin_y_twist_embed_map_vector[6] = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  double kl_bl_orientation_coord_ang_y_initial = 0.0;
  double kr_bl_position_coord_lin_z_initial_vector[6] = {0, 0, 1, 0, 0, 0};
  double kr_bl_position_lin_z_pid_controller_error_sum = 0.0;
  int kl_achd_solver_fext_nj = 7;
  int kr_achd_solver_nj = 7;
  double kr_elbow_base_base_distance_z_impedance_controller_signal = 0.0;
  int kl_achd_solver_nj = 7;
  double kl_bl_orientation_coord_ang_y_vector[6] = {0, 0, 0, 0, 1, 0};
  double kl_elbow_base_distance_coord_lin_z_axis[6] = {0, 0, 1, 0, 0, 0};
  double kinova_right_bracelet_table_contact_force_pid_controller_prev_error = 0.0;
  double kr_bl_position_coord_lin_y = 0.0;
  double kr_bl_position_coord_lin_z = 0.0;
  double kinova_left_bracelet_table_contact_force_lin_z = 0.0;
  double kl_bl_orientation_ang_x_twist_embed_map_kl_achd_solver_output_acceleration_energy[6]{};
  double kr_bl_orientation_ang_y_pid_controller_prev_error = 0.0;
  double kl_elbow_base_distance_coord_lin_z = 0.0;
  double kl_bl_position_lin_y_pid_controller_signal = 0.0;
  double kl_bl_orientation_ang_y_pid_controller_prev_error = 0.0;
  double kr_bl_orientation_ang_x_pid_controller_prev_error = 0.0;
  double kr_bl_orientation_coord_ang_y_initial = 0.0;
  double kl_bl_orientation_ang_x_pid_controller_prev_error = 0.0;
  double kr_bl_orientation_ang_z_pid_controller_signal = 0.0;
  double kr_bl_orientation_coord_ang_x_initial_vector[6] = {0, 0, 0, 1, 0, 0};
  double kr_bl_orientation_ang_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
  double kr_elbow_base_distance_coord_lin_z_axis[6] = {0, 0, 1, 0, 0, 0};
  double kl_bl_orientation_ang_x_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  double kl_bl_orientation_coord_ang_x_initial_vector[6] = {0, 0, 0, 1, 0, 0};
  double kl_bl_position_lin_z_pid_controller_error_sum = 0.0;
  double kr_bl_orientation_ang_z_pid_controller_prev_error = 0.0;
  double kl_bl_orientation_ang_z_pid_controller_prev_error = 0.0;

  std::string kinova_right_half_arm_2_link = "kinova_right_forearm_link";
  std::string kinova_left_half_arm_2_link = "kinova_left_forearm_link";

  double kr_bl_position_lin_y_pid_controller_kp = 60.0;
  double kr_bl_position_lin_y_pid_controller_ki = 2.9;
  double kr_bl_position_lin_y_pid_controller_kd = 70.5;

  double kl_bl_position_lin_y_pid_controller_kp = 60.0;
  double kl_bl_position_lin_y_pid_controller_ki = 2.9;
  double kl_bl_position_lin_y_pid_controller_kd = 70.5;

  double kr_bl_position_lin_z_pid_controller_kp = 100.0;
  double kr_bl_position_lin_z_pid_controller_ki = 2.9;
  double kr_bl_position_lin_z_pid_controller_kd = 75.5;

  double kl_bl_position_lin_z_pid_controller_kp = 60.0;
  double kl_bl_position_lin_z_pid_controller_ki = 2.9;
  double kl_bl_position_lin_z_pid_controller_kd = 75.5;

  double kr_elbow_base_z_distance_reference_value = 0.7;
  double kl_elbow_base_z_distance_reference_value = 0.7;
  double kr_elbow_base_base_distance_z_impedance_controller_stiffness_diag_mat[1] = {300.0};
  double kl_elbow_base_base_distance_z_impedance_controller_stiffness_diag_mat[1] = {300.0};

  double kr_arm_table_contact_force_reference_value = -15.0;
  double kl_arm_table_contact_force_reference_value = -15.0;

  double kl_bl_orientation_ang_x_pid_controller_kp = 80.0;
  double kl_bl_orientation_ang_x_pid_controller_ki = 2.9;
  double kl_bl_orientation_ang_x_pid_controller_kd = 50.5;
  double kl_bl_orientation_ang_y_pid_controller_kp = 80.0;
  double kl_bl_orientation_ang_y_pid_controller_ki = 2.9;
  double kl_bl_orientation_ang_y_pid_controller_kd = 50.5;
  double kl_bl_orientation_ang_z_pid_controller_kp = 200.0;
  double kl_bl_orientation_ang_z_pid_controller_ki = 2.9;
  double kl_bl_orientation_ang_z_pid_controller_kd = 80.5;

  double kr_bl_orientation_ang_x_pid_controller_kp = 90.0;
  double kr_bl_orientation_ang_x_pid_controller_ki = 2.9;
  double kr_bl_orientation_ang_x_pid_controller_kd = 70.5;
  double kr_bl_orientation_ang_y_pid_controller_kp = 90.0;
  double kr_bl_orientation_ang_y_pid_controller_ki = 2.9;
  double kr_bl_orientation_ang_y_pid_controller_kd = 70.5;
  double kr_bl_orientation_ang_z_pid_controller_kp = 200.0;
  double kr_bl_orientation_ang_z_pid_controller_ki = 2.9;
  double kr_bl_orientation_ang_z_pid_controller_kd = 80.5;

  double kl_bl_orientation_ang_y_pid_controller_error_sum = 0.0;
  double kl_bl_orientation_ang_y_pid_controller_signal = 0.0;
  int kr_achd_solver_fext_nj = 7;
  double kr_bl_orientation_coord_ang_z_initial = 0.0;
  double kr_bl_position_lin_y_pid_controller_signal = 0.0;
  double kl_bl_position_coord_lin_z_initial_vector[6] = {0, 0, 1, 0, 0, 0};
  double kr_bl_position_coord_lin_z_initial = 0.0;
  std::string base_link = "base_link";
  double kr_bl_orientation_coord_ang_z_vector[6] = {0, 0, 0, 0, 0, 1};
  double kr_bl_orientation_coord_ang_x_vector[6] = {0, 0, 0, 1, 0, 0};
  double kl_bl_orientation_coord_ang_z = 0.0;
  double kl_bl_orientation_coord_ang_y = 0.0;
  std::string kinova_right_bracelet_link_origin_point = "kinova_right_bracelet_link";
  double kl_bl_orientation_coord_ang_x = 0.0;
  double kl_bl_position_lin_y_twist_embed_map_vector[6] = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  double kr_bl_position_coord_lin_y_vector[6] = {0, 1, 0, 0, 0, 0};
  double kr_bl_orientation_ang_y_pid_controller_error_sum = 0.0;
  double kinova_left_bracelet_table_contact_force_embed_map_vector[6] = {0.0, 0.0, 1.0,
                                                                         0.0, 0.0, 0.0};
  double kr_elbow_base_base_distance_z_embed_map_vector[3] = {0.0, 0.0, 1.0};
  double kl_bl_orientation_ang_x_pid_controller_error_sum = 0.0;
  double kl_bl_position_coord_lin_y_initial_vector[6] = {0, 1, 0, 0, 0, 0};
  double kinova_right_bracelet_table_contact_force_lin_z = 0.0;
  double kr_bl_position_coord_lin_y_initial_vector[6] = {0, 1, 0, 0, 0, 0};
  double kinova_right_bracelet_table_contact_force_lin_z_vector_z[6] = {0, 0, 1, 0, 0, 0};
  double kinova_left_bracelet_table_contact_force_pid_controller_error_sum = 0.0;
  std::string table = "table";
  double kr_bl_position_lin_z_pid_controller_prev_error = 0.0;
  double kr_bl_orientation_ang_z_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  double kr_bl_position_lin_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
  double kl_bl_orientation_coord_ang_z_vector[6] = {0, 0, 0, 0, 0, 1};
  double kr_bl_orientation_ang_y_pid_controller_signal = 0.0;
  double kl_bl_position_coord_lin_y_vector[6] = {0, 1, 0, 0, 0, 0};
  double kl_bl_orientation_coord_ang_y_initial_vector[6] = {0, 0, 0, 0, 1, 0};
  std::string kinova_left_bracelet_link = "kinova_left_bracelet_link";
  double kl_achd_solver_output_torques[7]{};
  double kl_bl_position_coord_lin_y_initial = 0.0;
  double kl_achd_solver_alpha[5][6] = {{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                                       {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                                       {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                                       {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                                       {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double kinova_right_bracelet_table_contact_force_pid_controller_kp = 1.0;
  double kr_bl_orientation_coord_ang_x_initial = 0.0;
  double kinova_right_bracelet_table_contact_force_pid_controller_ki = 0.0;
  double kr_achd_solver_fext_output_torques[7]{};
  double kinova_right_bracelet_table_contact_force_pid_controller_kd = 0.0;
  double kr_achd_solver_output_torques[7]{};
  double kinova_right_bracelet_table_contact_force_embed_map_vector[6] = {0.0, 0.0, 1.0,
                                                                          0.0, 0.0, 0.0};
  double kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench[6]{};
  double kr_bl_position_lin_y_pid_controller_error_sum = 0.0;
  double kr_bl_orientation_ang_x_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  std::string kinova_left_base_link = "kinova_left_base_link";
  std::string kinova_right_bracelet_link = "kinova_right_bracelet_link";
  double kl_bl_orientation_ang_x_pid_controller_signal = 0.0;
  double kr_bl_position_lin_z_twist_embed_map_vector[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  double kr_bl_orientation_ang_z_pid_controller_error_sum = 0.0;
  double kinova_left_bracelet_table_contact_force_pid_controller_signal = 0.0;
  double kr_achd_solver_predicted_accelerations[7]{};
  double kl_bl_position_lin_y_pid_controller_prev_error = 0.0;
  double kr_elbow_base_distance_coord_lin_z = 0.0;
  double kl_bl_orientation_coord_ang_z_initial_vector[6] = {0, 0, 0, 0, 0, 1};
  double kl_bl_orientation_ang_y_twist_embed_map_vector[6] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  double kr_achd_solver_root_acceleration[6] = {-9.685, -1.033, 1.34, 0.0, 0.0, 0.0};
  double kl_bl_orientation_coord_ang_x_initial = 0.0;
  double kr_bl_orientation_ang_x_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
  double kl_bl_position_coord_lin_y = 0.0;
  double kl_bl_position_coord_lin_z = 0.0;
  double kr_bl_position_lin_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{};
  double kinova_left_bracelet_table_contact_force_pid_controller_prev_error = 0.0;
  double kl_bl_orientation_coord_ang_x_vector[6] = {0, 0, 0, 1, 0, 0};
  double kl_bl_position_lin_y_pid_controller_error_sum = 0.0;
  std::string base_link_origin_point = "base_link";
  double kr_bl_orientation_coord_ang_z_initial_vector[6] = {0, 0, 0, 0, 0, 1};

  // uc1 vars
  double arms_bl_base_distance_reference_value = 0.75;

  double kr_bl_base_distance = 0.0;
  double kr_bl_base_distance_pid_controller_kp = 300.0;
  double kr_bl_base_distance_pid_controller_ki = 50.5;
  double kr_bl_base_distance_pid_controller_kd = 100.0;
  double kr_bl_base_distance_pid_error_sum = 0.0;
  double kr_bl_base_distance_pid_controller_signal = 0.0;

  double kl_bl_base_distance = 0.0;
  double kl_bl_base_distance_pid_controller_kp = 300.0;
  double kl_bl_base_distance_pid_controller_ki = 50.5;
  double kl_bl_base_distance_pid_controller_kd = 100.0;
  double kl_bl_base_distance_pid_error_sum = 0.0;
  double kl_bl_base_distance_pid_controller_signal = 0.0;

  double fd_solver_robile_output_torques[8]{};

  double kr_bl_base_distance_pid_prev_error = 0.0;
  double kl_bl_base_distance_pid_prev_error = 0.0;

  // log structs
  std::string log_dir = "../../logs/data/freddy_uc1_log_test";
  char log_dir_name[100];
  get_new_folder_name(log_dir.c_str(), log_dir_name);
  std::filesystem::create_directories(log_dir_name);
  std::filesystem::permissions(log_dir_name, std::filesystem::perms::others_all |
                                                 std::filesystem::perms::group_all |
                                                 std::filesystem::perms::owner_all);

  // write the run description to a .md file
  std::string run_description_file = log_dir_name;
  run_description_file += "/run_description.md";
  std::ofstream run_description_file_stream(run_description_file);
  run_description_file_stream << run_description;
  run_description_file_stream.close();

  LogManipulatorDataVector kr_log_data_vec("kinova_right", log_dir_name, 1);
  LogManipulatorDataVector kl_log_data_vec("kinova_left", log_dir_name, 2);
  LogMobileBaseDataVector base_log_data_vec(log_dir_name, 3);

  // explicitly referesh the robot data
  robot.kinova_left->mediator->refresh_feedback();
  robot.kinova_right->mediator->refresh_feedback();
  update_base_state(robot.mobile_base->mediator->kelo_base_config,
                    robot.mobile_base->mediator->ethercat_config);
  get_robot_data(&robot, *control_loop_dt);

  double table_line[3]{};
  double base_line[3]{};

  std::string table_line_entities[2] = {kinova_left_bracelet_link, kinova_right_bracelet_link};
  std::string base_line_entities[2] = {kinova_left_base_link, kinova_right_base_link};

  getLine(&robot, table_line_entities, 2, table_line);

  getLine(&robot, base_line_entities, 2, base_line);

  double angle_about_vec[6] = {0, 0, 0, 0, 0, 1};

  double base_init_angle = 0.0;
  getAngleBetweenLines(&robot, table_line, base_line, angle_about_vec, base_init_angle);

  // update compute variables
  getLinkPosition(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                  kr_bl_position_coord_lin_y_initial_vector, &robot,
                  kr_bl_position_coord_lin_y_initial);
  getLinkPosition(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                  kr_bl_position_coord_lin_z_initial_vector, &robot,
                  kr_bl_position_coord_lin_z_initial);

  getLinkPosition(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                  kl_bl_position_coord_lin_y_initial_vector, &robot,
                  kl_bl_position_coord_lin_y_initial);
  getLinkPosition(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                  kl_bl_position_coord_lin_z_initial_vector, &robot,
                  kl_bl_position_coord_lin_z_initial);

  double base_world_orientation_coord_ang_quat_initial[4] = {0, 0, 0, 1};
  double base_world_orientation_coord_ang_quat[4] = {0, 0, 0, 1};

  KDL::Rotation base_theta = KDL::Rotation::RotZ(base_init_angle);
  base_theta.GetQuaternion(base_world_orientation_coord_ang_quat_initial[0],
                           base_world_orientation_coord_ang_quat_initial[1],
                           base_world_orientation_coord_ang_quat_initial[2],
                           base_world_orientation_coord_ang_quat_initial[3]);

  double kr_world_orientation_coord_ang_quat_initial[4] = {0, 0, 0, 1};
  double kr_world_orientation_coord_ang_quat[4] = {0, 0, 0, 1};

  getLinkQuaternion(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                    &robot, kr_world_orientation_coord_ang_quat_initial);

  double kl_world_orientation_coord_ang_quat_initial[4] = {0, 0, 0, 1};
  double kl_world_orientation_coord_ang_quat[4] = {0, 0, 0, 1};

  getLinkQuaternion(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                    &robot, kl_world_orientation_coord_ang_quat_initial);

  KDL::Rotation kr_theta_init =
      KDL::Rotation::Quaternion(kr_world_orientation_coord_ang_quat_initial[0],
                                kr_world_orientation_coord_ang_quat_initial[1],
                                kr_world_orientation_coord_ang_quat_initial[2],
                                kr_world_orientation_coord_ang_quat_initial[3]);

  KDL::Rotation kl_theta_init =
      KDL::Rotation::Quaternion(kl_world_orientation_coord_ang_quat_initial[0],
                                kl_world_orientation_coord_ang_quat_initial[1],
                                kl_world_orientation_coord_ang_quat_initial[2],
                                kl_world_orientation_coord_ang_quat_initial[3]);

  KDL::Rotation base_theta_init =
      KDL::Rotation::Quaternion(base_world_orientation_coord_ang_quat_initial[0],
                                base_world_orientation_coord_ang_quat_initial[1],
                                base_world_orientation_coord_ang_quat_initial[2],
                                base_world_orientation_coord_ang_quat_initial[3]);

  double kr_bl_base_distance_controller_error = 0.0;
  std::string kr_bl_base_distance_entities[2] = {kinova_right_bracelet_link,
                                                 kinova_right_base_link};
  computeDistance(kr_bl_base_distance_entities, kinova_right_bracelet_link, &robot,
                  kr_bl_base_distance);
  computeEqualityError(arms_bl_base_distance_reference_value, kr_bl_base_distance,
                       kr_bl_base_distance_controller_error);
  kr_bl_base_distance_pid_prev_error = kr_bl_base_distance_controller_error;

  double kl_bl_base_distance_controller_error = 0.0;
  std::string kl_bl_base_distance_entities[2] = {kinova_left_bracelet_link, kinova_left_base_link};
  computeDistance(kl_bl_base_distance_entities, kinova_left_bracelet_link, &robot,
                  kl_bl_base_distance);
  computeEqualityError(arms_bl_base_distance_reference_value, kl_bl_base_distance,
                       kl_bl_base_distance_controller_error);
  kl_bl_base_distance_pid_prev_error = kl_bl_base_distance_controller_error;

  // base wheels alignment controllers
  double base_wheel_alignment_controller_Kp = 0.75;
  double base_wheel_alignment_controller_Ki = 0.75;
  double base_wheel_alignment_controller_Kd = 0.1;

  double base_w1_lin_prev_error = 0.0;
  double base_w2_lin_prev_error = 0.0;
  double base_w3_lin_prev_error = 0.0;
  double base_w4_lin_prev_error = 0.0;

  double base_w1_lin_error_sum = 0.0;
  double base_w2_lin_error_sum = 0.0;
  double base_w3_lin_error_sum = 0.0;
  double base_w4_lin_error_sum = 0.0;

  double base_w1_ang_prev_error = 0.0;
  double base_w2_ang_prev_error = 0.0;
  double base_w3_ang_prev_error = 0.0;
  double base_w4_ang_prev_error = 0.0;

  double base_w1_ang_error_sum = 0.0;
  double base_w2_ang_error_sum = 0.0;
  double base_w3_ang_error_sum = 0.0;
  double base_w4_ang_error_sum = 0.0;

  double tau_wheel_ref_limit = 1.5;

  double kr_achd_solver_beta[6]{};
  double kl_achd_solver_beta[6]{};
  double kinova_right_cmd_tau[7]{};
  double kinova_left_cmd_tau[7]{};

  int count = 0;

  while (true)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (flag)
    {
      kr_log_data_vec.writeToOpenFile();
      kl_log_data_vec.writeToOpenFile();
      base_log_data_vec.writeToOpenFile();

      free_robot_data(&robot);
      printf("Exiting somewhat cleanly...\n");
      exit(0);
    }

    count++;
    printf("count: %d\n", count);

    robot.kinova_left->mediator->refresh_feedback();
    robot.kinova_right->mediator->refresh_feedback();
    get_robot_data(&robot, *control_loop_dt);

    KDL::Frame base_to_world;
    base_to_world.p = KDL::Vector(robot.mobile_base->state->x_platform[0],
                                  robot.mobile_base->state->x_platform[1], 0.0);
    base_to_world.M = KDL::Rotation::RotZ(robot.mobile_base->state->x_platform[2]);

    // get base theta
    getLine(&robot, table_line_entities, 2, table_line);
    getLine(&robot, base_line_entities, 2, base_line);

    double base_current_angle = 0.0;
    getAngleBetweenLines(&robot, table_line, base_line, angle_about_vec, base_current_angle);

    KDL::Rotation base_theta_current = KDL::Rotation::RotZ(base_current_angle);
    base_theta_current.GetQuaternion(
        base_world_orientation_coord_ang_quat[0], base_world_orientation_coord_ang_quat[1],
        base_world_orientation_coord_ang_quat[2], base_world_orientation_coord_ang_quat[3]);

    // get ee theta's
    double kr_world_orientation_coord_ang_quat[4] = {0, 0, 0, 1};
    getLinkQuaternion(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                      &robot, kr_world_orientation_coord_ang_quat);

    double kl_world_orientation_coord_ang_quat[4] = {0, 0, 0, 1};
    getLinkQuaternion(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                      &robot, kl_world_orientation_coord_ang_quat);

    // theta diffs
    double base_theta_diff = 0.0;
    KDL::Vector base_angle_diff = KDL::diff(base_theta_init, base_theta_current);
    base_theta_diff = base_angle_diff.z();

    KDL::Rotation base_theta_comp = KDL::Rotation::RotZ(base_theta_diff);

    KDL::Rotation kl_theta_current = KDL::Rotation::Quaternion(
        kl_world_orientation_coord_ang_quat[0], kl_world_orientation_coord_ang_quat[1],
        kl_world_orientation_coord_ang_quat[2], kl_world_orientation_coord_ang_quat[3]);

    KDL::Rotation kr_theta_current = KDL::Rotation::Quaternion(
        kr_world_orientation_coord_ang_quat[0], kr_world_orientation_coord_ang_quat[1],
        kr_world_orientation_coord_ang_quat[2], kr_world_orientation_coord_ang_quat[3]);

    // transform the current kr theta into world frame based on base_theta_comp
    KDL::Rotation kr_theta_world = base_theta_comp * kr_theta_current;
    KDL::Vector kr_theta_diff = KDL::diff(kr_theta_world, kr_theta_init);

    // transform the current kl theta into world frame based on base_theta_comp
    KDL::Rotation kl_theta_world = base_theta_comp * kl_theta_current;
    KDL::Vector kl_theta_diff = KDL::diff(kl_theta_world, kl_theta_init);

    // controllers
    // pid controller
    double kl_bl_orientation_ang_x_pid_controller_error = 0;
    kl_bl_orientation_ang_x_pid_controller_error = kl_theta_diff.x();
    pidController(
        kl_bl_orientation_ang_x_pid_controller_error, kl_bl_orientation_ang_x_pid_controller_kp,
        kl_bl_orientation_ang_x_pid_controller_ki, kl_bl_orientation_ang_x_pid_controller_kd,
        *control_loop_dt, kl_bl_orientation_ang_x_pid_controller_error_sum, 1.0,
        kl_bl_orientation_ang_x_pid_controller_prev_error,
        kl_bl_orientation_ang_x_pid_controller_signal);

    // pid controller
    double kl_bl_orientation_ang_y_pid_controller_error = 0;
    kl_bl_orientation_ang_y_pid_controller_error = kl_theta_diff.y();
    pidController(
        kl_bl_orientation_ang_y_pid_controller_error, kl_bl_orientation_ang_y_pid_controller_kp,
        kl_bl_orientation_ang_y_pid_controller_ki, kl_bl_orientation_ang_y_pid_controller_kd,
        *control_loop_dt, kl_bl_orientation_ang_y_pid_controller_error_sum, 1.0,
        kl_bl_orientation_ang_y_pid_controller_prev_error,
        kl_bl_orientation_ang_y_pid_controller_signal);

    // pid controller
    double kl_bl_orientation_ang_z_pid_controller_error = 0;
    kl_bl_orientation_ang_z_pid_controller_error = kl_theta_diff.z();
    pidController(
        kl_bl_orientation_ang_z_pid_controller_error, kl_bl_orientation_ang_z_pid_controller_kp,
        kl_bl_orientation_ang_z_pid_controller_ki, kl_bl_orientation_ang_z_pid_controller_kd,
        *control_loop_dt, kl_bl_orientation_ang_z_pid_controller_error_sum, 1.0,
        kl_bl_orientation_ang_z_pid_controller_prev_error,
        kl_bl_orientation_ang_z_pid_controller_signal);

    // pid controller
    double kr_bl_orientation_ang_x_pid_controller_error = 0;
    kr_bl_orientation_ang_x_pid_controller_error = kr_theta_diff.x();
    pidController(
        kr_bl_orientation_ang_x_pid_controller_error, kr_bl_orientation_ang_x_pid_controller_kp,
        kr_bl_orientation_ang_x_pid_controller_ki, kr_bl_orientation_ang_x_pid_controller_kd,
        *control_loop_dt, kr_bl_orientation_ang_x_pid_controller_error_sum, 1.0,
        kr_bl_orientation_ang_x_pid_controller_prev_error,
        kr_bl_orientation_ang_x_pid_controller_signal);

    // pid controller
    double kr_bl_orientation_ang_y_pid_controller_error = 0;
    kr_bl_orientation_ang_y_pid_controller_error = kr_theta_diff.y();
    pidController(
        kr_bl_orientation_ang_y_pid_controller_error, kr_bl_orientation_ang_y_pid_controller_kp,
        kr_bl_orientation_ang_y_pid_controller_ki, kr_bl_orientation_ang_y_pid_controller_kd,
        *control_loop_dt, kr_bl_orientation_ang_y_pid_controller_error_sum, 1.0,
        kr_bl_orientation_ang_y_pid_controller_prev_error,
        kr_bl_orientation_ang_y_pid_controller_signal);

    // pid controller
    double kr_bl_orientation_ang_z_pid_controller_error = 0;
    kr_bl_orientation_ang_z_pid_controller_error = kr_theta_diff.z();
    pidController(
        kr_bl_orientation_ang_z_pid_controller_error, kr_bl_orientation_ang_z_pid_controller_kp,
        kr_bl_orientation_ang_z_pid_controller_ki, kr_bl_orientation_ang_z_pid_controller_kd,
        *control_loop_dt, kr_bl_orientation_ang_z_pid_controller_error_sum, 1.0,
        kr_bl_orientation_ang_z_pid_controller_prev_error,
        kr_bl_orientation_ang_z_pid_controller_signal);

    // pid controller
    getLinkForce(kinova_left_bracelet_link, table, kinova_left_bracelet_link,
                 kinova_left_bracelet_table_contact_force_lin_z_vector_z, &robot,
                 kinova_left_bracelet_table_contact_force_lin_z);
    double kinova_left_bracelet_table_contact_force_pid_controller_error = 0;
    computeEqualityError(kinova_left_bracelet_table_contact_force_lin_z,
                         kl_arm_table_contact_force_reference_value,
                         kinova_left_bracelet_table_contact_force_pid_controller_error);
    pidController(kinova_left_bracelet_table_contact_force_pid_controller_error,
                  kinova_left_bracelet_table_contact_force_pid_controller_kp,
                  kinova_left_bracelet_table_contact_force_pid_controller_ki,
                  kinova_left_bracelet_table_contact_force_pid_controller_kd, *control_loop_dt,
                  kinova_left_bracelet_table_contact_force_pid_controller_error_sum, 1.0,
                  kinova_left_bracelet_table_contact_force_pid_controller_prev_error,
                  kinova_left_bracelet_table_contact_force_pid_controller_signal);

    // impedance controller
    double kl_elbow_base_base_distance_z_impedance_controller_stiffness_error = 0;
    std::string kl_elbow_base_distance_entities[2] = {kinova_left_half_arm_2_link, base_link};
    computeDistance1D(kl_elbow_base_distance_entities, kl_elbow_base_distance_coord_lin_z_axis,
                      base_link, &robot, kl_elbow_base_distance_coord_lin_z);
    computeEqualityError(kl_elbow_base_distance_coord_lin_z,
                         kl_elbow_base_z_distance_reference_value,
                         kl_elbow_base_base_distance_z_impedance_controller_stiffness_error);
    double kl_elbow_base_base_distance_z_impedance_controller_damping_diag_mat[1] = {0.0};
    impedanceController(kl_elbow_base_base_distance_z_impedance_controller_stiffness_error, 0.0,
                        kl_elbow_base_base_distance_z_impedance_controller_stiffness_diag_mat,
                        kl_elbow_base_base_distance_z_impedance_controller_damping_diag_mat,
                        kl_elbow_base_base_distance_z_impedance_controller_signal);

    // pid controller
    getLinkPosition(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kr_bl_position_coord_lin_y_vector, &robot, kr_bl_position_coord_lin_y);

    // transform from base to world
    KDL::Vector kr_bl_position_coord_lin_y_world =
        base_to_world * KDL::Vector(0.0, kr_bl_position_coord_lin_y, 0.0);
    kr_bl_position_coord_lin_y = kr_bl_position_coord_lin_y_world.y();

    double kr_bl_position_lin_y_pid_controller_error = 0;
    computeEqualityError(kr_bl_position_coord_lin_y, kr_bl_position_coord_lin_y_initial,
                         kr_bl_position_lin_y_pid_controller_error);
    pidController(kr_bl_position_lin_y_pid_controller_error,
                  kr_bl_position_lin_y_pid_controller_kp, kr_bl_position_lin_y_pid_controller_ki,
                  kr_bl_position_lin_y_pid_controller_kd, *control_loop_dt,
                  kr_bl_position_lin_y_pid_controller_error_sum, 1.0,
                  kr_bl_position_lin_y_pid_controller_prev_error,
                  kr_bl_position_lin_y_pid_controller_signal);

    // pid controller
    getLinkPosition(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kl_bl_position_coord_lin_z_vector, &robot, kl_bl_position_coord_lin_z);
    double kl_bl_position_lin_z_pid_controller_error = 0;
    computeEqualityError(kl_bl_position_coord_lin_z, kl_bl_position_coord_lin_z_initial,
                         kl_bl_position_lin_z_pid_controller_error);
    pidController(kl_bl_position_lin_z_pid_controller_error,
                  kl_bl_position_lin_z_pid_controller_kp, kl_bl_position_lin_z_pid_controller_ki,
                  kl_bl_position_lin_z_pid_controller_kd, *control_loop_dt,
                  kl_bl_position_lin_z_pid_controller_error_sum, 1.0,
                  kl_bl_position_lin_z_pid_controller_prev_error,
                  kl_bl_position_lin_z_pid_controller_signal);

    // pid controller
    getLinkForce(kinova_right_bracelet_link, table, kinova_right_bracelet_link,
                 kinova_right_bracelet_table_contact_force_lin_z_vector_z, &robot,
                 kinova_right_bracelet_table_contact_force_lin_z);
    double kinova_right_bracelet_table_contact_force_pid_controller_error = 0;
    computeEqualityError(kinova_right_bracelet_table_contact_force_lin_z,
                         kr_arm_table_contact_force_reference_value,
                         kinova_right_bracelet_table_contact_force_pid_controller_error);
    pidController(kinova_right_bracelet_table_contact_force_pid_controller_error,
                  kinova_right_bracelet_table_contact_force_pid_controller_kp,
                  kinova_right_bracelet_table_contact_force_pid_controller_ki,
                  kinova_right_bracelet_table_contact_force_pid_controller_kd, *control_loop_dt,
                  kinova_right_bracelet_table_contact_force_pid_controller_error_sum, 1.0,
                  kinova_right_bracelet_table_contact_force_pid_controller_prev_error,
                  kinova_right_bracelet_table_contact_force_pid_controller_signal);

    // impedance controller
    double kr_elbow_base_base_distance_z_impedance_controller_stiffness_error = 0;
    std::string kr_elbow_base_distance_entities[2] = {kinova_right_half_arm_2_link, base_link};
    computeDistance1D(kr_elbow_base_distance_entities, kr_elbow_base_distance_coord_lin_z_axis,
                      base_link, &robot, kr_elbow_base_distance_coord_lin_z);
    computeEqualityError(kr_elbow_base_distance_coord_lin_z,
                         kr_elbow_base_z_distance_reference_value,
                         kr_elbow_base_base_distance_z_impedance_controller_stiffness_error);
    double kr_elbow_base_base_distance_z_impedance_controller_damping_diag_mat[1] = {0.0};
    impedanceController(kr_elbow_base_base_distance_z_impedance_controller_stiffness_error, 0.0,
                        kr_elbow_base_base_distance_z_impedance_controller_stiffness_diag_mat,
                        kr_elbow_base_base_distance_z_impedance_controller_damping_diag_mat,
                        kr_elbow_base_base_distance_z_impedance_controller_signal);

    // pid controller
    getLinkPosition(kinova_right_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kr_bl_position_coord_lin_z_vector, &robot, kr_bl_position_coord_lin_z);
    double kr_bl_position_lin_z_pid_controller_error = 0;
    computeEqualityError(kr_bl_position_coord_lin_z, kr_bl_position_coord_lin_z_initial,
                         kr_bl_position_lin_z_pid_controller_error);
    pidController(kr_bl_position_lin_z_pid_controller_error,
                  kr_bl_position_lin_z_pid_controller_kp, kr_bl_position_lin_z_pid_controller_ki,
                  kr_bl_position_lin_z_pid_controller_kd, *control_loop_dt,
                  kr_bl_position_lin_z_pid_controller_error_sum, 1.0,
                  kr_bl_position_lin_z_pid_controller_prev_error,
                  kr_bl_position_lin_z_pid_controller_signal);

    // pid controller
    getLinkPosition(kinova_left_bracelet_link_origin_point, base_link, base_link_origin_point,
                    kl_bl_position_coord_lin_y_vector, &robot, kl_bl_position_coord_lin_y);

    // transform from base to world
    KDL::Vector kl_bl_position_coord_lin_y_world =
        base_to_world * KDL::Vector(0.0, kl_bl_position_coord_lin_y, 0.0);
    kl_bl_position_coord_lin_y = kl_bl_position_coord_lin_y_world.y();

    double kl_bl_position_lin_y_pid_controller_error = 0;
    computeEqualityError(kl_bl_position_coord_lin_y, kl_bl_position_coord_lin_y_initial,
                         kl_bl_position_lin_y_pid_controller_error);
    pidController(kl_bl_position_lin_y_pid_controller_error,
                  kl_bl_position_lin_y_pid_controller_kp, kl_bl_position_lin_y_pid_controller_ki,
                  kl_bl_position_lin_y_pid_controller_kd, *control_loop_dt,
                  kl_bl_position_lin_y_pid_controller_error_sum, 1.0,
                  kl_bl_position_lin_y_pid_controller_prev_error,
                  kl_bl_position_lin_y_pid_controller_signal);

    // uc1 controllers
    // impedance controller
    double kr_bl_base_distance_pid_controller_signal = 0.0;
    double kr_bl_base_distance_controller_error = 0.0;
    std::string kr_bl_base_distance_entities[2] = {kinova_right_bracelet_link,
                                                   kinova_right_base_link};
    computeDistance(kr_bl_base_distance_entities, kinova_right_bracelet_link, &robot,
                    kr_bl_base_distance);
    computeEqualityError(arms_bl_base_distance_reference_value, kr_bl_base_distance,
                         kr_bl_base_distance_controller_error);
    pidController(kr_bl_base_distance_controller_error, kr_bl_base_distance_pid_controller_kp,
                  kr_bl_base_distance_pid_controller_ki, kr_bl_base_distance_pid_controller_kd,
                  *control_loop_dt, kr_bl_base_distance_pid_error_sum, 50.0,
                  kr_bl_base_distance_pid_prev_error, kr_bl_base_distance_pid_controller_signal);

    // impedance controller
    double kl_bl_base_distance_pid_controller_signal = 0.0;
    double kl_bl_base_distance_controller_error = 0.0;
    std::string kl_bl_base_distance_entities[2] = {kinova_left_bracelet_link,
                                                   kinova_left_base_link};
    computeDistance(kl_bl_base_distance_entities, kinova_left_bracelet_link, &robot,
                    kl_bl_base_distance);
    computeEqualityError(arms_bl_base_distance_reference_value, kl_bl_base_distance,
                         kl_bl_base_distance_controller_error);
    pidController(kl_bl_base_distance_controller_error, kl_bl_base_distance_pid_controller_kp,
                  kl_bl_base_distance_pid_controller_ki, kl_bl_base_distance_pid_controller_kd,
                  *control_loop_dt, kl_bl_base_distance_pid_error_sum, 50.0,
                  kl_bl_base_distance_pid_prev_error, kl_bl_base_distance_pid_controller_signal);

    // embed maps
    double
        kinova_left_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench
            [6]{};
    double kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench[6]{};

    for (size_t i = 0;
         i < sizeof(kinova_left_bracelet_table_contact_force_embed_map_vector) /
                 sizeof(kinova_left_bracelet_table_contact_force_embed_map_vector[0]);
         i++)
    {
      if (kinova_left_bracelet_table_contact_force_embed_map_vector[i] != 0.0)
      {
        kinova_left_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench
            [i] += kinova_left_bracelet_table_contact_force_pid_controller_signal;
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
        kinova_right_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench
            [6]{};
    double kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench[6]{};

    for (size_t i = 0;
         i < sizeof(kinova_right_bracelet_table_contact_force_embed_map_vector) /
                 sizeof(kinova_right_bracelet_table_contact_force_embed_map_vector[0]);
         i++)
    {
      if (kinova_right_bracelet_table_contact_force_embed_map_vector[i] != 0.0)
      {
        kinova_right_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench
            [i] += kinova_right_bracelet_table_contact_force_pid_controller_signal;
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

    // uc1 embed maps
    double fd_solver_robile_output_external_wrench_kl[6]{};
    decomposeSignal(&robot, kinova_left_base_link, kinova_left_bracelet_link,
                    kinova_left_bracelet_link, kl_bl_base_distance_pid_controller_signal,
                    fd_solver_robile_output_external_wrench_kl);

    transform_wrench2(&robot, kinova_left_bracelet_link, base_link,
                      fd_solver_robile_output_external_wrench_kl,
                      fd_solver_robile_output_external_wrench_kl);

    double fd_solver_robile_output_external_wrench_kr[6]{};
    decomposeSignal(&robot, kinova_right_base_link, kinova_right_bracelet_link,
                    kinova_right_bracelet_link, kr_bl_base_distance_pid_controller_signal,
                    fd_solver_robile_output_external_wrench_kr);

    transform_wrench2(&robot, kinova_right_bracelet_link, base_link,
                      fd_solver_robile_output_external_wrench_kr,
                      fd_solver_robile_output_external_wrench_kr);

    // std::cout << std::endl;

    // solvers
    // fd solver
    double fd_solver_robile_output_torques[8]{};
    double fd_solver_robile_platform_wrench[6]{};
    add(fd_solver_robile_output_external_wrench_kl, fd_solver_robile_platform_wrench,
        fd_solver_robile_platform_wrench, 6);
    add(fd_solver_robile_output_external_wrench_kr, fd_solver_robile_platform_wrench,
        fd_solver_robile_platform_wrench, 6);
    double plat_force[3] = {fd_solver_robile_platform_wrench[0],
                            fd_solver_robile_platform_wrench[1],
                            fd_solver_robile_platform_wrench[5]};

    std::cout << "plat_force: ";
    print_array(plat_force, 3);

    double lin_offsets[robot.mobile_base->mediator->kelo_base_config->nWheels];
    double ang_offsets[robot.mobile_base->mediator->kelo_base_config->nWheels];
    get_pivot_alignment_offsets(&robot, plat_force, lin_offsets, ang_offsets);

    double base_w1_lin_signal, base_w2_lin_signal, base_w3_lin_signal, base_w4_lin_signal = 0.0;
    double base_w1_ang_signal, base_w2_ang_signal, base_w3_ang_signal, base_w4_ang_signal = 0.0;

    double base_wheel_alignment_controller_kp[4] = {
        base_wheel_alignment_controller_Kp, base_wheel_alignment_controller_Kp,
        2.0 * base_wheel_alignment_controller_Kp, base_wheel_alignment_controller_Kp};
    double base_wheel_alignment_controller_ki[4] = {
        base_wheel_alignment_controller_Ki, base_wheel_alignment_controller_Ki,
        2.0 * base_wheel_alignment_controller_Ki, base_wheel_alignment_controller_Ki};
    double base_wheel_alignment_controller_kd[4] = {
        base_wheel_alignment_controller_Kd, base_wheel_alignment_controller_Kd,
        base_wheel_alignment_controller_Kd, base_wheel_alignment_controller_Kd};

    pidController(lin_offsets[0], base_wheel_alignment_controller_kp[0],
                  base_wheel_alignment_controller_ki[0], base_wheel_alignment_controller_kd[0],
                  *control_loop_dt, base_w1_lin_error_sum, 10.0, base_w1_lin_prev_error,
                  base_w1_lin_signal);
    pidController(lin_offsets[1], base_wheel_alignment_controller_kp[1],
                  base_wheel_alignment_controller_ki[1], base_wheel_alignment_controller_kd[1],
                  *control_loop_dt, base_w2_lin_error_sum, 10.0, base_w2_lin_prev_error,
                  base_w2_lin_signal);
    pidController(lin_offsets[2], base_wheel_alignment_controller_kp[2],
                  base_wheel_alignment_controller_ki[2], base_wheel_alignment_controller_kd[2],
                  *control_loop_dt, base_w3_lin_error_sum, 10.0, base_w3_lin_prev_error,
                  base_w3_lin_signal);
    pidController(lin_offsets[3], base_wheel_alignment_controller_kp[3],
                  base_wheel_alignment_controller_ki[3], base_wheel_alignment_controller_kd[3],
                  *control_loop_dt, base_w4_lin_error_sum, 10.0, base_w4_lin_prev_error,
                  base_w4_lin_signal);

    pidController(ang_offsets[0], base_wheel_alignment_controller_kp[0],
                  base_wheel_alignment_controller_ki[0], base_wheel_alignment_controller_kd[0],
                  *control_loop_dt, base_w1_ang_error_sum, 10.0, base_w1_ang_prev_error,
                  base_w1_ang_signal);
    pidController(ang_offsets[1], base_wheel_alignment_controller_kp[1],
                  base_wheel_alignment_controller_ki[1], base_wheel_alignment_controller_kd[1],
                  *control_loop_dt, base_w2_ang_error_sum, 10.0, base_w2_ang_prev_error,
                  base_w2_ang_signal);
    pidController(ang_offsets[2], base_wheel_alignment_controller_kp[2],
                  base_wheel_alignment_controller_ki[2], base_wheel_alignment_controller_kd[2],
                  *control_loop_dt, base_w3_ang_error_sum, 10.0, base_w3_ang_prev_error,
                  base_w3_ang_signal);
    pidController(ang_offsets[3], base_wheel_alignment_controller_kp[3],
                  base_wheel_alignment_controller_ki[3], base_wheel_alignment_controller_kd[3],
                  *control_loop_dt, base_w4_ang_error_sum, 10.0, base_w4_ang_prev_error,
                  base_w4_ang_signal);

    double wheel_alignment_lin_signals[4] = {base_w1_lin_signal, base_w2_lin_signal,
                                             base_w3_lin_signal, base_w4_lin_signal};
    double wheel_alignment_ang_signals[4] = {base_w1_ang_signal, base_w2_ang_signal,
                                             base_w3_ang_signal, base_w4_ang_signal};

    Eigen::Vector2d lin_pf = Eigen::Vector2d(plat_force[0], plat_force[1]);

    // compute the weights for the platform force
    double platform_weights[2];
    platform_weights[0] =
        abs(plat_force[2]) < 1e-6
            ? 1.0
            : sqrt(pow(plat_force[0], 2) + pow(plat_force[1], 2)) /
                  (sqrt(pow(plat_force[0], 2) + pow(plat_force[1], 2) + pow(plat_force[2], 2)));

    platform_weights[1] = 1.0 - platform_weights[0];

    double lin_force_weight = lin_pf.norm() == 0.0 ? 0.0 : platform_weights[0];
    double moment_weight = plat_force[2] == 0.0 ? 0.0 : platform_weights[1];

    double alignment_taus[robot.mobile_base->mediator->kelo_base_config->nWheels]{};
    for (size_t i = 0; i < robot.mobile_base->mediator->kelo_base_config->nWheels; i++)
    {
      alignment_taus[i] = wheel_alignment_lin_signals[i] * lin_force_weight +
                          wheel_alignment_ang_signals[i] * moment_weight;
    }

    double tau_wheel_ref[robot.mobile_base->mediator->kelo_base_config->nWheels * 2]{};
    for (size_t i = 0; i < robot.mobile_base->mediator->kelo_base_config->nWheels; i++)
    {
      tau_wheel_ref[2 * i] = alignment_taus[i];
      tau_wheel_ref[2 * i + 1] = -alignment_taus[i];
    }

    // for (size_t i = 0; i < robot.mobile_base->mediator->kelo_base_config->nWheels * 2; i++)
    // {
    //   if (tau_wheel_ref[i] > tau_wheel_ref_limit)
    //   {
    //     tau_wheel_ref[i] = tau_wheel_ref_limit;
    //   }
    //   else if (tau_wheel_ref[i] < -tau_wheel_ref_limit)
    //   {
    //     tau_wheel_ref[i] = -tau_wheel_ref_limit;
    //   }
    // }

    // base_fd_solver(&robot, plat_force, fd_solver_robile_output_torques);

    // for (size_t i = 0; i < robot.mobile_base->mediator->kelo_base_config->nWheels * 2; i++)
    // {
    //   fd_solver_robile_output_torques[i] += tau_wheel_ref[i];
    // }

    // achd_solver_fext
    double kl_achd_solver_fext_ext_wrenches[7][6];
    int link_id = -1;
    double
        kinova_left_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench_transf
            [6]{};
    transform_wrench(
        &robot, kinova_left_bracelet_link, kinova_left_base_link,
        kinova_left_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench,
        kinova_left_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench_transf);
    getLinkId(&robot, kinova_left_base_link, kinova_left_bracelet_link, kinova_left_bracelet_link,
              link_id);
    for (size_t i = 0; i < 6; i++)
    {
      kl_achd_solver_fext_ext_wrenches[link_id][i] =
          kinova_left_bracelet_table_contact_force_embed_map_kl_achd_solver_fext_output_external_wrench_transf
              [i];
    }

    double
        kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench_transf
            [6]{};
    transform_wrench(
        &robot, base_link, kinova_left_base_link,
        kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench,
        kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench_transf);
    getLinkId(&robot, kinova_left_base_link, kinova_left_bracelet_link,
              kinova_left_half_arm_2_link, link_id);
    for (size_t i = 0; i < 6; i++)
    {
      kl_achd_solver_fext_ext_wrenches[link_id][i] =
          kl_elbow_base_base_distance_z_embed_map_kl_achd_solver_fext_output_external_wrench_transf
              [i];
    }

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
    double kl_achd_solver_alpha_transf[kl_achd_solver_nc][6];
    transform_alpha(&robot, base_link, kinova_left_base_link, kl_achd_solver_alpha,
                    kl_achd_solver_nc, kl_achd_solver_alpha_transf);
    double kl_solver_beta[kl_achd_solver_nc]{};
    // for non-zero elements in the beta vector, add the corresponding twist to the beta vector
    int kl_s_i = 0;
    for (size_t i = 0; i < sizeof(kl_achd_solver_beta) / sizeof(kl_achd_solver_beta[0]); i++)
    {
      if (kl_achd_solver_beta[i] != 0.0 || kl_achd_solver_beta[i] != 0)
      {
        kl_solver_beta[kl_s_i] = kl_achd_solver_beta[i];
        kl_s_i++;
      }
    }
    achd_solver(&robot, kinova_left_base_link, kinova_left_bracelet_link, kl_achd_solver_nc,
                kl_achd_solver_root_acceleration, kl_achd_solver_alpha_transf, kl_solver_beta,
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
    double kr_achd_solver_alpha_transf[kr_achd_solver_nc][6];
    transform_alpha(&robot, base_link, kinova_right_base_link, kr_achd_solver_alpha,
                    kr_achd_solver_nc, kr_achd_solver_alpha_transf);

    double kr_solver_beta[kr_achd_solver_nc]{};
    // for non-zero elements in the beta vector, add the corresponding twist to the beta vector
    int kr_s_i = 0;
    for (size_t i = 0; i < sizeof(kr_achd_solver_beta) / sizeof(kr_achd_solver_beta[0]); i++)
    {
      if (kr_achd_solver_beta[i] != 0.0 || kr_achd_solver_beta[i] != 0)
      {
        kr_solver_beta[kr_s_i] = kr_achd_solver_beta[i];
        kr_s_i++;
      }
    }
    achd_solver(&robot, kinova_right_base_link, kinova_right_bracelet_link, kr_achd_solver_nc,
                kr_achd_solver_root_acceleration, kr_achd_solver_alpha_transf, kr_solver_beta,
                kr_achd_solver_feed_forward_torques, kr_achd_solver_predicted_accelerations,
                kr_achd_solver_output_torques);

    // achd_solver_fext
    double kr_achd_solver_fext_ext_wrenches[7][6]{};
    link_id = -1;
    double
        kinova_right_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench_transf
            [6]{};
    transform_wrench(
        &robot, kinova_right_bracelet_link, kinova_right_base_link,
        kinova_right_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench,
        kinova_right_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench_transf);
    getLinkId(&robot, kinova_right_base_link, kinova_right_bracelet_link,
              kinova_right_bracelet_link, link_id);
    for (size_t i = 0; i < 6; i++)
    {
      kr_achd_solver_fext_ext_wrenches[link_id][i] =
          kinova_right_bracelet_table_contact_force_embed_map_kr_achd_solver_fext_output_external_wrench_transf
              [i];
    }
    double
        kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench_transf
            [6]{};
    transform_wrench(
        &robot, base_link, kinova_right_base_link,
        kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench,
        kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench_transf);
    getLinkId(&robot, kinova_right_base_link, kinova_right_bracelet_link,
              kinova_right_half_arm_2_link, link_id);
    for (size_t i = 0; i < 6; i++)
    {
      kr_achd_solver_fext_ext_wrenches[link_id][i] =
          kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench_transf
              [i];
    }
    achd_solver_fext(&robot, kinova_right_base_link, kinova_right_bracelet_link,
                     kr_achd_solver_fext_ext_wrenches, kr_achd_solver_fext_output_torques);

    // Command the torques to the robots
    double kinova_right_cmd_tau[7]{};
    add(kr_achd_solver_output_torques, kinova_right_cmd_tau, kinova_right_cmd_tau, 7);
    add(kr_achd_solver_fext_output_torques, kinova_right_cmd_tau, kinova_right_cmd_tau, 7);

    double kinova_left_cmd_tau[7]{};
    add(kl_achd_solver_output_torques, kinova_left_cmd_tau, kinova_left_cmd_tau, 7);
    add(kl_achd_solver_fext_output_torques, kinova_left_cmd_tau, kinova_left_cmd_tau, 7);

    KDL::JntArray kinova_right_cmd_tau_kdl(7);
    cap_and_convert_manipulator_torques(kinova_right_cmd_tau, 7, kinova_right_cmd_tau_kdl);

    KDL::JntArray kinova_left_cmd_tau_kdl(7);
    cap_and_convert_manipulator_torques(kinova_left_cmd_tau, 7, kinova_left_cmd_tau_kdl);

    double base_tau_limit = 5.0;
    for (size_t i = 0; i < 8; i++)
    {
      if (fd_solver_robile_output_torques[i] > base_tau_limit)
      {
        fd_solver_robile_output_torques[i] = base_tau_limit;
      }
      else if (fd_solver_robile_output_torques[i] < -base_tau_limit)
      {
        fd_solver_robile_output_torques[i] = -base_tau_limit;
      }
    }

    kr_log_data_vec.addManipulatorData(robot.kinova_right, kr_achd_solver_beta,
                                       kinova_right_cmd_tau, nullptr);
    kl_log_data_vec.addManipulatorData(robot.kinova_left, kl_achd_solver_beta, kinova_left_cmd_tau,
                                       nullptr);
    base_log_data_vec.addMobileBaseData(robot.mobile_base, robot.mobile_base->state->x_platform,
                                        robot.mobile_base->state->xd_platform, plat_force,
                                        fd_solver_robile_output_torques);

    // set torques
    if (count > 1)
    {
      set_mobile_base_torques(&robot, fd_solver_robile_output_torques);
    }
    // set_manipulator_torques(&robot, kinova_left_base_link, &kinova_left_cmd_tau_kdl);
    // set_manipulator_torques(&robot, kinova_right_base_link, &kinova_right_cmd_tau_kdl);

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