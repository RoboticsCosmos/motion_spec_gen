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

int main()
{
  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  // set the base and tool links
  std::string base_link = "base_link";
  std::string tool_link_1 = "kinova_left_bracelet_link";
  std::string tool_link_2 = "kinova_right_bracelet_link";

  // initialize the chain
  KDL::Chain kinova_left_chain;
  initialize_robot_chain(robot_urdf, base_link, tool_link_1, kinova_left_chain);

  KDL::Chain kinova_right_chain;
  initialize_robot_chain(robot_urdf, base_link, tool_link_2, kinova_right_chain);

  // Initialize the robot structs
  Manipulator kinova_right_state;
  MobileBase freddy_base_state;
  Manipulator kinova_left_state;

  initialize_manipulator_state(kinova_right_chain.getNrOfJoints(),
                               kinova_right_chain.getNrOfSegments(), &kinova_right_state);
  initialize_mobile_base_state(&freddy_base_state);
  initialize_manipulator_state(kinova_left_chain.getNrOfJoints(),
                               kinova_left_chain.getNrOfSegments(), &kinova_left_state);

  // Initialize the robot connections
  // Initialize the Manipulator connections
  kinova_mediator *kinova_right_mediator = new kinova_mediator();
  kinova_right_mediator->initialize(0, 0, 0.0);
  kinova_right_mediator->set_control_mode(2);
  // Initialize the MobileBase connections
  KeloBaseConfig freddy_base_config;
  EthercatConfig freddy_base_ethercat_config;
  initialize_kelo_base(&freddy_base_config, &freddy_base_ethercat_config);
  int result = 0;
  establish_kelo_base_connection(&freddy_base_ethercat_config, "eno1", &result);
  if (result != 0)
  {
    printf("Failed to establish connection to KeloBase\n");
    exit(1);
  }
  // Initialize the Manipulator connections
  kinova_mediator *kinova_left_mediator = new kinova_mediator();
  kinova_left_mediator->initialize(0, 0, 0.0);
  kinova_left_mediator->set_control_mode(2);

  // initialize variables
  double achd_solver_kinova_left_output_torques[7]{};
  double kinova_left_bracelet_table_contact_force_lin_z_vector_z[6] = {0, 0, 1};
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_error_sum;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_signal;
  double *achd_solver_kinova_left_alpha[2] = {
      new double[6]{1.0, 0.0}, new double[6]{0.0, 1.0}, new double[6]{0.0, 0.0},
      new double[6]{0.0, 0.0}, new double[6]{0.0, 0.0}, new double[6]{0.0, 0.0}};
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
  double achd_solver_fext_kinova_right_root_acceleration[6] = {0.0, 0.0, 9.81,
                                                               0.0, 0.0, 0.0};
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_signal;
  double kinova_left_bracelet_base_lin_vel_y_embed_map_vector[6] = {0.0, 1.0, 0.0,
                                                                    0.0, 0.0, 0.0};
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_prev_error;
  double achd_solver_kinova_left_predicted_accelerations[7]{};
  double kinova_right_bracelet_base_lin_vel_x_embed_map_vector[6] = {1.0, 0.0, 0.0,
                                                                     0.0, 0.0, 0.0};
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
  double kinova_left_bracelet_base_distance_embed_map_vector[3] = {1.0, 1.0, 1.0};
  double achd_solver_kinova_right_root_acceleration[6] = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  double kinova_left_bracelet_base_distance_impedance_controller_stiffness_diag_mat[1] = {
      1.0};
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
  int achd_solver_kinova_right_nc = 2;
  double arms_distance_reference_value = 0.5;
  double kinova_right_bracelet_table_contact_force_embed_map_vector[6] = {0.0, 0.0, 1.0,
                                                                          0.0, 0.0, 0.0};
  double achd_solver_kinova_right_output_torques[7]{};
  int achd_solver_kinova_left_nc = 2;
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
  double *achd_solver_kinova_right_alpha[2] = {
      new double[6]{1.0, 0.0}, new double[6]{0.0, 1.0}, new double[6]{0.0, 0.0},
      new double[6]{0.0, 0.0}, new double[6]{0.0, 0.0}, new double[6]{0.0, 0.0}};
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_kp = 20.0;
  double achd_solver_kinova_left_root_acceleration[6] = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_signal;
  double kinova_right_bracelet_base_distance_impedance_controller_stiffness_diag_mat[1] =
      {1.0};
  double kinova_left_bracelet_base_lin_vel_x_pid_controller_ki = 0.9;
  double kinova_left_bracelet_table_contact_force_pid_controller_prev_error;
  double kinova_right_bracelet_base_vel_vector_lin_y[6] = {0, 1, 0, 0, 0, 0};
  double achd_solver_fext_kinova_left_root_acceleration[6] = {0.0, 0.0, 9.81,
                                                              0.0, 0.0, 0.0};
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

  while (true)
  {
    // Get the robot structs with the data from robots
    get_manipulator_data(&kinova_right_state, kinova_right_mediator);
    get_kelo_base_state(&freddy_base_config, &freddy_base_ethercat_config,
                        freddy_base_state.pivot_angles);
    get_manipulator_data(&kinova_left_state, kinova_left_mediator);

    // controllers
    // pid controller
    // TODO: computeForce
    // computeForce(kinova_left_bracelet_table_contact_force_lin_z);
    double error;
    computeEqualityError(kinova_left_bracelet_table_contact_force_lin_z,
                         arm_table_contact_force_reference_value, error);
    pidController(error, kinova_left_bracelet_table_contact_force_pid_controller_kp,
                  kinova_left_bracelet_table_contact_force_pid_controller_ki,
                  kinova_left_bracelet_table_contact_force_pid_controller_kd,
                  kinova_left_bracelet_table_contact_force_pid_controller_time_step,
                  kinova_left_bracelet_table_contact_force_pid_controller_error_sum,
                  kinova_left_bracelet_table_contact_force_pid_controller_prev_error,
                  kinova_left_bracelet_table_contact_force_pid_controller_signal);

    // pid controller
    computeForwardVelocityKinematics(kinova_left_bracelet_link, base_link, base_link,
                                     kinova_left_bracelet_base_vel_vector_lin_y,
                                     &kinova_left_state, &kinova_left_chain,
                                     of_vel_qname_lin_y);
    double error;
    computeEqualityError(of_vel_qname_lin_y, arm_bracelet_link_lin_xy_vel_reference_value,
                         error);
    pidController(error, kinova_left_bracelet_base_lin_vel_y_pid_controller_kp,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_ki,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_kd,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_time_step,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_error_sum,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_prev_error,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_signal);

    // pid controller
    // TODO: computeForce
    // computeForce(kinova_right_bracelet_table_contact_force_lin_z);
    double error;
    computeEqualityError(kinova_right_bracelet_table_contact_force_lin_z,
                         arm_table_contact_force_reference_value, error);
    pidController(error, kinova_right_bracelet_table_contact_force_pid_controller_kp,
                  kinova_right_bracelet_table_contact_force_pid_controller_ki,
                  kinova_right_bracelet_table_contact_force_pid_controller_kd,
                  kinova_right_bracelet_table_contact_force_pid_controller_time_step,
                  kinova_right_bracelet_table_contact_force_pid_controller_error_sum,
                  kinova_right_bracelet_table_contact_force_pid_controller_prev_error,
                  kinova_right_bracelet_table_contact_force_pid_controller_signal);

    // pid controller
    computeForwardVelocityKinematics(kinova_right_bracelet_link, base_link, base_link,
                                     kinova_right_bracelet_base_vel_vector_lin_x,
                                     &kinova_right_state, &kinova_right_chain,
                                     of_vel_qname_lin_x);
    double error;
    computeEqualityError(of_vel_qname_lin_x, arm_bracelet_link_lin_xy_vel_reference_value,
                         error);
    pidController(error, kinova_right_bracelet_base_lin_vel_x_pid_controller_kp,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_ki,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_kd,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_time_step,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_error_sum,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_prev_error,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_signal);

    // pid controller
    computeForwardVelocityKinematics(kinova_right_bracelet_link, base_link, base_link,
                                     kinova_right_bracelet_base_vel_vector_lin_y,
                                     &kinova_right_state, &kinova_right_chain,
                                     of_vel_qname_lin_y);
    double error;
    computeEqualityError(of_vel_qname_lin_y, arm_bracelet_link_lin_xy_vel_reference_value,
                         error);
    pidController(error, kinova_right_bracelet_base_lin_vel_y_pid_controller_kp,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_ki,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_kd,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_time_step,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_error_sum,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_prev_error,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_signal);

    // pid controller
    computeForwardVelocityKinematics(kinova_left_bracelet_link, base_link, base_link,
                                     kinova_left_bracelet_base_vel_vector_lin_x,
                                     &kinova_left_state, &kinova_left_chain,
                                     of_vel_qname_lin_x);
    double error;
    computeEqualityError(of_vel_qname_lin_x, arm_bracelet_link_lin_xy_vel_reference_value,
                         error);
    pidController(error, kinova_left_bracelet_base_lin_vel_x_pid_controller_kp,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_ki,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_kd,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_time_step,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_error_sum,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_prev_error,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_signal);

    // embed maps

    // solvers
  }

  return 0;
}