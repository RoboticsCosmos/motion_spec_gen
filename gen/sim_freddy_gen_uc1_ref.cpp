extern "C"
{
#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"
}

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
  Manipulator<kinova_mediator> kinova_right;
  kinova_right.base_frame = "kinova_right_base_link";
  kinova_right.tool_frame = "kinova_right_bracelet_link";
  kinova_right.mediator = nullptr;
  kinova_right.state = new ManipulatorState();

  Manipulator<kinova_mediator> kinova_left;
  kinova_left.base_frame = "kinova_left_base_link";
  kinova_left.tool_frame = "kinova_left_bracelet_link";
  kinova_left.mediator = nullptr;
  kinova_left.state = new ManipulatorState();

  MobileBase<Robile> freddy_base;
  Robile robile = {nullptr, new KeloBaseConfig()};
  freddy_base.mediator = &robile;
  freddy_base.state = new MobileBaseState();

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  Freddy freddy = {&kinova_left, &kinova_right, &freddy_base};

  initialize_robot_sim(robot_urdf, &freddy);

  // initialize variables
  double achd_solver_kinova_left_output_torques[7]{};
  double kinova_left_bracelet_table_contact_force_lin_z_vector_z[6] = {0, 0, 1};
  double kinova_right_bracelet_base_lin_vel_x_pid_controller_error_sum;
  double kinova_left_bracelet_base_lin_vel_y_pid_controller_time_step = 1;
  double kinova_right_bracelet_base_lin_vel_y_pid_controller_signal;
  double *achd_solver_kinova_left_alpha[2] = {
      new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0}};
  double kinova_right_bracelet_table_contact_force_pid_controller_error_sum;
  int achd_solver_fext_kinova_right_nj = 7;
  double kinova_right_bracelet_base_distance_impedance_controller_signal;
  double kinova_left_bracelet_table_contact_force_pid_controller_kp = 1.0;
  double kinova_right_bracelet_base_vel_vector_lin_x[6] = {1, 0, 0, 0, 0, 0};
  double kinova_left_bracelet_table_contact_force_pid_controller_ki = 0.0;
  double fd_solver_robile_root_acceleration[6] = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  double arm_bracelet_link_lin_xy_vel_reference_value = 0.0;
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
  std::string kinova_right_base_link = "kinova_right_base_link";
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
  double kinova_right_bracelet_table_contact_force_pid_controller_kp = 1.0;
  int achd_solver_kinova_right_nj = 7;
  double kinova_right_bracelet_table_contact_force_pid_controller_ki = 0.0;
  double kinova_right_bracelet_table_contact_force_pid_controller_kd = 0.0;
  int achd_solver_kinova_right_nc = 2;
  double arms_distance_reference_value = 0.5;
  double kinova_right_bracelet_table_contact_force_embed_map_vector[6] = {0.0, 0.0, 1.0,
                                                                          0.0, 0.0, 0.0};
  double achd_solver_kinova_right_output_torques[7]{};
  int achd_solver_kinova_left_nc = 2;
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
  double *achd_solver_kinova_right_alpha[2] = {
      new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0}};
  double fd_solver_robile_output_external_wrench[6]{};
  double fd_solver_robile_output_external_wrench2[6]{};
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

  set_init_sim_data(&freddy);

  int count = 0;
  while (count < 1)
  {
    count++;
    printf("\n--->count: %d\n", count);
    // Get the robot structs with the data from robots
    get_robot_data_sim(&freddy);

    // controllers
    // pid controller
    printf("first controller\n");
    getLinkForce(kinova_left_bracelet_link, table, kinova_left_bracelet_link,
                 kinova_left_bracelet_table_contact_force_lin_z_vector_z, &freddy,
                 kinova_left_bracelet_table_contact_force_lin_z);
    printf("kinova_left_bracelet_table_contact_force_lin_z: %f\n",
           kinova_left_bracelet_table_contact_force_lin_z);
    double kinova_left_bracelet_table_contact_force_pid_controller_error = 0;
    computeEqualityError(kinova_left_bracelet_table_contact_force_lin_z,
                         arm_table_contact_force_reference_value,
                         kinova_left_bracelet_table_contact_force_pid_controller_error);
    printf("kinova_left_bracelet_table_contact_force_pid_controller_error: %f\n",
           kinova_left_bracelet_table_contact_force_pid_controller_error);
    pidController(kinova_left_bracelet_table_contact_force_pid_controller_error,
                  kinova_left_bracelet_table_contact_force_pid_controller_kp,
                  kinova_left_bracelet_table_contact_force_pid_controller_ki,
                  kinova_left_bracelet_table_contact_force_pid_controller_kd,
                  kinova_left_bracelet_table_contact_force_pid_controller_time_step,
                  kinova_left_bracelet_table_contact_force_pid_controller_error_sum,
                  kinova_left_bracelet_table_contact_force_pid_controller_prev_error,
                  kinova_left_bracelet_table_contact_force_pid_controller_signal);
    printf("kinova_left_bracelet_table_contact_force_pid_controller_signal: %f\n",
           kinova_left_bracelet_table_contact_force_pid_controller_signal);
    printf("\n");

    // pid controller
    printf("second controller\n");
    getLinkVelocity(kinova_left_bracelet_link, base_link, base_link,
                    kinova_left_bracelet_base_vel_vector_lin_y, &freddy,
                    of_vel_qname_lin_y);
    printf("of_vel_qname_lin_y: %f\n", of_vel_qname_lin_y);
    double kinova_left_bracelet_base_lin_vel_y_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_lin_y, arm_bracelet_link_lin_xy_vel_reference_value,
                         kinova_left_bracelet_base_lin_vel_y_pid_controller_error);
    printf("kinova_left_bracelet_base_lin_vel_y_pid_controller_error: %f\n",
           kinova_left_bracelet_base_lin_vel_y_pid_controller_error);
    pidController(kinova_left_bracelet_base_lin_vel_y_pid_controller_error,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_kp,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_ki,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_kd,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_time_step,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_error_sum,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_prev_error,
                  kinova_left_bracelet_base_lin_vel_y_pid_controller_signal);
    printf("kinova_left_bracelet_base_lin_vel_y_pid_controller_signal: %f\n",
           kinova_left_bracelet_base_lin_vel_y_pid_controller_signal);
    printf("\n");

    // pid controller
    printf("third controller\n");
    getLinkForce(kinova_right_bracelet_link, table, kinova_right_bracelet_link,
                 kinova_right_bracelet_table_contact_force_lin_z_vector_z, &freddy,
                 kinova_right_bracelet_table_contact_force_lin_z);
    printf("kinova_right_bracelet_table_contact_force_lin_z: %f\n",
           kinova_right_bracelet_table_contact_force_lin_z);
    double kinova_right_bracelet_table_contact_force_pid_controller_error = 0;
    computeEqualityError(kinova_right_bracelet_table_contact_force_lin_z,
                         arm_table_contact_force_reference_value,
                         kinova_right_bracelet_table_contact_force_pid_controller_error);
    printf("kinova_right_bracelet_table_contact_force_pid_controller_error: %f\n",
           kinova_right_bracelet_table_contact_force_pid_controller_error);
    pidController(kinova_right_bracelet_table_contact_force_pid_controller_error,
                  kinova_right_bracelet_table_contact_force_pid_controller_kp,
                  kinova_right_bracelet_table_contact_force_pid_controller_ki,
                  kinova_right_bracelet_table_contact_force_pid_controller_kd,
                  kinova_right_bracelet_table_contact_force_pid_controller_time_step,
                  kinova_right_bracelet_table_contact_force_pid_controller_error_sum,
                  kinova_right_bracelet_table_contact_force_pid_controller_prev_error,
                  kinova_right_bracelet_table_contact_force_pid_controller_signal);
    printf("kinova_right_bracelet_table_contact_force_pid_controller_signal: %f\n",
           kinova_right_bracelet_table_contact_force_pid_controller_signal);
    printf("\n");

    // pid controller
    printf("fourth controller\n");
    getLinkVelocity(kinova_right_bracelet_link, base_link, base_link,
                    kinova_right_bracelet_base_vel_vector_lin_x, &freddy,
                    of_vel_qname_lin_x);
    printf("of_vel_qname_lin_x: %f\n", of_vel_qname_lin_x);
    double kinova_right_bracelet_base_lin_vel_x_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_lin_x, arm_bracelet_link_lin_xy_vel_reference_value,
                         kinova_right_bracelet_base_lin_vel_x_pid_controller_error);
    printf("kinova_right_bracelet_base_lin_vel_x_pid_controller_error: %f\n",
           kinova_right_bracelet_base_lin_vel_x_pid_controller_error);
    pidController(kinova_right_bracelet_base_lin_vel_x_pid_controller_error,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_kp,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_ki,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_kd,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_time_step,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_error_sum,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_prev_error,
                  kinova_right_bracelet_base_lin_vel_x_pid_controller_signal);
    printf("kinova_right_bracelet_base_lin_vel_x_pid_controller_signal: %f\n",
           kinova_right_bracelet_base_lin_vel_x_pid_controller_signal);
    printf("\n");

    // pid controller
    printf("fifth controller\n");
    getLinkVelocity(kinova_right_bracelet_link, base_link, base_link,
                    kinova_right_bracelet_base_vel_vector_lin_y, &freddy,
                    of_vel_qname_lin_y);
    printf("of_vel_qname_lin_y: %f\n", of_vel_qname_lin_y);
    double kinova_right_bracelet_base_lin_vel_y_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_lin_y, arm_bracelet_link_lin_xy_vel_reference_value,
                         kinova_right_bracelet_base_lin_vel_y_pid_controller_error);
    printf("kinova_right_bracelet_base_lin_vel_y_pid_controller_error: %f\n",
           kinova_right_bracelet_base_lin_vel_y_pid_controller_error);
    pidController(kinova_right_bracelet_base_lin_vel_y_pid_controller_error,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_kp,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_ki,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_kd,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_time_step,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_error_sum,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_prev_error,
                  kinova_right_bracelet_base_lin_vel_y_pid_controller_signal);
    printf("kinova_right_bracelet_base_lin_vel_y_pid_controller_signal: %f\n",
           kinova_right_bracelet_base_lin_vel_y_pid_controller_signal);
    printf("\n");

    // impedance controller
    printf("sixth controller\n");
    double kinova_right_bracelet_base_distance_impedance_controller_stiffness_error = 0;
    computeDistance(
        new std::string[2]{kinova_right_bracelet_link, kinova_right_base_link},
        kinova_right_bracelet_link, &freddy, kinova_right_bracelet_base_distance);
    printf("kinova_right_bracelet_base_distance: %f\n",
           kinova_right_bracelet_base_distance);
    computeEqualityError(
        kinova_right_bracelet_base_distance, arms_distance_reference_value,
        kinova_right_bracelet_base_distance_impedance_controller_stiffness_error);
    printf(
        "kinova_right_bracelet_base_distance_impedance_controller_stiffness_error: %f\n",
        kinova_right_bracelet_base_distance_impedance_controller_stiffness_error);
    impedanceController(
        kinova_right_bracelet_base_distance_impedance_controller_stiffness_error, 0.0,
        kinova_right_bracelet_base_distance_impedance_controller_stiffness_diag_mat,
        new double[1]{0.0},
        kinova_right_bracelet_base_distance_impedance_controller_signal);
    printf("kinova_right_bracelet_base_distance_impedance_controller_signal: %f\n",
           kinova_right_bracelet_base_distance_impedance_controller_signal);
    printf("\n");

    // impedance controller
    printf("seventh controller\n");
    double kinova_left_bracelet_base_distance_impedance_controller_stiffness_error = 0;
    computeDistance(new std::string[2]{kinova_left_bracelet_link, kinova_left_base_link},
                    kinova_left_bracelet_link, &freddy,
                    kinova_left_bracelet_base_distance);
    printf("kinova_left_bracelet_base_distance: %f\n",
           kinova_left_bracelet_base_distance);
    computeEqualityError(
        kinova_left_bracelet_base_distance, arms_distance_reference_value,
        kinova_left_bracelet_base_distance_impedance_controller_stiffness_error);
    printf(
        "kinova_left_bracelet_base_distance_impedance_controller_stiffness_error: %f\n",
        kinova_left_bracelet_base_distance_impedance_controller_stiffness_error);
    impedanceController(
        kinova_left_bracelet_base_distance_impedance_controller_stiffness_error, 0.0,
        kinova_left_bracelet_base_distance_impedance_controller_stiffness_diag_mat,
        new double[1]{0.0},
        kinova_left_bracelet_base_distance_impedance_controller_signal);
    printf("kinova_left_bracelet_base_distance_impedance_controller_signal: %f\n",
           kinova_left_bracelet_base_distance_impedance_controller_signal);
    printf("\n");

    // pid controller
    printf("eighth controller\n");
    getLinkVelocity(kinova_left_bracelet_link, base_link, base_link,
                    kinova_left_bracelet_base_vel_vector_lin_x, &freddy,
                    of_vel_qname_lin_x);
    printf("of_vel_qname_lin_x: %f\n", of_vel_qname_lin_x);
    double kinova_left_bracelet_base_lin_vel_x_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_lin_x, arm_bracelet_link_lin_xy_vel_reference_value,
                         kinova_left_bracelet_base_lin_vel_x_pid_controller_error);
    printf("kinova_left_bracelet_base_lin_vel_x_pid_controller_error: %f\n",
           kinova_left_bracelet_base_lin_vel_x_pid_controller_error);
    pidController(kinova_left_bracelet_base_lin_vel_x_pid_controller_error,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_kp,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_ki,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_kd,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_time_step,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_error_sum,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_prev_error,
                  kinova_left_bracelet_base_lin_vel_x_pid_controller_signal);
    printf("kinova_left_bracelet_base_lin_vel_x_pid_controller_signal: %f\n",
           kinova_left_bracelet_base_lin_vel_x_pid_controller_signal);
    printf("\n");

    // embed maps
    printf("embed maps\n");
    for (size_t i = 0;
         i < sizeof(kinova_right_bracelet_base_lin_vel_x_embed_map_vector) /
                 sizeof(kinova_right_bracelet_base_lin_vel_x_embed_map_vector[0]);
         i++)
    {
      if (kinova_right_bracelet_base_lin_vel_x_embed_map_vector[i] != 0.0)
      {
        achd_solver_kinova_right_output_acceleration_energy[i] +=
            kinova_right_bracelet_base_lin_vel_x_pid_controller_signal;
      }
    }
    for (size_t i = 0;
         i < sizeof(kinova_right_bracelet_base_lin_vel_y_twist_embed_map_vector) /
                 sizeof(kinova_right_bracelet_base_lin_vel_y_twist_embed_map_vector[0]);
         i++)
    {
      if (kinova_right_bracelet_base_lin_vel_y_twist_embed_map_vector[i] != 0.0)
      {
        achd_solver_kinova_right_output_acceleration_energy[i] +=
            kinova_right_bracelet_base_lin_vel_y_pid_controller_signal;
      }
    }
    for (size_t i = 0;
         i < sizeof(kinova_left_bracelet_table_contact_force_embed_map_vector) /
                 sizeof(kinova_left_bracelet_table_contact_force_embed_map_vector[0]);
         i++)
    {
      if (kinova_left_bracelet_table_contact_force_embed_map_vector[i] != 0.0)
      {
        achd_solver_fext_kinova_left_output_external_wrench[i] +=
            kinova_left_bracelet_table_contact_force_pid_controller_signal;
      }
    }
    decomposeSignal(&freddy, kinova_left_base_link, kinova_left_bracelet_link,
                    kinova_left_base_link,
                    kinova_left_bracelet_base_distance_impedance_controller_signal,
                    fd_solver_robile_output_external_wrench);
    std::cout << "kl fd_solver_robile_output_external_wrench: ";
    for (size_t i = 0; i < 6; i++)
    {
      std::cout << fd_solver_robile_output_external_wrench[i] << " ";
    }
    std::cout << std::endl;
    transform_wrench(&freddy, kinova_left_base_link, base_link,
                     fd_solver_robile_output_external_wrench,
                     fd_solver_robile_output_external_wrench);
    std::cout << "kl fd_solver_robile_output_external_wrench transformed: ";
    for (size_t i = 0; i < 6; i++)
    {
      std::cout << fd_solver_robile_output_external_wrench[i] << " ";
    }
    std::cout << std::endl;
    printf("\n");
    decomposeSignal(&freddy, kinova_right_base_link, kinova_right_bracelet_link,
                    kinova_right_base_link,
                    kinova_right_bracelet_base_distance_impedance_controller_signal,
                    fd_solver_robile_output_external_wrench2);
    std::cout << "kr fd_solver_robile_output_external_wrench2: ";
    for (size_t i = 0; i < 6; i++)
    {
      std::cout << fd_solver_robile_output_external_wrench2[i] << " ";
    }
    std::cout << std::endl;
    transform_wrench(&freddy, kinova_right_base_link, base_link,
                     fd_solver_robile_output_external_wrench2,
                     fd_solver_robile_output_external_wrench2);
    std::cout << "kr fd_solver_robile_output_external_wrench2 transformed: ";
    for (size_t i = 0; i < 6; i++)
    {
      std::cout << fd_solver_robile_output_external_wrench2[i] << " ";
    }
    std::cout << std::endl;
    printf("\n");
    for (size_t i = 0;
         i < sizeof(kinova_left_bracelet_base_lin_vel_x_twist_embed_map_vector) /
                 sizeof(kinova_left_bracelet_base_lin_vel_x_twist_embed_map_vector[0]);
         i++)
    {
      if (kinova_left_bracelet_base_lin_vel_x_twist_embed_map_vector[i] != 0.0)
      {
        achd_solver_kinova_left_output_acceleration_energy[i] +=
            kinova_left_bracelet_base_lin_vel_x_pid_controller_signal;
      }
    }
    for (size_t i = 0;
         i < sizeof(kinova_left_bracelet_base_lin_vel_y_embed_map_vector) /
                 sizeof(kinova_left_bracelet_base_lin_vel_y_embed_map_vector[0]);
         i++)
    {
      if (kinova_left_bracelet_base_lin_vel_y_embed_map_vector[i] != 0.0)
      {
        achd_solver_kinova_left_output_acceleration_energy[i] +=
            kinova_left_bracelet_base_lin_vel_y_pid_controller_signal;
      }
    }
    for (size_t i = 0;
         i < sizeof(kinova_right_bracelet_table_contact_force_embed_map_vector) /
                 sizeof(kinova_right_bracelet_table_contact_force_embed_map_vector[0]);
         i++)
    {
      if (kinova_right_bracelet_table_contact_force_embed_map_vector[i] != 0.0)
      {
        achd_solver_fext_kinova_right_output_external_wrench[i] +=
            kinova_right_bracelet_table_contact_force_pid_controller_signal;
      }
    }

    // solvers
    printf("solvers\n");

    // achd_solver
    printf("first solver\n");
    double achd_solver_kinova_right_beta[6]{};
    for (size_t i = 0; i < 6; i++)
    {
      achd_solver_kinova_right_beta[i] = achd_solver_kinova_right_root_acceleration[i];
    }
    add(achd_solver_kinova_right_output_acceleration_energy,
        achd_solver_kinova_right_beta, achd_solver_kinova_right_beta, 6);
    double *achd_solver_kinova_right_alpha_transf[2] = {new double[6]{}, new double[6]{}};
    double achd_solver_kinova_right_beta_transf[6]{};
    transform_alpha_beta(
        &freddy, base_link, kinova_right_base_link, achd_solver_kinova_right_alpha,
        achd_solver_kinova_right_beta, achd_solver_kinova_right_nc,
        achd_solver_kinova_right_alpha_transf, achd_solver_kinova_right_beta_transf);
    achd_solver(&freddy, kinova_right_base_link, kinova_right_bracelet_link,
                achd_solver_kinova_right_nc, achd_solver_kinova_right_root_acceleration,
                achd_solver_kinova_right_alpha_transf,
                achd_solver_kinova_right_beta_transf,
                achd_solver_kinova_right_feed_forward_torques,
                achd_solver_kinova_right_predicted_accelerations,
                achd_solver_kinova_right_output_torques);

    // achd_solver_fext
    printf("second solver\n");
    double achd_solver_fext_kinova_left_ext_wrench_tool[6]{};
    add(achd_solver_fext_kinova_left_output_external_wrench,
        achd_solver_fext_kinova_left_ext_wrench_tool,
        achd_solver_fext_kinova_left_ext_wrench_tool, 6);
    transform_wrench(&freddy, base_link, kinova_left_base_link,
                     achd_solver_fext_kinova_left_ext_wrench_tool,
                     achd_solver_fext_kinova_left_ext_wrench_tool);
    achd_solver_fext(&freddy, kinova_left_base_link, kinova_left_bracelet_link,
                     achd_solver_fext_kinova_left_ext_wrench_tool,
                     achd_solver_fext_kinova_left_output_torques);

    // base_fd_solver
    printf("third solver\n");
    double fd_solver_robile_platform_force[6]{};
    add(fd_solver_robile_output_external_wrench, fd_solver_robile_platform_force,
        fd_solver_robile_platform_force, 6);
    add(fd_solver_robile_output_external_wrench2, fd_solver_robile_platform_force,
        fd_solver_robile_platform_force, 6);
    // double fd_solver_robile_platform_force1[3] = {fd_solver_robile_platform_force[0],
    //                                                fd_solver_robile_platform_force[1],
    //                                                fd_solver_robile_platform_force[5]};
    double fd_solver_robile_platform_force1[3] = {0., 0., 0.};
    printf("fd_solver_robile_platform_force1: ");
    for (size_t i = 0; i < 3; i++)
    {
      std::cout << fd_solver_robile_platform_force1[i] << " ";
    }
    std::cout << std::endl;
    base_fd_solver(&freddy, fd_solver_robile_platform_force,
                   fd_solver_robile_output_torques);

    // achd_solver
    printf("fourth solver\n");
    double achd_solver_kinova_left_beta[6]{};
    for (size_t i = 0; i < 6; i++)
    {
      achd_solver_kinova_left_beta[i] = achd_solver_kinova_left_root_acceleration[i];
    }
    add(achd_solver_kinova_left_output_acceleration_energy,
    achd_solver_kinova_left_beta,
        achd_solver_kinova_left_beta, 6);
    transform_alpha_beta(&freddy, base_link, kinova_left_base_link,
                         achd_solver_kinova_left_alpha, achd_solver_kinova_left_beta,
                         achd_solver_kinova_left_nc, achd_solver_kinova_left_alpha,
                         achd_solver_kinova_left_beta);
    achd_solver(&freddy, kinova_left_base_link, kinova_left_bracelet_link,
                achd_solver_kinova_left_nc, achd_solver_kinova_left_root_acceleration,
                achd_solver_kinova_left_alpha, achd_solver_kinova_left_beta,
                achd_solver_kinova_left_feed_forward_torques,
                achd_solver_kinova_left_predicted_accelerations,
                achd_solver_kinova_left_output_torques);

    // achd_solver_fext
    printf("fifth solver\n");
    double achd_solver_fext_kinova_right_ext_wrench_tool[6]{};
    add(achd_solver_fext_kinova_right_output_external_wrench,
        achd_solver_fext_kinova_right_ext_wrench_tool,
        achd_solver_fext_kinova_right_ext_wrench_tool, 6);
    transform_wrench(&freddy, base_link, kinova_right_base_link,
                     achd_solver_fext_kinova_right_ext_wrench_tool,
                     achd_solver_fext_kinova_right_ext_wrench_tool);
    achd_solver_fext(&freddy, kinova_right_base_link, kinova_right_bracelet_link,
                     achd_solver_fext_kinova_right_ext_wrench_tool,
                     achd_solver_fext_kinova_right_output_torques);


    std::cout << "end of loop\n";
    // Command the torques to the robots
  }

  return 0;
}