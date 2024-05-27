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
  kinova_right.mediator = new kinova_mediator();
  kinova_right.state = new ManipulatorState();
  bool kinova_right_torque_control_mode_set = false;


  Freddy robot = { &kinova_right };

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  char *ethernet_interface = "eno1";
  initialize_robot(robot_urdf, ethernet_interface, &robot);

  // initialize variables
  double kr_vel_twist_lin_z_pid_controller_time_step = 1; 
  double kr_vel_twist_ang_y_pid_controller_error_sum = 0.0; 
  double kr_vel_bl_wrt_base_vector_ang_y[6] = { 0,0,0,0,1,0 }; 
  double kr_vel_bl_wrt_base_vector_ang_x[6] = { 0,0,0,1,0,0 }; 
  double kr_vel_twist_ang_x_pid_controller_prev_error = 0.0; 
  double kr_vel_twist_ang_y_pid_controller_kd = 0.0; 
  double pid_kr_vel_ang_y_twist_embed_map_vector[6] = { 0.0,0.0,0.0,0.0,1.0,0.0 }; 
  double kr_vel_bl_wrt_base_vector_ang_z[6] = { 0,0,0,0,0,1 }; 
  double pid_kr_vel_lin_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{}; 
  double *kr_achd_solver_alpha[6] = { new double[6]{ 1.0,0.0,0.0,0.0,0.0,0.0 }, new double[6]{ 0.0,1.0,0.0,0.0,0.0,0.0 }, new double[6]{ 0.0,0.0,1.0,0.0,0.0,0.0 }, new double[6]{ 0.0,0.0,0.0,1.0,0.0,0.0 }, new double[6]{ 0.0,0.0,0.0,0.0,1.0,0.0 }, new double[6]{ 0.0,0.0,0.0,0.0,0.0,1.0 } }; 
  double kr_elbow_base_z_distance_reference_value = 0.8; 
  double kr_vel_twist_ang_z_pid_controller_signal = 0.0; 
  double kr_vel_twist_lin_x_pid_controller_signal = 0.0; 
  double kr_vel_twist_lin_x_pid_controller_ki = 0.9; 
  double kr_vel_twist_lin_x_pid_controller_kd = 0.0; 
  double kr_vel_twist_ang_y_pid_controller_prev_error = 0.0; 
  int kr_achd_solver_fext_nj = 7; 
  double kr_achd_solver_fext_output_torques[7]{}; 
  double kr_vel_twist_lin_x_pid_controller_kp = 20.0; 
  double kr_vel_twist_ang_y_pid_controller_kp = 20.0; 
  double kr_vel_twist_lin_x_pid_controller_prev_error = 0.0; 
  double kr_vel_twist_ang_y_pid_controller_ki = 0.9; 
  double kr_achd_solver_output_torques[7]{}; 
  std::string base_link = "base_link"; 
  int kr_achd_solver_nc = 6; 
  double kr_vel_twist_lin_z_pid_controller_ki = 0.9; 
  double kr_vel_twist_lin_y_pid_controller_signal = 0.0; 
  double kr_vel_twist_lin_z_pid_controller_kp = 20.0; 
  double kr_vel_twist_ang_z_pid_controller_prev_error = 0.0; 
  double kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench[6]{}; 
  int kr_achd_solver_nj = 7; 
  double pid_kr_vel_ang_x_twist_embed_map_vector[6] = { 0.0,0.0,0.0,1.0,0.0,0.0 }; 
  double kr_vel_twist_ang_z_pid_controller_ki = 0.9; 
  double kr_vel_twist_lin_z_pid_controller_kd = 0.0; 
  double kr_elbow_base_base_distance_z_impedance_controller_signal = 0.0; 
  double kr_vel_twist_lin_y_pid_controller_prev_error = 0.0; 
  double kr_vel_twist_ang_z_pid_controller_kp = 20.0; 
  std::string kinova_right_bracelet_link = "kinova_right_bracelet_link"; 
  double kr_vel_twist_lin_y_pid_controller_kp = 20.0; 
  double kr_achd_solver_feed_forward_torques[7]{}; 
  double pid_kr_vel_lin_x_twist_embed_map_vector[6] = { 1.0,0.0,0.0,0.0,0.0,0.0 }; 
  double kr_vel_twist_lin_z_pid_controller_signal = 0.0; 
  double kr_vel_twist_lin_y_pid_controller_ki = 0.9; 
  double kr_vel_twist_ang_z_pid_controller_kd = 0.0; 
  double kr_vel_twist_lin_y_pid_controller_kd = 0.0; 
  double kr_vel_twist_lin_z_pid_controller_prev_error = 0.0; 
  double kr_achd_solver_predicted_accelerations[7]{}; 
  double kr_elbow_base_base_distance_z_embed_map_vector[3] = { 0.0,0.0,1.0 }; 
  double kr_vel_twist_lin_y_pid_controller_time_step = 1; 
  double kr_vel_twist_ang_x_pid_controller_error_sum = 0.0; 
  double kr_achd_solver_root_acceleration[6] = { -9.685,-1.033,1.324,0.0,0.0,0.0 }; 
  double kr_elbow_base_base_distance_z_impedance_controller_stiffness_diag_mat[1] = { 10.0 }; 
  double pid_kr_vel_lin_y_twist_embed_map_vector[6] = { 0.0,1.0,0.0,0.0,0.0,0.0 }; 
  double pid_kr_vel_ang_x_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{}; 
  double pid_kr_vel_ang_z_twist_embed_map_vector[6] = { 0.0,0.0,0.0,0.0,0.0,1.0 }; 
  double kr_vel_bl_wrt_base_vector_lin_x[6] = { 1,0,0,0,0,0 }; 
  double pid_kr_vel_ang_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{}; 
  double kr_vel_twist_ang_x_pid_controller_time_step = 1; 
  std::string kinova_right_base_link = "kinova_right_base_link"; 
  double kr_elbow_base_distance = 0.0; 
  double kr_vel_twist_lin_z_pid_controller_error_sum = 0.0; 
  double kr_vel_twist_ang_y_pid_controller_time_step = 1; 
  double kr_vel_bl_wrt_base_vector_lin_y[6] = { 0,1,0,0,0,0 }; 
  double kr_vel_bl_wrt_base_vector_lin_z[6] = { 0,0,1,0,0,0 }; 
  double kr_vel_twist_ang_z_pid_controller_time_step = 1; 
  double kr_vel_twist_ang_x_pid_controller_signal = 0.0; 
  std::string kinova_right_half_arm_2_link = "kinova_right_half_arm_2_link"; 
  double kr_vel_twist_ang_x_pid_controller_kd = 0.0; 
  double pid_kr_vel_ang_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{}; 
  double kr_vel_twist_ang_x_pid_controller_ki = 0.9; 
  double of_vel_qname_lin_x = 0.0; 
  double kr_vel_twist_lin_x_pid_controller_error_sum = 0.0; 
  double of_vel_qname_lin_y = 0.0; 
  double of_vel_qname_lin_z = 0.0; 
  double kr_vel_twist_lin_x_pid_controller_time_step = 1; 
  double of_vel_qname_ang_y = 0.0; 
  double of_vel_qname_ang_x = 0.0; 
  double pid_kr_vel_lin_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{}; 
  double pid_kr_vel_lin_z_twist_embed_map_vector[6] = { 0.0,0.0,1.0,0.0,0.0,0.0 }; 
  double kr_vel_twist_ang_x_pid_controller_kp = 20.0; 
  double kr_bl_vel_twist_reference_value = 0.0; 
  double of_vel_qname_ang_z = 0.0; 
  double kr_vel_twist_lin_y_pid_controller_error_sum = 0.0; 
  double kr_vel_twist_ang_z_pid_controller_error_sum = 0.0; 
  double kr_vel_twist_ang_y_pid_controller_signal = 0.0; 
  double pid_kr_vel_lin_x_twist_embed_map_kr_achd_solver_output_acceleration_energy[6]{}; 

  int count = 0;
  const double desired_frequency = 900.0;  // Hz
  const auto desired_period =
  std::chrono::duration<double>(1.0 / desired_frequency);  // s

  while (true) {
    auto start_time = std::chrono::high_resolution_clock::now();

    count++;
    std::cout << "count: " << count << std::endl;

    get_robot_data(&robot);

    // controllers
    // pid controller
    getLinkVelocity(kinova_right_bracelet_link, base_link, base_link, kr_vel_bl_wrt_base_vector_lin_y, &robot, of_vel_qname_lin_y);
    double kr_vel_twist_lin_y_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_lin_y, kr_bl_vel_twist_reference_value, kr_vel_twist_lin_y_pid_controller_error);
    pidController(kr_vel_twist_lin_y_pid_controller_error, kr_vel_twist_lin_y_pid_controller_kp, kr_vel_twist_lin_y_pid_controller_ki, kr_vel_twist_lin_y_pid_controller_kd, kr_vel_twist_lin_y_pid_controller_time_step, kr_vel_twist_lin_y_pid_controller_error_sum, kr_vel_twist_lin_y_pid_controller_prev_error, kr_vel_twist_lin_y_pid_controller_signal);

    // pid controller
    getLinkVelocity(kinova_right_bracelet_link, base_link, base_link, kr_vel_bl_wrt_base_vector_lin_z, &robot, of_vel_qname_lin_z);
    double kr_vel_twist_lin_z_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_lin_z, kr_bl_vel_twist_reference_value, kr_vel_twist_lin_z_pid_controller_error);
    pidController(kr_vel_twist_lin_z_pid_controller_error, kr_vel_twist_lin_z_pid_controller_kp, kr_vel_twist_lin_z_pid_controller_ki, kr_vel_twist_lin_z_pid_controller_kd, kr_vel_twist_lin_z_pid_controller_time_step, kr_vel_twist_lin_z_pid_controller_error_sum, kr_vel_twist_lin_z_pid_controller_prev_error, kr_vel_twist_lin_z_pid_controller_signal);

    // pid controller
    getLinkVelocity(kinova_right_bracelet_link, base_link, base_link, kr_vel_bl_wrt_base_vector_lin_x, &robot, of_vel_qname_lin_x);
    double kr_vel_twist_lin_x_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_lin_x, kr_bl_vel_twist_reference_value, kr_vel_twist_lin_x_pid_controller_error);
    pidController(kr_vel_twist_lin_x_pid_controller_error, kr_vel_twist_lin_x_pid_controller_kp, kr_vel_twist_lin_x_pid_controller_ki, kr_vel_twist_lin_x_pid_controller_kd, kr_vel_twist_lin_x_pid_controller_time_step, kr_vel_twist_lin_x_pid_controller_error_sum, kr_vel_twist_lin_x_pid_controller_prev_error, kr_vel_twist_lin_x_pid_controller_signal);

    // impedance controller
    double kr_elbow_base_base_distance_z_impedance_controller_stiffness_error = 0;
    impedanceController(kr_elbow_base_base_distance_z_impedance_controller_stiffness_error, 0.0, kr_elbow_base_base_distance_z_impedance_controller_stiffness_diag_mat, new double[1]{0.0}, kr_elbow_base_base_distance_z_impedance_controller_signal);

    // pid controller
    getLinkVelocity(kinova_right_bracelet_link, base_link, base_link, kr_vel_bl_wrt_base_vector_ang_y, &robot, of_vel_qname_ang_y);
    double kr_vel_twist_ang_y_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_ang_y, kr_bl_vel_twist_reference_value, kr_vel_twist_ang_y_pid_controller_error);
    pidController(kr_vel_twist_ang_y_pid_controller_error, kr_vel_twist_ang_y_pid_controller_kp, kr_vel_twist_ang_y_pid_controller_ki, kr_vel_twist_ang_y_pid_controller_kd, kr_vel_twist_ang_y_pid_controller_time_step, kr_vel_twist_ang_y_pid_controller_error_sum, kr_vel_twist_ang_y_pid_controller_prev_error, kr_vel_twist_ang_y_pid_controller_signal);

    // pid controller
    getLinkVelocity(kinova_right_bracelet_link, base_link, base_link, kr_vel_bl_wrt_base_vector_ang_z, &robot, of_vel_qname_ang_z);
    double kr_vel_twist_ang_z_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_ang_z, kr_bl_vel_twist_reference_value, kr_vel_twist_ang_z_pid_controller_error);
    pidController(kr_vel_twist_ang_z_pid_controller_error, kr_vel_twist_ang_z_pid_controller_kp, kr_vel_twist_ang_z_pid_controller_ki, kr_vel_twist_ang_z_pid_controller_kd, kr_vel_twist_ang_z_pid_controller_time_step, kr_vel_twist_ang_z_pid_controller_error_sum, kr_vel_twist_ang_z_pid_controller_prev_error, kr_vel_twist_ang_z_pid_controller_signal);

    // pid controller
    getLinkVelocity(kinova_right_bracelet_link, base_link, base_link, kr_vel_bl_wrt_base_vector_ang_x, &robot, of_vel_qname_ang_x);
    double kr_vel_twist_ang_x_pid_controller_error = 0;
    computeEqualityError(of_vel_qname_ang_x, kr_bl_vel_twist_reference_value, kr_vel_twist_ang_x_pid_controller_error);
    pidController(kr_vel_twist_ang_x_pid_controller_error, kr_vel_twist_ang_x_pid_controller_kp, kr_vel_twist_ang_x_pid_controller_ki, kr_vel_twist_ang_x_pid_controller_kd, kr_vel_twist_ang_x_pid_controller_time_step, kr_vel_twist_ang_x_pid_controller_error_sum, kr_vel_twist_ang_x_pid_controller_prev_error, kr_vel_twist_ang_x_pid_controller_signal);



  }

  free_robot_data(&robot);

  return 0;
}