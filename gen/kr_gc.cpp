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
  double kr_elbow_base_distance_axis[6] = { 0,0,1 }; 
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

    if (flag)
    {
      free_robot_data(&robot);
      printf("Exiting somewhat cleanly...\n");
      exit(0);
    }

    count++;
    printf("count: %d\n", count);

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
    computeDistance1D(new std::string[2]{ kinova_right_half_arm_2_link, base_link }, kr_elbow_base_distance_axis, base_link, &robot, kr_elbow_base_distance);
    computeEqualityError(kr_elbow_base_distance, kr_elbow_base_z_distance_reference_value, kr_elbow_base_base_distance_z_impedance_controller_stiffness_error);
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


    // embed maps
    for (size_t i = 0; i < sizeof(pid_kr_vel_lin_x_twist_embed_map_vector)/sizeof(pid_kr_vel_lin_x_twist_embed_map_vector[0]); i++)
    {
      if (pid_kr_vel_lin_x_twist_embed_map_vector[i] != 0.0)
      {
        pid_kr_vel_lin_x_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] += kr_vel_twist_lin_x_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(pid_kr_vel_lin_y_twist_embed_map_vector)/sizeof(pid_kr_vel_lin_y_twist_embed_map_vector[0]); i++)
    {
      if (pid_kr_vel_lin_y_twist_embed_map_vector[i] != 0.0)
      {
        pid_kr_vel_lin_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] += kr_vel_twist_lin_y_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(pid_kr_vel_lin_z_twist_embed_map_vector)/sizeof(pid_kr_vel_lin_z_twist_embed_map_vector[0]); i++)
    {
      if (pid_kr_vel_lin_z_twist_embed_map_vector[i] != 0.0)
      {
        pid_kr_vel_lin_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] += kr_vel_twist_lin_z_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(pid_kr_vel_ang_x_twist_embed_map_vector)/sizeof(pid_kr_vel_ang_x_twist_embed_map_vector[0]); i++)
    {
      if (pid_kr_vel_ang_x_twist_embed_map_vector[i] != 0.0)
      {
        pid_kr_vel_ang_x_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] += kr_vel_twist_ang_x_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(pid_kr_vel_ang_y_twist_embed_map_vector)/sizeof(pid_kr_vel_ang_y_twist_embed_map_vector[0]); i++)
    {
      if (pid_kr_vel_ang_y_twist_embed_map_vector[i] != 0.0)
      {
        pid_kr_vel_ang_y_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] += kr_vel_twist_ang_y_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(pid_kr_vel_ang_z_twist_embed_map_vector)/sizeof(pid_kr_vel_ang_z_twist_embed_map_vector[0]); i++)
    {
      if (pid_kr_vel_ang_z_twist_embed_map_vector[i] != 0.0)
      {
        pid_kr_vel_ang_z_twist_embed_map_kr_achd_solver_output_acceleration_energy[i] += kr_vel_twist_ang_z_pid_controller_signal;
      }
    } 
    for (size_t i = 0; i < sizeof(kr_elbow_base_base_distance_z_embed_map_vector)/sizeof(kr_elbow_base_base_distance_z_embed_map_vector[0]); i++)
    {
      if (kr_elbow_base_base_distance_z_embed_map_vector[i] != 0.0)
      {
        kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench[i] += kr_elbow_base_base_distance_z_impedance_controller_signal;
      }
    } 

    // solvers
    // achd_solver
    double kr_achd_solver_beta[6]{};
    for (size_t i = 0; i < 6; i++)
    {
      kr_achd_solver_beta[i] = -kr_achd_solver_root_acceleration[i];
    }
    add(pid_kr_vel_lin_x_twist_embed_map_kr_achd_solver_output_acceleration_energy, kr_achd_solver_beta, kr_achd_solver_beta, 6);
    add(pid_kr_vel_lin_y_twist_embed_map_kr_achd_solver_output_acceleration_energy, kr_achd_solver_beta, kr_achd_solver_beta, 6);
    add(pid_kr_vel_lin_z_twist_embed_map_kr_achd_solver_output_acceleration_energy, kr_achd_solver_beta, kr_achd_solver_beta, 6);
    add(pid_kr_vel_ang_x_twist_embed_map_kr_achd_solver_output_acceleration_energy, kr_achd_solver_beta, kr_achd_solver_beta, 6);
    add(pid_kr_vel_ang_y_twist_embed_map_kr_achd_solver_output_acceleration_energy, kr_achd_solver_beta, kr_achd_solver_beta, 6);
    add(pid_kr_vel_ang_z_twist_embed_map_kr_achd_solver_output_acceleration_energy, kr_achd_solver_beta, kr_achd_solver_beta, 6);
    achd_solver(&robot, kinova_right_base_link, kinova_right_bracelet_link, kr_achd_solver_nc, kr_achd_solver_root_acceleration, kr_achd_solver_alpha, kr_achd_solver_beta, kr_achd_solver_feed_forward_torques, kr_achd_solver_predicted_accelerations, kr_achd_solver_output_torques);
     
    // achd_solver_fext
    double *kr_achd_solver_fext_ext_wrenches[7];
    for (size_t i = 0; i < 7; i++)
    {
      kr_achd_solver_fext_ext_wrenches[i] = new double[6] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    int link_id = -1;
    getLinkId(&robot, kinova_right_half_arm_2_link, link_id);
    kr_achd_solver_fext_ext_wrenches[link_id] = kr_elbow_base_base_distance_z_embed_map_kr_achd_solver_fext_output_external_wrench; 
    achd_solver_fext(&robot, kr_achd_solver_fext_ext_wrenches, kr_achd_solver_fext_output_torques);
     

    // Command the torques to the robots
    double kinova_right_cmd_tau[7]{};
    add(kr_achd_solver_output_torques, kinova_right_cmd_tau, kinova_right_cmd_tau, 7);
    add(kr_achd_solver_fext_output_torques, kinova_right_cmd_tau, kinova_right_cmd_tau, 7);
    KDL::JntArray kinova_right_cmd_tau_kdl(7);
    cap_and_convert_torques(kinova_right_cmd_tau, 7, kinova_right_cmd_tau_kdl);
    if (!kinova_right_torque_control_mode_set) {
      robot.kinova_right->mediator->set_control_mode(2);
      kinova_right_torque_control_mode_set = true;
    }

    set_manipulator_torques(&robot, kinova_right_base_link, &kinova_right_cmd_tau_kdl);

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