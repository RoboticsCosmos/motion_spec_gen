#include <array>
#include <string>
#include <filesystem>
#include <iostream>
#include <controllers/pid_controller.hpp>
#include <kdl_utils/utils.hpp>
#include <kdl_utils/math_utils.hpp>
#include <kdl_utils/solver_utils.hpp>


int main()
{

  Kinova rob;
  double init_q[7] = {0.0, 0.26, 0.0, 2.26, 0.0, -0.95, -1.57};
  initialize_robot_state(7, 7, init_q, &rob);

  double dt = 0.001;

  // initialize variables
  double pid_move_arm_down_vel_ang_y_twist_embed_map_vector[6] = { 0.0,0.0,0.0,0.0,1.0,0.0 }; 
  double move_arm_down_vel_twist_lin_x_pid_controller_kp = 20.0; 
  double move_arm_down_vel_twist_ang_x_pid_controller_kp = 20.0; 
  double vel_bl_wrt_base_vector_ang_z[6] = { 0,0,0,0,0,1 }; 
  double vel_bl_wrt_base_vector_ang_y[6] = { 0,0,0,0,1,0 }; 
  double move_arm_down_vel_twist_monitor_pre_ang_z_reference_value = 0.0; 
  double move_arm_down_vel_twist_ang_y_pid_controller_kd = 0.0; 
  double vel_bl_wrt_base_vector_ang_x[6] = { 0,0,0,1,0,0 }; 
  double move_arm_down_vel_twist_lin_x_pid_controller_ki = 0.9; 
  int achd_solver_ns = 8; 
  double move_arm_down_vel_twist_lin_y_pid_controller_error_sum; 
  double move_arm_down_vel_twist_ang_z_pid_controller_error_sum; 
  double move_arm_down_vel_twist_lin_y_pid_controller_kd = 0.0; 
  double move_arm_down_vel_twist_ang_y_pid_controller_ki = 0.9; 
  double move_arm_down_vel_twist_lin_y_pid_controller_time_step = 1; 
  double move_arm_down_vel_twist_lin_x_pid_controller_kd = 0.0; 
  double move_arm_down_vel_twist_lin_y_pid_controller_ki = 0.9; 
  int achd_solver_nc = 6; 
  double achd_solver_root_acceleration[6] = { 0.0,0.0,9.81,0.0,0.0,0.0 }; 
  double move_arm_down_vel_twist_lin_y_pid_controller_kp = 20.0; 
  std::string base_link = "base_link"; 
  double move_arm_down_vel_twist_monitor_pre_lin_z_reference_value = 0.0; 
  int achd_solver_nj = 7; 
  double *achd_solver_alpha[6] = { new double[6]{ 1.0,0.0,0.0,0.0,0.0,0.0 }, new double[6]{ 0.0,1.0,0.0,0.0,0.0,0.0 }, new double[6]{ 0.0,0.0,1.0,0.0,0.0,0.0 }, new double[6]{ 0.0,0.0,0.0,1.0,0.0,0.0 }, new double[6]{ 0.0,0.0,0.0,0.0,1.0,0.0 }, new double[6]{ 0.0,0.0,0.0,0.0,0.0,1.0 } }; 
  double move_arm_down_vel_twist_lin_z_pid_controller_prev_error; 
  double move_arm_down_vel_twist_ang_x_pid_controller_signal; 
  double move_arm_down_vel_twist_lin_z_pid_controller_signal; 
  double move_arm_down_vel_twist_monitor_pre_ang_y_reference_value = 0.0; 
  double move_arm_down_vel_twist_ang_x_pid_controller_reference_value = 0.0; 
  double move_arm_down_vel_twist_lin_x_pid_controller_reference_value = 0.0; 
  double move_arm_down_vel_twist_monitor_pre_lin_y_reference_value = 0.0; 
  double move_arm_down_vel_twist_lin_y_pid_controller_prev_error; 
  double move_arm_down_vel_twist_ang_x_pid_controller_kd = 0.0; 
  double move_arm_down_vel_twist_monitor_post_lin_z_reference_value = 0.0; 
  bool move_arm_down_vel_twist_monitor_post_lin_z_flag; 
  bool move_arm_down_vel_twist_monitor_pre_lin_x_flag; 
  double move_arm_down_vel_twist_ang_y_pid_controller_kp = 20.0; 
  double move_arm_down_vel_twist_ang_z_pid_controller_reference_value = 0.0; 
  double achd_solver_output_torques[7]{}; 
  double achd_solver_output_acceleration_energy[6]{}; 
  double move_arm_down_vel_twist_ang_z_pid_controller_prev_error; 
  double pid_move_arm_down_vel_lin_x_twist_embed_map_vector[6] = { 1.0,0.0,0.0,0.0,0.0,0.0 }; 
  double move_arm_down_vel_twist_ang_x_pid_controller_ki = 0.9; 
  double pid_move_arm_down_vel_ang_x_twist_embed_map_vector[6] = { 0.0,0.0,0.0,1.0,0.0,0.0 }; 
  double move_arm_down_vel_twist_lin_y_pid_controller_reference_value = 0.0; 
  double move_arm_down_vel_twist_lin_x_pid_controller_prev_error; 
  double move_arm_down_vel_twist_ang_z_pid_controller_signal; 
  double move_arm_down_vel_twist_lin_z_pid_controller_reference_value = -0.05; 
  double move_arm_down_vel_twist_ang_y_pid_controller_reference_value = 0.0; 
  double move_arm_down_vel_twist_monitor_post_lin_x_reference_value = 0.0; 
  bool move_arm_down_vel_twist_monitor_pre_lin_y_flag; 
  double move_arm_down_vel_twist_monitor_pre_lin_x_reference_value = 0.0; 
  double move_arm_down_vel_twist_ang_y_pid_controller_prev_error; 
  bool move_arm_down_vel_twist_monitor_pre_ang_y_flag; 
  double move_arm_down_vel_twist_lin_x_pid_controller_signal; 
  double move_arm_down_vel_twist_lin_z_pid_controller_error_sum; 
  double move_arm_down_vel_twist_ang_y_pid_controller_signal; 
  std::string bracelet_link = "bracelet_link"; 
  double move_arm_down_vel_twist_lin_y_pid_controller_signal; 
  bool move_arm_down_vel_twist_monitor_post_lin_y_flag; 
  double pid_move_arm_down_vel_lin_y_twist_embed_map_vector[6] = { 0.0,1.0,0.0,0.0,0.0,0.0 }; 
  double move_arm_down_vel_twist_lin_z_pid_controller_time_step = 1; 
  double move_arm_down_vel_twist_ang_z_pid_controller_time_step = 1; 
  bool move_arm_down_vel_twist_monitor_pre_ang_x_flag; 
  double move_arm_down_vel_twist_ang_z_pid_controller_kp = 20.0; 
  double achd_solver_predicted_accelerations[7]{}; 
  double vel_bl_wrt_base_vector_lin_y[6] = { 0,1,0,0,0,0 }; 
  double move_arm_down_vel_twist_lin_z_pid_controller_kd = 0.0; 
  double vel_bl_wrt_base_vector_lin_z[6] = { 0,0,1,0,0,0 }; 
  bool move_arm_down_vel_twist_monitor_pre_lin_z_flag; 
  double pid_move_arm_down_vel_ang_z_twist_embed_map_vector[6] = { 0.0,0.0,0.0,0.0,0.0,1.0 }; 
  double achd_solver_feed_forward_torques[7]{}; 
  double move_arm_down_vel_twist_ang_z_pid_controller_kd = 0.0; 
  double move_arm_down_vel_twist_ang_x_pid_controller_prev_error; 
  double move_arm_down_vel_twist_ang_z_pid_controller_ki = 0.9; 
  double vel_bl_wrt_base_vector_lin_x[6] = { 1,0,0,0,0,0 }; 
  double move_arm_down_vel_twist_lin_x_pid_controller_time_step = 1; 
  double move_arm_down_vel_twist_ang_y_pid_controller_time_step = 1; 
  double move_arm_down_vel_twist_monitor_post_lin_y_reference_value = 0.0; 
  bool move_arm_down_vel_twist_monitor_post_lin_x_flag; 
  double move_arm_down_vel_twist_ang_x_pid_controller_time_step = 1; 
  double pid_move_arm_down_vel_lin_z_twist_embed_map_vector[6] = { 0.0,0.0,1.0,0.0,0.0,0.0 }; 
  double of_vel_qname_lin_x; 
  double of_vel_qname_lin_y; 
  double of_vel_qname_lin_z; 
  double of_vel_qname_ang_y; 
  double move_arm_down_vel_twist_lin_z_pid_controller_kp = 20.0; 
  double of_vel_qname_ang_x; 
  double move_arm_down_vel_twist_ang_y_pid_controller_error_sum; 
  bool move_arm_down_vel_twist_monitor_pre_ang_z_flag; 
  double of_vel_qname_ang_z; 
  double move_arm_down_vel_twist_lin_z_pid_controller_ki = 0.9; 
  double move_arm_down_vel_twist_monitor_pre_ang_x_reference_value = 0.0; 
  double move_arm_down_vel_twist_lin_x_pid_controller_error_sum; 
  double move_arm_down_vel_twist_ang_x_pid_controller_error_sum; 

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "gen3_robotiq_2f_85.urdf").string();

  // set the base and tool links
  std::string kdl_base_link = "base_link";
  std::string kdl_tool_link = "bracelet_link";

  // initialize the chain
  KDL::Chain robot_chain;
  initialize_robot_chain(robot_urdf, kdl_base_link, kdl_tool_link, robot_chain);

  // pre monitors
  // measure the variable
  computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_lin_z, &rob, &robot_chain, of_vel_qname_lin_z);

  // compare
  compareEqual(of_vel_qname_lin_z, move_arm_down_vel_twist_monitor_pre_lin_z_reference_value, move_arm_down_vel_twist_monitor_pre_lin_z_flag);;

  // check if the flag is set
  if (move_arm_down_vel_twist_monitor_pre_lin_z_flag)
  {
    return -1;
  } 
  // measure the variable
  computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_ang_x, &rob, &robot_chain, of_vel_qname_ang_x);

  // compare
  compareEqual(of_vel_qname_ang_x, move_arm_down_vel_twist_monitor_pre_ang_x_reference_value, move_arm_down_vel_twist_monitor_pre_ang_x_flag);;

  // check if the flag is set
  if (move_arm_down_vel_twist_monitor_pre_ang_x_flag)
  {
    return -1;
  } 
  // measure the variable
  computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_lin_y, &rob, &robot_chain, of_vel_qname_lin_y);

  // compare
  compareEqual(of_vel_qname_lin_y, move_arm_down_vel_twist_monitor_pre_lin_y_reference_value, move_arm_down_vel_twist_monitor_pre_lin_y_flag);;

  // check if the flag is set
  if (move_arm_down_vel_twist_monitor_pre_lin_y_flag)
  {
    return -1;
  } 
  // measure the variable
  computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_ang_y, &rob, &robot_chain, of_vel_qname_ang_y);

  // compare
  compareEqual(of_vel_qname_ang_y, move_arm_down_vel_twist_monitor_pre_ang_y_reference_value, move_arm_down_vel_twist_monitor_pre_ang_y_flag);;

  // check if the flag is set
  if (move_arm_down_vel_twist_monitor_pre_ang_y_flag)
  {
    return -1;
  } 
  // measure the variable
  computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_ang_z, &rob, &robot_chain, of_vel_qname_ang_z);

  // compare
  compareEqual(of_vel_qname_ang_z, move_arm_down_vel_twist_monitor_pre_ang_z_reference_value, move_arm_down_vel_twist_monitor_pre_ang_z_flag);;

  // check if the flag is set
  if (move_arm_down_vel_twist_monitor_pre_ang_z_flag)
  {
    return -1;
  } 
  // measure the variable
  computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_lin_x, &rob, &robot_chain, of_vel_qname_lin_x);

  // compare
  compareEqual(of_vel_qname_lin_x, move_arm_down_vel_twist_monitor_pre_lin_x_reference_value, move_arm_down_vel_twist_monitor_pre_lin_x_flag);;

  // check if the flag is set
  if (move_arm_down_vel_twist_monitor_pre_lin_x_flag)
  {
    return -1;
  } 

  int count = 0;

  while (true && count < 300)
  {
    std::cout << std::endl;
    std ::cout << "---------------------->count: " << count << std::endl;
    count++;

    double command_accelerations[7]{};

    add(achd_solver_predicted_accelerations, command_accelerations, command_accelerations, 7);
    updateQandQdot(command_accelerations, dt, &rob);

    // controllers
    // measure the variable
    computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_ang_y, &rob, &robot_chain, of_vel_qname_ang_y);

    double error;
    computeError(of_vel_qname_ang_y, move_arm_down_vel_twist_ang_y_pid_controller_reference_value, error);
    pidController(error, move_arm_down_vel_twist_ang_y_pid_controller_kp, move_arm_down_vel_twist_ang_y_pid_controller_ki, move_arm_down_vel_twist_ang_y_pid_controller_kd, move_arm_down_vel_twist_ang_y_pid_controller_time_step, move_arm_down_vel_twist_ang_y_pid_controller_error_sum, move_arm_down_vel_twist_ang_y_pid_controller_prev_error, move_arm_down_vel_twist_ang_y_pid_controller_signal);
    // measure the variable
    computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_ang_x, &rob, &robot_chain, of_vel_qname_ang_x);

    double error;
    computeError(of_vel_qname_ang_x, move_arm_down_vel_twist_ang_x_pid_controller_reference_value, error);
    pidController(error, move_arm_down_vel_twist_ang_x_pid_controller_kp, move_arm_down_vel_twist_ang_x_pid_controller_ki, move_arm_down_vel_twist_ang_x_pid_controller_kd, move_arm_down_vel_twist_ang_x_pid_controller_time_step, move_arm_down_vel_twist_ang_x_pid_controller_error_sum, move_arm_down_vel_twist_ang_x_pid_controller_prev_error, move_arm_down_vel_twist_ang_x_pid_controller_signal);
    // measure the variable
    computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_lin_x, &rob, &robot_chain, of_vel_qname_lin_x);

    double error;
    computeError(of_vel_qname_lin_x, move_arm_down_vel_twist_lin_x_pid_controller_reference_value, error);
    pidController(error, move_arm_down_vel_twist_lin_x_pid_controller_kp, move_arm_down_vel_twist_lin_x_pid_controller_ki, move_arm_down_vel_twist_lin_x_pid_controller_kd, move_arm_down_vel_twist_lin_x_pid_controller_time_step, move_arm_down_vel_twist_lin_x_pid_controller_error_sum, move_arm_down_vel_twist_lin_x_pid_controller_prev_error, move_arm_down_vel_twist_lin_x_pid_controller_signal);
    // measure the variable
    computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_ang_z, &rob, &robot_chain, of_vel_qname_ang_z);

    double error;
    computeError(of_vel_qname_ang_z, move_arm_down_vel_twist_ang_z_pid_controller_reference_value, error);
    pidController(error, move_arm_down_vel_twist_ang_z_pid_controller_kp, move_arm_down_vel_twist_ang_z_pid_controller_ki, move_arm_down_vel_twist_ang_z_pid_controller_kd, move_arm_down_vel_twist_ang_z_pid_controller_time_step, move_arm_down_vel_twist_ang_z_pid_controller_error_sum, move_arm_down_vel_twist_ang_z_pid_controller_prev_error, move_arm_down_vel_twist_ang_z_pid_controller_signal);
    // measure the variable
    computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_lin_y, &rob, &robot_chain, of_vel_qname_lin_y);

    double error;
    computeError(of_vel_qname_lin_y, move_arm_down_vel_twist_lin_y_pid_controller_reference_value, error);
    pidController(error, move_arm_down_vel_twist_lin_y_pid_controller_kp, move_arm_down_vel_twist_lin_y_pid_controller_ki, move_arm_down_vel_twist_lin_y_pid_controller_kd, move_arm_down_vel_twist_lin_y_pid_controller_time_step, move_arm_down_vel_twist_lin_y_pid_controller_error_sum, move_arm_down_vel_twist_lin_y_pid_controller_prev_error, move_arm_down_vel_twist_lin_y_pid_controller_signal);
    // measure the variable
    computeForwardVelocityKinematics(bracelet_link, base_link, base_link, vel_bl_wrt_base_vector_lin_z, &rob, &robot_chain, of_vel_qname_lin_z);

    double error;
    computeError(of_vel_qname_lin_z, move_arm_down_vel_twist_lin_z_pid_controller_reference_value, error);
    pidController(error, move_arm_down_vel_twist_lin_z_pid_controller_kp, move_arm_down_vel_twist_lin_z_pid_controller_ki, move_arm_down_vel_twist_lin_z_pid_controller_kd, move_arm_down_vel_twist_lin_z_pid_controller_time_step, move_arm_down_vel_twist_lin_z_pid_controller_error_sum, move_arm_down_vel_twist_lin_z_pid_controller_prev_error, move_arm_down_vel_twist_lin_z_pid_controller_signal);

    // embed maps
    for (size_t i = 0; i < sizeof(pid_move_arm_down_vel_lin_x_twist_embed_map_vector)/sizeof(pid_move_arm_down_vel_lin_x_twist_embed_map_vector[0]); i++)
    {
      if (pid_move_arm_down_vel_lin_x_twist_embed_map_vector[i] != 0.0)
      {
        achd_solver_output_acceleration_energy[i] += move_arm_down_vel_twist_lin_x_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(pid_move_arm_down_vel_lin_y_twist_embed_map_vector)/sizeof(pid_move_arm_down_vel_lin_y_twist_embed_map_vector[0]); i++)
    {
      if (pid_move_arm_down_vel_lin_y_twist_embed_map_vector[i] != 0.0)
      {
        achd_solver_output_acceleration_energy[i] += move_arm_down_vel_twist_lin_y_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(pid_move_arm_down_vel_lin_z_twist_embed_map_vector)/sizeof(pid_move_arm_down_vel_lin_z_twist_embed_map_vector[0]); i++)
    {
      if (pid_move_arm_down_vel_lin_z_twist_embed_map_vector[i] != 0.0)
      {
        achd_solver_output_acceleration_energy[i] += move_arm_down_vel_twist_lin_z_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(pid_move_arm_down_vel_ang_x_twist_embed_map_vector)/sizeof(pid_move_arm_down_vel_ang_x_twist_embed_map_vector[0]); i++)
    {
      if (pid_move_arm_down_vel_ang_x_twist_embed_map_vector[i] != 0.0)
      {
        achd_solver_output_acceleration_energy[i] += move_arm_down_vel_twist_ang_x_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(pid_move_arm_down_vel_ang_y_twist_embed_map_vector)/sizeof(pid_move_arm_down_vel_ang_y_twist_embed_map_vector[0]); i++)
    {
      if (pid_move_arm_down_vel_ang_y_twist_embed_map_vector[i] != 0.0)
      {
        achd_solver_output_acceleration_energy[i] += move_arm_down_vel_twist_ang_y_pid_controller_signal;
      }
    }
    for (size_t i = 0; i < sizeof(pid_move_arm_down_vel_ang_z_twist_embed_map_vector)/sizeof(pid_move_arm_down_vel_ang_z_twist_embed_map_vector[0]); i++)
    {
      if (pid_move_arm_down_vel_ang_z_twist_embed_map_vector[i] != 0.0)
      {
        achd_solver_output_acceleration_energy[i] += move_arm_down_vel_twist_ang_z_pid_controller_signal;
      }
    } 

    // solvers
    // solver beta
    double beta[6]{};
    for (size_t i = 0; i < 6; i++)
    {
      beta[i] = achd_solver_root_acceleration[i];
    }

    // solver ext_wrench
    double *ext_wrench[7];

    for (size_t i = 0; i < 7; i++)
    {
      ext_wrench[i] = new double[6] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    add(achd_solver_output_acceleration_energy, beta, beta, 6);

    achd_solver(&rob, &robot_chain, achd_solver_nc, achd_solver_root_acceleration, achd_solver_alpha, beta, ext_wrench, achd_solver_feed_forward_torques, achd_solver_predicted_accelerations, achd_solver_output_torques); 

    

    // post monitors
    
  }

  return 0;
}
