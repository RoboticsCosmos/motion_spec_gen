#include <std/array.hpp>
#include <std/string.hpp>
#include <filesystem>
#include <controllers/pid_controller.hpp>
#include <kdl_utils/solver_utils.hpp>
#include <kdl_utils/utils.hpp>

struct Kinova {
  std::array<double, 7> q;
  std::array<double, 7> q_dot;
  std::array<double, 7> q_ddot;
};

int main()
{

  Kinova rob;

  // initialize variables
  std::array<double, 6> bracelet_link_vel_twist; 
  int move_arm_down_vel_twist_achd_solver_nc = 6; 
  bool move_arm_down_vel_twist_monitor_post_flag; 
  int move_arm_down_vel_twist_achd_solver_nj = 7; 
  std::array<double, 6> pid_move_arm_down_vel_twist_controller_signal; 
  int move_arm_down_vel_twist_achd_solver_ns = 8; 
  string bracelet_link = bracelet_link; 
  std::array<double, 6> pid_move_arm_down_vel_twist_controller_prev_error; 
  bool move_arm_down_vel_twist_monitor_pre_flag; 
  std::array<double, 6> move_arm_down_vel_twist_monitor_pre_bracelet_link_vel_twist_sp = { 0.0,0.0,0.0,0.0,0.0,0.0 }; 
  double move_arm_down_vel_twist_monitor_pre_threshold_value = 0.0; 
  double move_arm_down_vel_twist_monitor_post_threshold_value = 0.001; 
  std::array<double, 7> move_arm_down_vel_twist_achd_solver_feed_forward_torques; 
  std::array<std::array<double, 2>, 6> move_arm_down_vel_twist_achd_solver_alpha = { { 1.0,0.0,0.0,0.0,0.0,0.0 }, { 0.0,1.0,0.0,0.0,0.0,0.0 }, { 0.0,0.0,1.0,0.0,0.0,0.0 }, { 0.0,0.0,0.0,1.0,0.0,0.0 }, { 0.0,0.0,0.0,0.0,1.0,0.0 }, { 0.0,0.0,0.0,0.0,0.0,1.0 } }; 
  std::array<double, 7> move_arm_down_vel_twist_achd_solver_predicted_accelerations; 
  std::array<double, 6> pid_move_arm_down_vel_twist_embed_map_vector = { 1.0,1.0,1.0,1.0,1.0,1.0 }; 
  std::array<double, 7> move_arm_down_vel_twist_achd_solver_output_torques; 
  std::array<double, 6> move_arm_down_vel_twist_monitor_post_bracelet_link_vel_twist_sp = { 0.0,0.0,0.0,0.0,0.0,0.0 }; 
  std::array<double, 6> move_arm_down_vel_twist_achd_solver_root_acceleration = { 0.0,0.0,9.81,0.0,0.0,0.0 }; 
  double pid_move_arm_down_vel_twist_controller_ki = 0.9; 
  std::array<double, 6> pid_move_arm_down_vel_twist_controller_bracelet_link_vel_twist_sp = { 0.0,0.0,-0.05,0.0,0.0,0.0 }; 
  double pid_move_arm_down_vel_twist_controller_kd = 0.0; 
  double pid_move_arm_down_vel_twist_controller_time_step = 1; 
  std::array<double, 6> pid_move_arm_down_vel_twist_controller_error_sum; 
  std::array<double, 6> pid_move_arm_down_vel_twist_controller_output_acceleration_energy; 
  double pid_move_arm_down_vel_twist_controller_threshold_value = 0.001; 
  double pid_move_arm_down_vel_twist_controller_kp = 20.0; 

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "gen3_robotiq_2f_85.urdf").string();

  // set the base and tool links
  std::string base_link = "base_link";
  std::string tool_link = "bracelet_link";

  // initialize the chain
  KDL::Chain robot_chain;
  initialize_robot_chain(robot_urdf, base_link, tool_link, robot_chain);

  // pre monitors
  // measure the variable
  computeForwardVelocityKinematics(bracelet_link, bracelet_link_vel_twist, &rob, &robot_chain);

  for (size_t i = 0; i < bracelet_link_vel_twist.size(); i++)
  {
    // compare
    compare(bracelet_link_vel_twist[i], move_arm_down_vel_twist_monitor_pre_bracelet_link_vel_twist_sp[i], move_arm_down_vel_twist_monitor_pre_threshold_value, lt, move_arm_down_vel_twist_monitor_pre_flag);

    // check if the flag is set
    if (move_arm_down_vel_twist_monitor_pre_flag)
    {
      // break the loop
      break;
    }
  }

  // stop the execution if the flag is set
  if (move_arm_down_vel_twist_monitor_pre_flag)
  {
    return -1;
  }
   

  while (true) {

    std::array<double, 7> command_accelerations;

    add(move_arm_down_vel_twist_achd_solver_predicted_accelerations, command_accelerations, command_accelerations);
    updateQandQdot(command_accelerations, &rob);

    // controllers
    // measure the variable
    computeForwardVelocityKinematics(bracelet_link, bracelet_link_vel_twist, &rob, &robot_chain);

    for (size_t i = 0; i < pid_move_arm_down_vel_twist_embed_map_vector.size(); i++)
    {
      if (pid_move_arm_down_vel_twist_embed_map_vector[i] == 1.0)
      {
        double error;
        computeError(bracelet_link_vel_twist[i], pid_move_arm_down_vel_twist_controller_bracelet_link_vel_twist_sp[i], data.threshold_value, error);
        pidController(error, pid_move_arm_down_vel_twist_controller_kp, pid_move_arm_down_vel_twist_controller_ki, pid_move_arm_down_vel_twist_controller_kd, pid_move_arm_down_vel_twist_controller_time_step, pid_move_arm_down_vel_twist_controller_error_sum[i], pid_move_arm_down_vel_twist_controller_prev_error[i], pid_move_arm_down_vel_twist_controller_signal[i]);
      }
    }

    // embed maps
    for (size_t i = 0; i < pid_move_arm_down_vel_twist_embed_map_vector.size(); i++)
    {
      if (pid_move_arm_down_vel_twist_embed_map_vector[i] == 1.0)
      {
        pid_move_arm_down_vel_twist_controller_output_acceleration_energy[i] += pid_move_arm_down_vel_twist_controller_signal[i];
      }
    } 

    // solvers
    // solver beta
    std::array<double, 6> beta;
    // solver ext_wrench
    std::array<std::array<double, 6>, move_arm_down_vel_twist_achd_solver_ns> ext_wrench;

    add(pid_move_arm_down_vel_twist_controller_output_acceleration_energy, beta, beta);

    achd_solver(&rob, &robot_chain, move_arm_down_vel_twist_achd_solver_nc, move_arm_down_vel_twist_achd_solver_root_acceleration, move_arm_down_vel_twist_achd_solver_alpha, beta, ext_wrench, move_arm_down_vel_twist_achd_solver_feed_forward_torques, move_arm_down_vel_twist_achd_solver_predicted_accelerations, move_arm_down_vel_twist_achd_solver_output_torques); 

    // command torques
    std::array<double, 7> command_torques;

    add(move_arm_down_vel_twist_achd_solver_output_torques, command_torques, command_torques);

    // post monitors
    // measure the variable
    computeForwardVelocityKinematics(bracelet_link, bracelet_link_vel_twist, &rob, &robot_chain);

    for (size_t i = 0; i < bracelet_link_vel_twist.size(); i++)
    {
      // compare
      compare(bracelet_link_vel_twist[i], move_arm_down_vel_twist_monitor_post_bracelet_link_vel_twist_sp[i], move_arm_down_vel_twist_monitor_post_threshold_value, eq, move_arm_down_vel_twist_monitor_post_flag);

      // check if the flag is set
      if (move_arm_down_vel_twist_monitor_post_flag)
      {
        // break the loop
        break;
      }
    }

    // break the system loop if the flag is set
    if (move_arm_down_vel_twist_monitor_post_flag)
    {
      // break the loop
      break;
    }
     
  }

  return 0;
}
