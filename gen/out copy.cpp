#include <array>
#include <controllers/pid_controller.hpp>
#include <filesystem>
#include <iostream>
#include <kdl_utils/math_utils.hpp>
#include <kdl_utils/solver_utils.hpp>
#include <kdl_utils/utils.hpp>
#include <string>

int main()
{
  Kinova rob;
  double init_q[7] = {0.0, 0.26, 0.0, 2.26, 0.0, -0.95, -1.57};
  initialize_robot_state(7, 7, init_q, &rob);

  double dt = 0.001;

  // initialize variables
  double bracelet_link_vel_twist[6]{};
  int move_arm_down_vel_twist_achd_solver_nc = 6;
  bool move_arm_down_vel_twist_monitor_post_flag;
  int move_arm_down_vel_twist_achd_solver_nj = 7;
  double pid_move_arm_down_vel_twist_controller_signal[6]{};
  int move_arm_down_vel_twist_achd_solver_ns = 8;
  std::string bracelet_link = "bracelet_link";
  double pid_move_arm_down_vel_twist_controller_prev_error[6]{};
  bool move_arm_down_vel_twist_monitor_pre_flag;
  double move_arm_down_vel_twist_monitor_pre_bracelet_link_vel_twist_sp[6] = {0.0, 0.0, 0.0,
                                                                              0.0, 0.0, 0.0};
  double move_arm_down_vel_twist_monitor_pre_threshold_value = 0.0;
  double move_arm_down_vel_twist_monitor_post_threshold_value = 0.001;
  double move_arm_down_vel_twist_achd_solver_feed_forward_torques[7]{};
  double *move_arm_down_vel_twist_achd_solver_alpha[6] = {
      new double[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, new double[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, new double[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
      new double[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0}, new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  double move_arm_down_vel_twist_achd_solver_predicted_accelerations[7]{};
  double pid_move_arm_down_vel_twist_embed_map_vector[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  double move_arm_down_vel_twist_achd_solver_output_torques[7]{};
  double move_arm_down_vel_twist_monitor_post_bracelet_link_vel_twist_sp[6] = {0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0};
  double move_arm_down_vel_twist_achd_solver_root_acceleration[6] = {0.0, 0.0, 9.81,
                                                                     0.0, 0.0, 0.0};
  double pid_move_arm_down_vel_twist_controller_ki = 0.9;
  double pid_move_arm_down_vel_twist_controller_bracelet_link_vel_twist_sp[6] = {0.0, 0.0, -0.05,
                                                                                 0.0, 0.0, 0.0};
  double pid_move_arm_down_vel_twist_controller_kd = 0.0;
  double pid_move_arm_down_vel_twist_controller_time_step = 1;
  double pid_move_arm_down_vel_twist_controller_error_sum[6]{};
  double pid_move_arm_down_vel_twist_controller_output_acceleration_energy[6]{};
  double pid_move_arm_down_vel_twist_controller_threshold_value = 0.0;
  double pid_move_arm_down_vel_twist_controller_kp = 20.0;

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "gen3_robotiq_2f_85.urdf").string();

  // set the base and tool links
  std::string base_link = "base_link";
  std::string tool_link = "bracelet_link";

  // initialize the chain
  KDL::Chain robot_chain;
  initialize_robot_chain(robot_urdf, base_link, tool_link, robot_chain);

  // pre monitors
  // measure the variable
  computeForwardVelocityKinematics(bracelet_link, &rob, &robot_chain, bracelet_link_vel_twist);

  for (size_t i = 0; i < sizeof(bracelet_link_vel_twist) / sizeof(bracelet_link_vel_twist[0]); i++)
  {
    // compare
    compare(bracelet_link_vel_twist[i],
            move_arm_down_vel_twist_monitor_pre_bracelet_link_vel_twist_sp[i],
            move_arm_down_vel_twist_monitor_pre_threshold_value, "lt",
            move_arm_down_vel_twist_monitor_pre_flag);

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

  int count = 0;

  while (true && count < 300)
  {
    std::cout << std::endl;
    std ::cout << "---------------------->count: " << count << std::endl;
    count++;

    double command_accelerations[7]{};

    add(move_arm_down_vel_twist_achd_solver_predicted_accelerations, command_accelerations,
        command_accelerations, 7);
    updateQandQdot(command_accelerations, dt, &rob);

    // controllers
    // measure the variable
    computeForwardVelocityKinematics(bracelet_link, &rob, &robot_chain, bracelet_link_vel_twist);

    // print
    std::cout << std::endl;
    std::cout << "bracelet_link_vel_twist: ";
    for (size_t i = 0; i < sizeof(bracelet_link_vel_twist) / sizeof(bracelet_link_vel_twist[0]);
         i++)
    {
      std::cout << bracelet_link_vel_twist[i] << " ";
    }
    std::cout << std::endl;

    for (size_t i = 0; i < sizeof(bracelet_link_vel_twist) / sizeof(bracelet_link_vel_twist[0]);
         i++)
    {
      if (pid_move_arm_down_vel_twist_embed_map_vector[i] == 1.0)
      {
        double error;
        computeError(bracelet_link_vel_twist[i],
                     pid_move_arm_down_vel_twist_controller_bracelet_link_vel_twist_sp[i],
                     pid_move_arm_down_vel_twist_controller_threshold_value, error);
        pidController(error, pid_move_arm_down_vel_twist_controller_kp,
                      pid_move_arm_down_vel_twist_controller_ki,
                      pid_move_arm_down_vel_twist_controller_kd,
                      pid_move_arm_down_vel_twist_controller_time_step,
                      pid_move_arm_down_vel_twist_controller_error_sum[i],
                      pid_move_arm_down_vel_twist_controller_prev_error[i],
                      pid_move_arm_down_vel_twist_controller_signal[i]);
      }
    }

    // embed maps
    for (size_t i = 0; i < sizeof(pid_move_arm_down_vel_twist_embed_map_vector) /
                               sizeof(pid_move_arm_down_vel_twist_embed_map_vector[0]);
         i++)
    {
      if (pid_move_arm_down_vel_twist_embed_map_vector[i] == 1.0)
      {
        pid_move_arm_down_vel_twist_controller_output_acceleration_energy[i] =
            pid_move_arm_down_vel_twist_controller_signal[i];
      }
    }

    // solvers
    // solver beta
    double beta[6]{};
    for (size_t i = 0; i < 6; i++)
    {
      beta[i] = move_arm_down_vel_twist_achd_solver_root_acceleration[i];
    }

    // solver ext_wrench
    double *ext_wrench[7];

    for (size_t i = 0; i < 7; i++)
    {
      ext_wrench[i] = new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    add(pid_move_arm_down_vel_twist_controller_output_acceleration_energy, beta, beta, 6);

    achd_solver(&rob, &robot_chain, move_arm_down_vel_twist_achd_solver_nc,
                move_arm_down_vel_twist_achd_solver_root_acceleration,
                move_arm_down_vel_twist_achd_solver_alpha, beta, ext_wrench,
                move_arm_down_vel_twist_achd_solver_feed_forward_torques,
                move_arm_down_vel_twist_achd_solver_predicted_accelerations,
                move_arm_down_vel_twist_achd_solver_output_torques);

    // command torques
    double command_torques[7];

    add(move_arm_down_vel_twist_achd_solver_output_torques, command_torques, command_torques, 7);

    // post monitors
    // measure the variable
    computeForwardVelocityKinematics(bracelet_link, &rob, &robot_chain, bracelet_link_vel_twist);

    for (size_t i = 0; i < sizeof(bracelet_link_vel_twist) / sizeof(bracelet_link_vel_twist[0]);
         i++)
    {
      // compare
      compare(bracelet_link_vel_twist[i],
              move_arm_down_vel_twist_monitor_post_bracelet_link_vel_twist_sp[i],
              move_arm_down_vel_twist_monitor_post_threshold_value, "eq",
              move_arm_down_vel_twist_monitor_post_flag);

      // check if the flag is set
      if (move_arm_down_vel_twist_monitor_post_flag)
      {
        // break the loop
        break;
      }
    }

    
  }

  return 0;
}
