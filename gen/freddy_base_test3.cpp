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

#include <unsupported/Eigen/MatrixFunctions>

#include "kdl/treefksolverpos_recursive.hpp"

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

  Freddy robot = {nullptr, nullptr, &freddy_base};

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy_corrected_base.urdf").string();

  char ethernet_interface[100] = "eno1";
  initialize_robot(robot_urdf, ethernet_interface, &robot);

  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(robot_urdf, tree))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    exit(1);
  }

  // pivot joints
  // left_front_wheel_pivot_joint, right_front_wheel_pivot_joint, left_rear_wheel_pivot_joint,
  // right_rear_wheel_pivot_joint

  // get the ids of the pivot joints from the tree
  std::string pivot_joint_names[4] = {
      "left_front_wheel_pivot_joint",
      "left_back_wheel_pivot_joint",
      "right_back_wheel_pivot_joint",
      "right_front_wheel_pivot_joint",
  };

  int pivot_joint_ids[4]{};

  // print joint names
  KDL::SegmentMap sm = robot.tree.getSegments();

  // print segment name and joint name
  int index = 0;
  for (KDL::SegmentMap::const_iterator it = sm.begin(); it != sm.end(); it++)
  {
    // check joint type
    if (it->second.segment.getJoint().getType() == KDL::Joint::None)
    {
      continue;
    }
    // print index and segment name
    // printf("Index: %d, Segment: %s, Joint: %s\n", index, it->second.segment.getName().c_str(),
    //        it->second.segment.getJoint().getName().c_str());
    // update pivot joint ids
    for (size_t i = 0; i < 4; i++)
    {
      if (it->second.segment.getJoint().getName() == pivot_joint_names[i])
      {
        pivot_joint_ids[i] = index;
      }
    }
    index++;
  }

  printf("Pivot joint ids: ");
  for (size_t i = 0; i < 4; i++)
  {
    printf("%d ", pivot_joint_ids[i]);
  }
  printf("\n");

  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s
  double control_loop_timestep = desired_period.count();                               // s
  double *control_loop_dt = &control_loop_timestep;                                    // s

  

  double fd_solver_robile_output_torques[8]{};

  get_robot_data(&robot, *control_loop_dt);

  int count = 0;

  while (true)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (flag)
    {
      printf("Exiting somewhat cleanly...\n");
      free_robot_data(&robot);
      exit(0);
    }

    count++;
    printf("\n");
    // printf("count: %d\n", count);

    update_base_state(robot.mobile_base->mediator->kelo_base_config,
                      robot.mobile_base->mediator->ethercat_config);
    get_robot_data(&robot, *control_loop_dt);
    std::cout << std::endl;

    // solver
    double platform_force[3] = {0.0, 0.0, 10.0};  // [N], [N], [Nm]

    std::cout << "platform force: ";
    print_array(platform_force, 3);

    std::cout << "pivolt angles: ";
    for (size_t i = 0; i < 4; i++)
    {
      std::cout << RAD2DEG(robot.mobile_base->state->pivot_angles[i]) << " ";
    }
    std::cout << std::endl;

    // compute the fk of pivot links
    KDL::TreeFkSolverPos_recursive fk_solver = KDL::TreeFkSolverPos_recursive(tree);

    KDL::JntArray q = KDL::JntArray(tree.getNrOfJoints());
    // set the pivot angles based on the pivot joint ids
    for (size_t i = 0; i < 4; i++)
    {
      q(pivot_joint_ids[i] - 1) = robot.mobile_base->state->pivot_angles[i];
    }

    KDL::Frame frame1;
    fk_solver.JntToCart(q, frame1, "left_front_wheel_pivot_link");

    KDL::Frame frame2;
    fk_solver.JntToCart(q, frame2, "left_back_wheel_pivot_link");

    KDL::Frame frame3;
    fk_solver.JntToCart(q, frame3, "right_back_wheel_pivot_link");

    KDL::Frame frame4;
    fk_solver.JntToCart(q, frame4, "right_front_wheel_pivot_link");

    // eigen vec of linear platform force
    Eigen::Vector2d lin_platform_force;
    lin_platform_force(0) = platform_force[0];
    lin_platform_force(1) = platform_force[1];

    // normalize the linear platform force
    lin_platform_force.normalize();

    // 2x2 matrix
    Eigen::Matrix2d lin_ctrl_matrix;
    // x-axis into the direction of the platform force
    // complement y-axis orthogonal to x-axis
    lin_ctrl_matrix << lin_platform_force(0), -lin_platform_force(1),
                      lin_platform_force(1), lin_platform_force(0);

    // wheel 1
    KDL::Vector attachment_vector_w1 = frame1.p;
    attachment_vector_w1.Normalize();

    Eigen::Matrix2d w1_ang_ctrl_matrix;
    w1_ang_ctrl_matrix << attachment_vector_w1(1),  attachment_vector_w1(0),
                          -attachment_vector_w1(0), attachment_vector_w1(1);

    Eigen::Matrix2d w1_rb_cori_matrix;
    w1_rb_cori_matrix << cos(robot.mobile_base->state->pivot_angles[0]), -sin(robot.mobile_base->state->pivot_angles[0]),
                         sin(robot.mobile_base->state->pivot_angles[0]),  cos(robot.mobile_base->state->pivot_angles[0]);

    Eigen::Matrix2d w1_lin_diff_matrix = w1_rb_cori_matrix.transpose() * lin_ctrl_matrix;
    Eigen::Matrix2d w1_ang_diff_matrix = w1_rb_cori_matrix.transpose() * w1_ang_ctrl_matrix;

    double w1_lin_log_vee_omega = atan2(w1_lin_diff_matrix(1, 0), w1_lin_diff_matrix(0, 0));
    double w1_ang_log_vee_omega = atan2(w1_ang_diff_matrix(1, 0), w1_ang_diff_matrix(0, 0));

    // wheel 2
    KDL::Vector attachment_vector_w2 = frame2.p;
    attachment_vector_w2.Normalize();

    Eigen::Matrix2d w2_ang_ctrl_matrix;
    w2_ang_ctrl_matrix << attachment_vector_w2(1),  attachment_vector_w2(0),
                          -attachment_vector_w2(0), attachment_vector_w2(1);
    
    Eigen::Matrix2d w2_rb_cori_matrix;
    w2_rb_cori_matrix << cos(robot.mobile_base->state->pivot_angles[1]), -sin(robot.mobile_base->state->pivot_angles[1]),
                         sin(robot.mobile_base->state->pivot_angles[1]),  cos(robot.mobile_base->state->pivot_angles[1]);

    Eigen::Matrix2d w2_lin_diff_matrix = w2_rb_cori_matrix.transpose() * lin_ctrl_matrix;
    Eigen::Matrix2d w2_ang_diff_matrix = w2_rb_cori_matrix.transpose() * w2_ang_ctrl_matrix;

    double w2_lin_log_vee_omega = atan2(w2_lin_diff_matrix(0, 1), w2_lin_diff_matrix(0, 0));
    double w2_ang_log_vee_omega = atan2(w2_ang_diff_matrix(0, 1), w2_ang_diff_matrix(0, 0));

    // wheel 3
    KDL::Vector attachment_vector_w3 = frame3.p;
    attachment_vector_w3.Normalize();

    Eigen::Matrix2d w3_ang_ctrl_matrix;
    w3_ang_ctrl_matrix << attachment_vector_w3(1),  attachment_vector_w3(0),
                          -attachment_vector_w3(0), attachment_vector_w3(1);

    // 2x2 matrix
    Eigen::Matrix2d w3_rb_cori_matrix;
    w3_rb_cori_matrix << cos(robot.mobile_base->state->pivot_angles[2]), -sin(robot.mobile_base->state->pivot_angles[2]),
                         sin(robot.mobile_base->state->pivot_angles[2]),  cos(robot.mobile_base->state->pivot_angles[2]);


    // compute difference between matrices
    Eigen::Matrix2d w3_lin_diff_matrix = w3_rb_cori_matrix.transpose() * lin_ctrl_matrix;
    Eigen::Matrix2d w3_ang_diff_matrix = w3_rb_cori_matrix.transpose() * w3_ang_ctrl_matrix;

    double w3_lin_log_vee_omega = atan2(w3_lin_diff_matrix(1, 0), w3_lin_diff_matrix(0, 0));
    double w3_ang_log_vee_omega = atan2(w3_ang_diff_matrix(1, 0), w3_ang_diff_matrix(0, 0));

    // wheel 4
    KDL::Vector attachment_vector_w4 = frame4.p;
    attachment_vector_w4.Normalize();

    Eigen::Matrix2d w4_ang_ctrl_matrix;
    w4_ang_ctrl_matrix << -attachment_vector_w4(1),  attachment_vector_w4(0),
                          attachment_vector_w4(0), attachment_vector_w4(1);

    Eigen::Matrix2d w4_rb_cori_matrix;
    w4_rb_cori_matrix << cos(robot.mobile_base->state->pivot_angles[3]), -sin(robot.mobile_base->state->pivot_angles[3]),
                         sin(robot.mobile_base->state->pivot_angles[3]),  cos(robot.mobile_base->state->pivot_angles[3]);

    Eigen::Matrix2d w4_lin_diff_matrix = w4_rb_cori_matrix.transpose() * lin_ctrl_matrix;
    Eigen::Matrix2d w4_ang_diff_matrix = w4_rb_cori_matrix.transpose() * w4_ang_ctrl_matrix;

    double w4_lin_log_vee_omega = atan2(w4_lin_diff_matrix(1, 0), w4_lin_diff_matrix(0, 0));
    double w4_ang_log_vee_omega = atan2(w4_ang_diff_matrix(1, 0), w4_ang_diff_matrix(0, 0));

    std::cout << "w1 omega: " << w1_lin_log_vee_omega << " " << w1_ang_log_vee_omega << std::endl;
    std::cout << "w2 omega: " << w2_lin_log_vee_omega << " " << w2_ang_log_vee_omega << std::endl;
    std::cout << "w3 omega: " << w3_lin_log_vee_omega << " " << w3_ang_log_vee_omega << std::endl;
    std::cout << "w4 omega: " << w4_lin_log_vee_omega << " " << w4_ang_log_vee_omega << std::endl;

    usleep(90000);

    // std::cout << "wheels tau_c2: ";
    // print_array(tau_wheel_c2, 8);

    std::cout << "w1 tau: " << lin_platform_force.norm()*w1_lin_log_vee_omega + platform_force[2]*w1_ang_log_vee_omega << std::endl;
    std::cout << "w2 tau: " << lin_platform_force.norm()*w2_lin_log_vee_omega + platform_force[2]*w2_ang_log_vee_omega << std::endl;
    std::cout << "w3 tau: " << lin_platform_force.norm()*w3_lin_log_vee_omega + platform_force[2]*w3_ang_log_vee_omega << std::endl;
    std::cout << "w4 tau: " << lin_platform_force.norm()*w4_lin_log_vee_omega + platform_force[2]*w4_ang_log_vee_omega << std::endl;

    // set torques
    // set_mobile_base_torques(&robot, tau_wheel_c2);

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