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

void method(double attachment_vector[2], double platform_force[3], double pivot_direction[2],
            double &lin_diff, double &ang_diff)
{
  Eigen::Vector2d lin_platform_force;
  lin_platform_force << platform_force[0], platform_force[1];
  lin_platform_force.normalize();

  Eigen::Vector2d attachment;
  attachment << attachment_vector[0], attachment_vector[1];

  Eigen::Vector2d pivot;
  pivot << pivot_direction[0], pivot_direction[1];

  // compute tangent vector of the attachment vector in cw or ccw based on platform force[2] -
  // moment if the moment is positive, the tangents are in ccw, otherwise in cw
  Eigen::Rotation2Dd rot_ccw(M_PI / 2);
  Eigen::Rotation2Dd rot_cw(-M_PI / 2);

  Eigen::Vector2d tangent = platform_force[2] > 0 ? rot_ccw * attachment : rot_cw * attachment;

  // get the angular difference between the pivot direction and the tangent
  ang_diff = atan2(pivot.x() * tangent.y() - pivot.y() * tangent.x(),
                   pivot.x() * tangent.x() + pivot.y() * tangent.y());

  // get the angular difference between the pivot direction and platform linear force
  lin_diff = atan2(pivot.x() * lin_platform_force.y() - pivot.y() * lin_platform_force.x(),
                   pivot.x() * lin_platform_force.x() + pivot.y() * lin_platform_force.y());
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

  // initialize variables
  double caster_offsets[4]{};
  for (size_t i = 0; i < 4; i++)
  {
    caster_offsets[i] = robot.mobile_base->mediator->kelo_base_config->castor_offset;
  }

  double wheel_distances[4]{};
  for (size_t i = 0; i < 4; i++)
  {
    wheel_distances[i] = robot.mobile_base->mediator->kelo_base_config->half_wheel_distance * 2;
  }

  double wheel_diameters[8]{};
  for (size_t i = 0; i < 8; i++)
  {
    wheel_diameters[i] = robot.mobile_base->mediator->kelo_base_config->radius * 2;
  }

  double w_platform[3 * 3] = {
      // [1/N^2], [1/(N Nm)], [1/(Nm)^2]
      1.0,    0.0,    0.0,  // xx, xy, xm
      0.0,    1.0,    0.0,  // yx, yy, ym
      0.0,    0.0,    1.0   // mx, my, mm
  };

  double w_drive[4 * 4] = {
      // [1/N^2]
      1.0, 0.0, 0.0, 1.0,  // fl-xx, fl-xy, fl-yx, fl-yy
      1.0, 0.0, 0.0, 1.0,  // rl-xx, rl-xy, rl-yx, rl-yy
      1.0, 0.0, 0.0, 1.0,  // rr-xx, rr-xy, rr-yx, rr-yy
      1.0, 0.0, 0.0, 1.0   // fr-xx, fr-xy, fr-yx, fr-yy
  };

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
    double platform_force[3] = {400.0, 0.0, -50.0};  // [N], [N], [Nm]

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

    // my approach
    double attachment_vec_w1[2] = {frame1.p.x(), frame1.p.y()};
    double attachment_vec_w2[2] = {frame2.p.x(), frame2.p.y()};
    double attachment_vec_w3[2] = {frame3.p.x(), frame3.p.y()};
    double attachment_vec_w4[2] = {frame4.p.x(), frame4.p.y()};

    double pivot_direction_w1[2] = {frame1.M.UnitX().x(), frame1.M.UnitX().y()};
    double pivot_direction_w2[2] = {frame2.M.UnitX().x(), frame2.M.UnitX().y()};
    double pivot_direction_w3[2] = {frame3.M.UnitX().x(), frame3.M.UnitX().y()};
    double pivot_direction_w4[2] = {frame4.M.UnitX().x(), frame4.M.UnitX().y()};

    double lin_diff_w1, ang_diff_w1;
    double lin_diff_w2, ang_diff_w2;
    double lin_diff_w3, ang_diff_w3;
    double lin_diff_w4, ang_diff_w4;

    method(attachment_vec_w1, platform_force, pivot_direction_w1, lin_diff_w1, ang_diff_w1);
    method(attachment_vec_w2, platform_force, pivot_direction_w2, lin_diff_w2, ang_diff_w2);
    method(attachment_vec_w3, platform_force, pivot_direction_w3, lin_diff_w3, ang_diff_w3);
    method(attachment_vec_w4, platform_force, pivot_direction_w4, lin_diff_w4, ang_diff_w4);

    std::cout << "w1: " << lin_diff_w1 << " " << ang_diff_w1 << std::endl;
    std::cout << "w2: " << lin_diff_w2 << " " << ang_diff_w2 << std::endl;
    std::cout << "w3: " << lin_diff_w3 << " " << ang_diff_w3 << std::endl;
    std::cout << "w4: " << lin_diff_w4 << " " << ang_diff_w4 << std::endl;

    Eigen::Vector2d lin_pf = Eigen::Vector2d(platform_force[0], platform_force[1]);

    double lin_force_norm = lin_pf.norm() == 0.0 ? 0.0 : 1.0;
    double moment_norm = platform_force[2] == 0.0 ? 0.0 : 1.0;

    double E1 = 2*lin_force_norm * lin_diff_w1 + moment_norm * ang_diff_w1;
    double E2 = 2*lin_force_norm * lin_diff_w2 + moment_norm * ang_diff_w2;
    double E3 = 2*lin_force_norm * lin_diff_w3 + moment_norm * ang_diff_w3;
    double E4 = 2*lin_force_norm * lin_diff_w4 + moment_norm * ang_diff_w4;

    // E1 = 0.0;
    // E2 = 0.0;
    // E3 = 0.0;
    // E4 = 0.0;

    std::cout << "E: " << E1 << " " << E2 << " " << E3 << " " << E4 << std::endl;

    // change the platform force rotation
    KDL::Rotation R = KDL::Rotation::RotZ(DEG2RAD(90.0));
    KDL::Vector pf_vec = KDL::Vector(platform_force[0], platform_force[1], 0.0);
    KDL::Vector pf_vec_rot = R * pf_vec;
    platform_force[0] = pf_vec_rot.x();
    platform_force[1] = pf_vec_rot.y();

    std::cout << "platform force_R: ";
    print_array(platform_force, 3);

    // approach 2
    double tau_wheel_ref[8] = { E1, -E1,
                                E2, -E2,
                                E3, -E3, 
                                E4, -E4};
    // double tau_wheel_ref[8]{};
    double tau_wheel_c2[8]{};
    double force_dist_mat_whl[3 * 2 * robot.mobile_base->mediator->kelo_base_config->nWheels]{};

    kelo_pltf_frc_comp_mat_whl(robot.mobile_base->mediator->kelo_base_config->nWheels,
                               robot.mobile_base->mediator->kelo_base_config->wheel_coordinates,
                               caster_offsets, wheel_distances, wheel_diameters,
                               robot.mobile_base->state->pivot_angles, force_dist_mat_whl);

    kelo_pltf_slv_inv_frc_dist_cgls(robot.mobile_base->mediator->kelo_base_config->nWheels,
                                    force_dist_mat_whl, w_drive, platform_force, tau_wheel_ref,
                                    tau_wheel_c2);

    // set torques
    set_mobile_base_torques(&robot, tau_wheel_c2);

    // KDL::Vector2 w1_attachment_vector = KDL::Vector2(frame1.p.x(), frame1.p.y());
    // KDL::Vector2 w2_attachment_vector = KDL::Vector2(frame2.p.x(), frame2.p.y());
    // KDL::Vector2 w3_attachment_vector = KDL::Vector2(frame3.p.x(), frame3.p.y());
    // KDL::Vector2 w4_attachment_vector = KDL::Vector2(frame4.p.x(), frame4.p.y());

    // w1_attachment_vector = w1_attachment_vector / w1_attachment_vector.Norm();
    // w2_attachment_vector = w2_attachment_vector / w2_attachment_vector.Norm();
    // w3_attachment_vector = w3_attachment_vector / w3_attachment_vector.Norm();
    // w4_attachment_vector = w4_attachment_vector / w4_attachment_vector.Norm();

    // // find the tangents of the attachment vectors in cw or ccw based on platform force [2] -
    // // moment if the moment is positive, the tangents are in ccw, otherwise in cw
    // KDL::Rotation2 rot_ccw = KDL::Rotation2::Rot(M_PI / 2);
    // KDL::Rotation2 rot_cw = KDL::Rotation2::Rot(-M_PI / 2);

    // KDL::Vector2 w1_tangent =
    //     platform_force[2] > 0 ? rot_ccw * w1_attachment_vector : rot_cw * w1_attachment_vector;
    // KDL::Vector2 w2_tangent =
    //     platform_force[2] > 0 ? rot_ccw * w2_attachment_vector : rot_cw * w2_attachment_vector;
    // KDL::Vector2 w3_tangent =
    //     platform_force[2] > 0 ? rot_ccw * w3_attachment_vector : rot_cw * w3_attachment_vector;
    // KDL::Vector2 w4_tangent =
    //     platform_force[2] > 0 ? rot_ccw * w4_attachment_vector : rot_cw * w4_attachment_vector;

    // // get the current pivot directions from the fk around the z-axis
    // KDL::Vector2 w1_pivot_direction = KDL::Vector2(frame1.M.UnitX().x(), frame1.M.UnitX().y());
    // KDL::Vector2 w2_pivot_direction = KDL::Vector2(frame2.M.UnitX().x(), frame2.M.UnitX().y());
    // KDL::Vector2 w3_pivot_direction = KDL::Vector2(frame3.M.UnitX().x(), frame3.M.UnitX().y());
    // KDL::Vector2 w4_pivot_direction = KDL::Vector2(frame4.M.UnitX().x(), frame4.M.UnitX().y());

    // // get the angular difference between the pivot direction and the tangent
    // double w1_angular_diff =
    //     atan2(w1_pivot_direction.x() * w1_tangent.y() - w1_pivot_direction.y() * w1_tangent.x(),
    //           w1_pivot_direction.x() * w1_tangent.x() + w1_pivot_direction.y() * w1_tangent.y());
    // double w2_angular_diff =
    //     atan2(w2_pivot_direction.x() * w2_tangent.y() - w2_pivot_direction.y() * w2_tangent.x(),
    //           w2_pivot_direction.x() * w2_tangent.x() + w2_pivot_direction.y() * w2_tangent.y());
    // double w3_angular_diff =
    //     atan2(w3_pivot_direction.x() * w3_tangent.y() - w3_pivot_direction.y() * w3_tangent.x(),
    //           w3_pivot_direction.x() * w3_tangent.x() + w3_pivot_direction.y() * w3_tangent.y());
    // double w4_angular_diff =
    //     atan2(w4_pivot_direction.x() * w4_tangent.y() - w4_pivot_direction.y() * w4_tangent.x(),
    //           w4_pivot_direction.x() * w4_tangent.x() + w4_pivot_direction.y() * w4_tangent.y());

    // // get the angular difference between the pivot direction and platform linear force
    // double w1_linear_diff = atan2(
    //     w1_pivot_direction.x() * platform_force[1] - w1_pivot_direction.y() * platform_force[0],
    //     w1_pivot_direction.x() * platform_force[0] + w1_pivot_direction.y() * platform_force[1]);
    // double w2_linear_diff = atan2(
    //     w2_pivot_direction.x() * platform_force[1] - w2_pivot_direction.y() * platform_force[0],
    //     w2_pivot_direction.x() * platform_force[0] + w2_pivot_direction.y() * platform_force[1]);
    // double w3_linear_diff = atan2(
    //     w3_pivot_direction.x() * platform_force[1] - w3_pivot_direction.y() * platform_force[0],
    //     w3_pivot_direction.x() * platform_force[0] + w3_pivot_direction.y() * platform_force[1]);
    // double w4_linear_diff = atan2(
    //     w4_pivot_direction.x() * platform_force[1] - w4_pivot_direction.y() * platform_force[0],
    //     w4_pivot_direction.x() * platform_force[0] + w4_pivot_direction.y() * platform_force[1]);

    // std::cout << "w lin: " << w1_linear_diff << " " << w2_linear_diff << " " << w3_linear_diff
    //           << " " << w4_linear_diff << std::endl;
    // std::cout << "w ang: " << w1_angular_diff << " " << w2_angular_diff << " " << w3_angular_diff
    //           << " " << w4_angular_diff << std::endl;

    // // eigen vec of linear platform force
    // Eigen::Vector2d lin_platform_force;
    // lin_platform_force(0) = platform_force[0];
    // lin_platform_force(1) = platform_force[1];

    // // normalize the linear platform force
    // lin_platform_force.normalize();

    // // 2x2 matrix
    // Eigen::Matrix2d lin_ctrl_matrix;
    // // x-axis into the direction of the platform force
    // // complement y-axis orthogonal to x-axis
    // lin_ctrl_matrix << lin_platform_force(0), -lin_platform_force(1), lin_platform_force(1),
    //     lin_platform_force(0);

    // // wheel 1
    // KDL::Vector attachment_vector_w1 = frame1.p;
    // attachment_vector_w1.Normalize();

    // Eigen::Matrix2d w1_ang_ctrl_matrix;
    // w1_ang_ctrl_matrix << attachment_vector_w1(1), attachment_vector_w1(0),
    //     -attachment_vector_w1(0), attachment_vector_w1(1);

    // Eigen::Matrix2d w1_rb_cori_matrix;
    // w1_rb_cori_matrix << cos(robot.mobile_base->state->pivot_angles[0]),
    //     -sin(robot.mobile_base->state->pivot_angles[0]),
    //     sin(robot.mobile_base->state->pivot_angles[0]),
    //     cos(robot.mobile_base->state->pivot_angles[0]);

    // Eigen::Matrix2d w1_lin_diff_matrix = w1_rb_cori_matrix.transpose() * lin_ctrl_matrix;
    // Eigen::Matrix2d w1_ang_diff_matrix = w1_rb_cori_matrix.transpose() * w1_ang_ctrl_matrix;

    // double w1_lin_log_vee_omega = atan2(w1_lin_diff_matrix(1, 0), w1_lin_diff_matrix(0, 0));
    // double w1_ang_log_vee_omega = atan2(w1_ang_diff_matrix(1, 0), w1_ang_diff_matrix(0, 0));

    // // wheel 2
    // KDL::Vector attachment_vector_w2 = frame2.p;
    // attachment_vector_w2.Normalize();

    // Eigen::Matrix2d w2_ang_ctrl_matrix;
    // w2_ang_ctrl_matrix << attachment_vector_w2(1), attachment_vector_w2(0),
    //     -attachment_vector_w2(0), attachment_vector_w2(1);

    // Eigen::Matrix2d w2_rb_cori_matrix;
    // w2_rb_cori_matrix << cos(robot.mobile_base->state->pivot_angles[1]),
    //     -sin(robot.mobile_base->state->pivot_angles[1]),
    //     sin(robot.mobile_base->state->pivot_angles[1]),
    //     cos(robot.mobile_base->state->pivot_angles[1]);

    // Eigen::Matrix2d w2_lin_diff_matrix = w2_rb_cori_matrix.transpose() * lin_ctrl_matrix;
    // Eigen::Matrix2d w2_ang_diff_matrix = w2_rb_cori_matrix.transpose() * w2_ang_ctrl_matrix;

    // double w2_lin_log_vee_omega = atan2(w2_lin_diff_matrix(0, 1), w2_lin_diff_matrix(0, 0));
    // double w2_ang_log_vee_omega = atan2(w2_ang_diff_matrix(0, 1), w2_ang_diff_matrix(0, 0));

    // // wheel 3
    // KDL::Vector attachment_vector_w3 = frame3.p;
    // attachment_vector_w3.Normalize();

    // Eigen::Matrix2d w3_ang_ctrl_matrix;
    // w3_ang_ctrl_matrix << attachment_vector_w3(1), attachment_vector_w3(0),
    //     -attachment_vector_w3(0), attachment_vector_w3(1);

    // // 2x2 matrix
    // Eigen::Matrix2d w3_rb_cori_matrix;
    // w3_rb_cori_matrix << cos(robot.mobile_base->state->pivot_angles[2]),
    //     -sin(robot.mobile_base->state->pivot_angles[2]),
    //     sin(robot.mobile_base->state->pivot_angles[2]),
    //     cos(robot.mobile_base->state->pivot_angles[2]);

    // // compute difference between matrices
    // Eigen::Matrix2d w3_lin_diff_matrix = w3_rb_cori_matrix.transpose() * lin_ctrl_matrix;
    // Eigen::Matrix2d w3_ang_diff_matrix = w3_rb_cori_matrix.transpose() * w3_ang_ctrl_matrix;

    // double w3_lin_log_vee_omega = atan2(w3_lin_diff_matrix(1, 0), w3_lin_diff_matrix(0, 0));
    // double w3_ang_log_vee_omega = atan2(w3_ang_diff_matrix(1, 0), w3_ang_diff_matrix(0, 0));

    // // wheel 4
    // KDL::Vector attachment_vector_w4 = frame4.p;
    // attachment_vector_w4.Normalize();

    // Eigen::Matrix2d w4_ang_ctrl_matrix;
    // w4_ang_ctrl_matrix << attachment_vector_w4(1), attachment_vector_w4(0),
    //     -attachment_vector_w4(0), attachment_vector_w4(1);

    // Eigen::Matrix2d w4_rb_cori_matrix;
    // w4_rb_cori_matrix << cos(robot.mobile_base->state->pivot_angles[3]),
    //     -sin(robot.mobile_base->state->pivot_angles[3]),
    //     sin(robot.mobile_base->state->pivot_angles[3]),
    //     cos(robot.mobile_base->state->pivot_angles[3]);

    // Eigen::Matrix2d w4_lin_diff_matrix = w4_rb_cori_matrix.transpose() * lin_ctrl_matrix;
    // Eigen::Matrix2d w4_ang_diff_matrix = w4_rb_cori_matrix.transpose() * w4_ang_ctrl_matrix;

    // double w4_lin_log_vee_omega = atan2(w4_lin_diff_matrix(1, 0), w4_lin_diff_matrix(0, 0));
    // double w4_ang_log_vee_omega = atan2(w4_ang_diff_matrix(1, 0), w4_ang_diff_matrix(0, 0));

    // std::cout << "w1 omega: " << w1_lin_log_vee_omega << " " << w1_ang_log_vee_omega << std::endl;
    // std::cout << "w2 omega: " << w2_lin_log_vee_omega << " " << w2_ang_log_vee_omega << std::endl;
    // std::cout << "w3 omega: " << w3_lin_log_vee_omega << " " << w3_ang_log_vee_omega << std::endl;
    // std::cout << "w4 omega: " << w4_lin_log_vee_omega << " " << w4_ang_log_vee_omega << std::endl;

    // usleep(90000);

    // std::cout << "wheels tau_c2: ";
    // print_array(tau_wheel_c2, 8);

    // std::cout << "w1 tau: "
    //           << lin_platform_force.norm() * w1_lin_log_vee_omega +
    //                  platform_force[2] * w1_ang_log_vee_omega
    //           << std::endl;
    // std::cout << "w2 tau: "
    //           << lin_platform_force.norm() * w2_lin_log_vee_omega +
    //                  platform_force[2] * w2_ang_log_vee_omega
    //           << std::endl;
    // std::cout << "w3 tau: "
    //           << lin_platform_force.norm() * w3_lin_log_vee_omega +
    //                  platform_force[2] * w3_ang_log_vee_omega
    //           << std::endl;
    // std::cout << "w4 tau: "
    //           << lin_platform_force.norm() * w4_lin_log_vee_omega +
    //                  platform_force[2] * w4_ang_log_vee_omega
    //           << std::endl;

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