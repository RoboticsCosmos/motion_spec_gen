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

    count++;
    printf("\n");
    // printf("count: %d\n", count);

    update_base_state(robot.mobile_base->mediator->kelo_base_config,
                      robot.mobile_base->mediator->ethercat_config);
    get_robot_data(&robot, *control_loop_dt);
    std::cout << std::endl;

    // solver
    double platform_force[3] = {50.0, 0.0, 0.0};  // [N], [N], [Nm]

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

    Eigen::Vector2d lin_pf = Eigen::Vector2d(platform_force[0], platform_force[1]);

    double lin_force_norm = lin_pf.norm() == 0.0 ? 0.0 : 1.0;
    double moment_norm = platform_force[2] == 0.0 ? 0.0 : 1.0;

    double E1 = lin_force_norm * lin_diff_w1 + moment_norm * ang_diff_w1;
    double E2 = lin_force_norm * lin_diff_w2 + moment_norm * ang_diff_w2;
    double E3 = lin_force_norm * lin_diff_w3 + moment_norm * ang_diff_w3;
    double E4 = lin_force_norm * lin_diff_w4 + moment_norm * ang_diff_w4;

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