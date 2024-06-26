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

  KeloBaseConfig kelo_base_config;
  kelo_base_config.nWheels = 4;
  int index_to_EtherCAT[4] = {6, 7, 3, 4};
  kelo_base_config.index_to_EtherCAT = index_to_EtherCAT;
  kelo_base_config.radius = 0.115 / 2;
  kelo_base_config.castor_offset = 0.01;
  kelo_base_config.half_wheel_distance = 0.0775 / 2;
  double wheel_coordinates[8] = {0.188, 0.2075, 
                                -0.188, 0.2075, 
                                -0.188, -0.2075, 
                                 0.188, -0.2075};
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
  std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "freddy.urdf").string();

  char ethernet_interface[100] = "eno1";
  initialize_robot(robot_urdf, ethernet_interface, &robot);

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
      1.0, 0.0, 0.0,  // xx, xy, xm
      0.0, 1.0, 0.0,  // yx, yy, ym
      0.0, 0.0, 1.0   // mx, my, mm
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

    // update_base_state(robot.mobile_base->mediator->kelo_base_config,
    //                   robot.mobile_base->mediator->ethercat_config);
    get_robot_data(&robot, *control_loop_dt);
    std::cout << std::endl;

    // std::cout << "odom: ";
    // print_array(robot.mobile_base->state->x_platform, 3);

    // solvers
    // fd solver
    double platform_force[3] = {150.0, 0.0, 0.0};  // [N], [N], [Nm]

    KDL::Rotation rot = KDL::Rotation::RotZ(DEG2RAD(90));
    KDL::Vector vec = KDL::Vector(platform_force[0], platform_force[1], 0.0);
    vec = rot * vec;
    platform_force[0] = vec.x();
    platform_force[1] = vec.y();

    std::cout << "platform force: ";
    print_array(platform_force, 3);

    double fd_solver_robile_platform_wrench[6] = {50.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // double tau_wheel_ref[8] = {0.0, 1.0, 
    //                            0.0, 1.0, 
    //                            2.0, 0.0, 
    //                            0.0, 1.0};
    double tau_wheel_ref[8]{};
    double tau_wheel_u[8]{};
    double tau_wheel_c[8]{};
    double fd_solver_robile_output_torques[8]{};

    double force_dist_mat[3 * 2 * robot.mobile_base->mediator->kelo_base_config->nWheels]{};

    kelo_pltf_frc_comp_mat_whl(robot.mobile_base->mediator->kelo_base_config->nWheels,
                               robot.mobile_base->mediator->kelo_base_config->wheel_coordinates,
                               caster_offsets, wheel_distances, wheel_diameters,
                               robot.mobile_base->state->pivot_angles, force_dist_mat);

    // kelo_pltf_vel_dist_mat(robot.mobile_base->mediator->kelo_base_config->nWheels,
    //                        robot.mobile_base->mediator->kelo_base_config->wheel_coordinates,
    //                        caster_offsets, wheel_distances, wheel_diameters,
    //                        robot.mobile_base->state->pivot_angles, force_dist_mat);

    kelo_pltf_slv_inv_frc_dist_ugls(robot.mobile_base->mediator->kelo_base_config->nWheels,
                                    force_dist_mat, w_platform, w_drive, platform_force,
                                    tau_wheel_u);

    kelo_pltf_slv_inv_frc_dist_cgls(robot.mobile_base->mediator->kelo_base_config->nWheels,
                                    force_dist_mat, w_drive, platform_force, tau_wheel_ref,
                                    tau_wheel_c);

    base_fd_solver(&robot, fd_solver_robile_platform_wrench, fd_solver_robile_output_torques);

    std::cout << "pivolt angles: ";
    print_array(robot.mobile_base->state->pivot_angles, 4);

    std::cout << "[sven] wheels tau_u: ";
    print_array(tau_wheel_u, 8);
    std::cout << "[sven] wheels tau_c: ";
    print_array(tau_wheel_c, 8);
    std::cout << "[sdp] wheels tau   : ";
    print_array(fd_solver_robile_output_torques, 8);

    // set torques
    set_mobile_base_torques(&robot, tau_wheel_c);

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