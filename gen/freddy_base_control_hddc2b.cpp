#include <array>
#include <string>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <controllers/pid_controller.hpp>
#include <motion_spec_utils/utils.hpp>
#include <motion_spec_utils/math_utils.hpp>
#include <motion_spec_utils/solver_utils.hpp>
#include <csignal>

#include <unsupported/Eigen/MatrixFunctions>

#include "motion_spec_utils/log_structs.hpp"

#include <hddc2b/functions/platform.h>
#include <hddc2b/functions/solver.h>
#include <hddc2b/functions/drive.h>
#include <hddc2b/functions/wheel.h>
#include <solver.h>

#include "motion_spec_utils/log_structs.hpp"

volatile sig_atomic_t flag = 0;

void handle_signal(int sig)
{
  static int signal_caught = 0;
  if (!signal_caught)
  {
    signal_caught = 1;
    flag = 1;
    printf("Caught signal %d (%s)\n", sig, strsignal(sig));
  }
}

void print_matrix(int rows, int cols, const double *a)
{
  printf("[");
  for (int m_ = 0; m_ < rows; m_++)
  {
    printf("[");
    for (int n_ = 0; n_ < cols; n_++)
    {
      printf("%10f", a[m_ + n_ * rows]);
      if (n_ != cols - 1)
        printf(", ");
    }
    printf("]");
    if (m_ != rows - 1)
      printf(",\n ");
  }
  printf("]");
}

#define NUM_DRV 4
#define NUM_WHL_COORD 2
#define NUM_GND_COORD 2
#define NUM_DRV_COORD 2
#define NUM_PLTF_COORD 3
#define NUM_G_COORD ((NUM_DRV_COORD) * (NUM_PLTF_COORD))
#define EPS 0.001

int main(int argc, char **argv)
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

  // read the platform force from the command line
  double pf[3] = {0.0, 0.0, 0.0};
  int count_limit = -1;
  if (argc == 4)
  {
    pf[0] = std::stod(argv[1]);
    pf[1] = std::stod(argv[2]);
    pf[2] = std::stod(argv[3]);
  }
  else if (argc == 5)
  {
    pf[0] = std::stod(argv[1]);
    pf[1] = std::stod(argv[2]);
    pf[2] = std::stod(argv[3]);
    count_limit = std::stoi(argv[4]);
  }
  else
  {
    printf(
        "Usage: ./freddy_base_align <platform_force_x> <platform_force_y> <platform_torque_z>\n");
    exit(1);
  }

  printf("count_limit: %d\n", count_limit);

  // log structs
  std::string log_dir = "../../logs/data/wheel_align_log";
  char log_dir_name[100];
  get_new_folder_name(log_dir.c_str(), log_dir_name);
  std::string run_id = log_dir_name;
  run_id = run_id.substr(run_id.find_last_of("/") + 1);
  std::filesystem::create_directories(log_dir_name);
  std::filesystem::permissions(log_dir_name, std::filesystem::perms::all);

  LogWheelAlignDataVector wheel_align_log_data_vec(log_dir_name, 1);

  // Initialize the robot structs
  KeloBaseConfig *kelo_base_config = new KeloBaseConfig();
  kelo_base_config->nWheels = 4;
  int index_to_EtherCAT[4] = {3, 4, 6, 7};
  kelo_base_config->index_to_EtherCAT = index_to_EtherCAT;
  kelo_base_config->radius = 0.115 / 2;
  kelo_base_config->castor_offset = 0.01;
  kelo_base_config->half_wheel_distance = 0.0775 / 2;
  double wheel_coordinates[8] = {0.195, 0.21, -0.195, 0.21, -0.195, -0.21, 0.195, -0.21};
  kelo_base_config->wheel_coordinates = wheel_coordinates;
  double pivot_angles_deviation[4] = {0.0, 0.0, 0.0, 0.0};
  kelo_base_config->pivot_angles_deviation = pivot_angles_deviation;

  // Diameter of each wheel.
  double wheel_diameter[NUM_DRV * 2] = {
      // [m]
      0.115, 0.115,  // fl-r, fl-l
      0.115, 0.115,  // rl-r, rl-l
      0.115, 0.115,  // rr-r, rr-l
      0.115, 0.115   // fr-r, fr-l
  };

  // Kinematic parameters of the differential drive part:
  // Distance of the wheels from the centre between the wheels.
  double wheel_distance[NUM_DRV] = {
      // [m]
      0.0775, 0.0775, 0.0775, 0.0775  // fl, rl, rr, fr
  };

  // Kinematic parameters of the castor drive part:
  // Distance of the axle from the pivot joint's axis
  double castor_offset[NUM_DRV] = {
      // [m]
      0.01, 0.01, 0.01, 0.01  // fl, rl, rr, fr
  };

  // For _singular_ platforms the relative weight between the platform-level
  // forces, i.e. for the platform in this example it has no impact on the
  // force distribution.
  double w_platform[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
      // [1/N^2], [1/(N Nm)], [1/(Nm)^2]
      1.0, 0.0, 0.0,  // xx, xy, xm
      0.0, 1.0, 0.0,  // yx, yy, ym
      0.0, 0.0, 1.0   // mx, my, mm
  };

  // For _redundant_ platforms the relative weight between the drive-level
  // forces. The entries here disable the front-right drive unit.
  double w_drive[NUM_DRV * 4] = {
      // [1/N^2]
      1.0, 0.0, 0.0, 1.0,  // fl-xx, fl-xy, fl-yx, fl-yy
      1.0, 0.0, 0.0, 1.0,  // rl-xx, rl-xy, rl-yx, rl-yy
      1.0, 0.0, 0.0, 1.0,  // rr-xx, rr-xy, rr-yx, rr-yy
      1.0, 0.0, 0.0, 1.0   // fr-xx, fr-xy, fr-yx, fr-yy
  };

  // Weight of the angular and linear alignment distance, respectively.
  // The weight for the front-right drive unit means that it will always have
  // a "zero" alignment distance.
  // double w_align[NUM_DRV * 2] = {
  //     //
  //     1.0, 1.0,  // fl-ang, fl-lin
  //     1.0, 1.0,  // rl-ang, rl-lin
  //     1.0, 1.0,  // rr-ang, rr-lin
  //     1.0, 1.0   // fr-ang, fr-lin
  // };
  double w_align[NUM_DRV * 2] = {
      //
      0.5, 0.5,  // fl-ang, fl-lin
      0.5, 0.5,  // rl-ang, rl-lin
      0.5, 0.5,  // rr-ang, rr-lin
      0.5, 0.5   // fr-ang, fr-lin
  };

  EthercatConfig *ethercat_config = new EthercatConfig();

  MobileBase<Robile> freddy_base;
  Robile robile;
  robile.ethercat_config = ethercat_config;
  robile.kelo_base_config = kelo_base_config;

  freddy_base.mediator = &robile;
  freddy_base.state = new MobileBaseState();

  Freddy robot = {nullptr, nullptr, &freddy_base};

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "freddy_corrected_base.urdf").string();

  char ethernet_interface[100] = "eno1";
  initialize_robot(&robot, robot_urdf, ethernet_interface);

  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s
  double control_loop_timestep = desired_period.count();                               // s
  double *control_loop_dt = &control_loop_timestep;                                    // s

  double plat_clip_force = 20.0;
  double plat_sat_force = 300.0;

  // Force composition matrix (from drive forces to platform force)
  double g[NUM_DRV * NUM_G_COORD];

  // Drive force that results from distributing the platform-level force
  double f_drive[NUM_DRV * NUM_DRV_COORD];  // [N]
  bzero(f_drive, NUM_DRV * NUM_GND_COORD * sizeof(double));

  // Force at the wheel-ground contact point
  double f_wheel[NUM_DRV * NUM_GND_COORD];  // [N]

  update_base_state(robot.mobile_base->mediator->kelo_base_config,
                    robot.mobile_base->mediator->ethercat_config);
  get_robot_data(&robot, *control_loop_dt);

  int count = 0;

  bool while_condition = count_limit > 0 ? count < count_limit : true;

  while (while_condition)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    while_condition = count_limit > 0 ? count < count_limit : true;

    if (flag)
    {
      wheel_align_log_data_vec.writeToOpenFile();

      printf("Exiting somewhat cleanly...\n");
      free_robot_data(&robot);
      exit(0);
    }

    printf("\ncount: %d\n", count);

    get_robot_data(&robot, *control_loop_dt);

    double plat_force[3] = {pf[0], pf[1], pf[2]};  // [N], [N], [Nm]

    // saturation
    for (size_t i = 0; i < 3; i++)
    {
      if (plat_force[i] > plat_sat_force)
      {
        plat_force[i] = plat_sat_force;
      }
      else if (plat_force[i] < -plat_sat_force)
      {
        plat_force[i] = -plat_sat_force;
      }
    }

    printf("platform force: ");
    print_array(plat_force, 3);

    printf("pivot angles:     ");
    print_array(robot.mobile_base->state->pivot_angles, 4);

    printf("pivot velocities: ");
    print_array(robot.mobile_base->state->pivot_velocities, 4);

    // Actuator torque
    double tau_wheel[NUM_DRV * NUM_WHL_COORD];  // [Nm]

    double drive_align_dsts[NUM_DRV * NUM_DRV_COORD];  // [N]
    hddc2b_pltf_drv_algn_dst(
        NUM_DRV, robot.mobile_base->mediator->kelo_base_config->wheel_coordinates, w_align,
        robot.mobile_base->state->pivot_angles, plat_force, &drive_align_dsts[1], 2);

    printf("\ndrive_align_dsts:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, drive_align_dsts);

    double f_drive_ref[NUM_DRV * NUM_DRV_COORD];  // [N]
    for (size_t i = 0; i < NUM_DRV; i++)
    {
      f_drive_ref[2 * i] = 0.0;

      if (fabs(drive_align_dsts[2 * i + 1]) < 40)
      {
        f_drive_ref[2 * i + 1] = 0.0;
      }
      else
      {
        f_drive_ref[2 * i + 1] = drive_align_dsts[2 * i + 1];
      }
    }

    printf("\nf_drive_ref:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, f_drive_ref);

    //
    // Force distribution to wheels ...
    //

    hddc2b_pltf_frc_comp_mat(NUM_DRV,
                             robot.mobile_base->mediator->kelo_base_config->wheel_coordinates,
                             robot.mobile_base->state->pivot_angles, g);

    double f_krnl[NUM_DRV * NUM_DRV_COORD];  // [N]
    double f_null[NUM_DRV * NUM_DRV_COORD];  // [N]
    hddc2b_pltf_frc_pltf_to_drv(NUM_DRV, EPS, g, w_platform, plat_force, w_drive, f_drive_ref,
                                f_krnl, f_null);
    printf("\nf_krnl:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, f_krnl);
    printf("\nf_null:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, f_null);

    // take y coordinate ratio of f_null to f_drive_ref
    // select the non-zero greatest value of f_null_ratio as the scaling factor
    double f_scale_factor = -INFINITY;
    for (int i = 0; i < NUM_DRV; i++)
    {
      if (fabs(f_null[1 + i * NUM_DRV_COORD]) < 0.1 || fabs(f_null[1 + i * NUM_DRV_COORD]) < EPS)
      {
        continue;
      }

      double ratio = (f_drive_ref[1 + i * NUM_DRV_COORD] / f_null[1 + i * NUM_DRV_COORD]);
      if (ratio > f_scale_factor)
        f_scale_factor = ratio;
    }

    // check if still infinity, then set to 1.0
    if (f_scale_factor == -INFINITY)
      f_scale_factor = 1.0;

    f_scale_factor = fabs(f_scale_factor);
    printf("\nf_scale_factor: %f", f_scale_factor);

    // scale f_null
    double f_null_scaled[NUM_DRV * NUM_DRV_COORD];  // [N]
    for (size_t i = 0; i < NUM_DRV; i++)
    {
      f_null_scaled[2 * i + 1] = f_null[2 * i + 1] * f_scale_factor;
    }
    printf("\nf_null scaled:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, f_null_scaled);

    double f_drv[NUM_DRV * NUM_DRV_COORD];  // [N]
    hddc2b_pltf_frc_redu_ref_fini(NUM_DRV, f_krnl, f_null_scaled, f_drv);
    printf("\nf_drv:\n");
    print_matrix(NUM_DRV_COORD, NUM_DRV, f_drv);

    hddc2b_drv_frc_pvt_to_gnd(NUM_DRV, wheel_distance, castor_offset, f_drv, f_wheel);
    printf("\nf_wheel:\n");
    print_matrix(NUM_GND_COORD, NUM_DRV, f_wheel);

    hddc2b_whl_frc_gnd_to_hub(NUM_DRV, wheel_diameter, f_wheel, tau_wheel);
    printf("\ntau_wheel:\n");
    print_matrix(NUM_WHL_COORD, NUM_DRV, tau_wheel);

    // saturate torques
    double tau_limit = 10.0;
    for (size_t i = 0; i < 8; i++)
    {
      if (tau_wheel[i] > tau_limit)
      {
        tau_wheel[i] = tau_limit;
      }
      else if (tau_wheel[i] < -tau_limit)
      {
        tau_wheel[i] = -tau_limit;
      }
    }

    printf("\n");

    // log data
    wheel_align_log_data_vec.addWheelAlignData(robot.mobile_base->state->pivot_angles, plat_force,
                                               tau_wheel, f_drive_ref, f_krnl, f_null, f_scale_factor,
                                               f_null_scaled, f_drv, f_wheel);

    if (count > 2)
    {
      // raise(SIGINT);
      set_mobile_base_torques(&robot, tau_wheel);
      update_base_state(robot.mobile_base->mediator->kelo_base_config,
                        robot.mobile_base->mediator->ethercat_config);
    }

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

    count++;
  }

  free_robot_data(&robot);

  return 0;
}