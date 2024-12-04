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

#include <robif2b/functions/ethercat.h>
#include <robif2b/functions/kelo_drive.h>

#include <unsupported/Eigen/MatrixFunctions>

#define NUM_DRIVES 4
#define NUM_SLAVES 8

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

static struct
{
  int num_drives;
  struct
  {
    struct robif2b_kelo_drive_api_msr_pdo drv_msr_pdo[NUM_DRIVES];
    struct robif2b_kelo_drive_api_cmd_pdo drv_cmd_pdo[NUM_DRIVES];
  } ecat_comm;
  struct
  {
    const char *ethernet_if;
    int error_code;
    int num_exposed_slaves;
    int num_found_slaves;
    int num_active_slaves;
    int slave_idx[NUM_DRIVES];
    const char *name[NUM_DRIVES];
    unsigned int prod_code[NUM_DRIVES];
    size_t input_size[NUM_DRIVES];
    size_t output_size[NUM_DRIVES];
    bool is_connected[NUM_DRIVES];
  } ecat;
  struct
  {
    double pvt_off[NUM_DRIVES];
    double pvt_pos[NUM_DRIVES];
    double pvt_vel[NUM_DRIVES];
    double whl_pos[NUM_DRIVES * 2];
    double whl_vel[NUM_DRIVES * 2];
    double imu_ang_vel[NUM_DRIVES * 3];
    double imu_lin_acc[NUM_DRIVES * 3];
    double bat_volt;
    double bat_cur;
    double bat_pwr;
    int bat_lvl;
  } kelo_msr;
  struct
  {
    enum robif2b_ctrl_mode ctrl_mode[NUM_DRIVES];
    double vel[NUM_DRIVES * 2];
    double trq[NUM_DRIVES * 2];
    double cur[NUM_DRIVES * 2];
    double max_current[NUM_DRIVES * 2];
    double trq_const[NUM_DRIVES * 2];
  } kelo_cmd;
} state;

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
  if (argc == 4)
  {
    pf[0] = std::stod(argv[1]);
    pf[1] = std::stod(argv[2]);
    pf[2] = std::stod(argv[3]);
  }
  else
  {
    printf(
        "Usage: ./freddy_base_control_robif2b <platform_force_x> <platform_force_y> <platform_torque_z>\n");
    exit(1);
  }

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

  EthercatConfig *ethercat_config = new EthercatConfig();

  MobileBase<Robile> freddy_base;
  Robile robile;
  robile.ethercat_config = nullptr;
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
  initialize_robot(&robot, robot_urdf, ethernet_interface, false);

  // Configuration
  state.num_drives = NUM_DRIVES;
  state.ecat.ethernet_if = "eno1";
  state.ecat.num_exposed_slaves = NUM_DRIVES;
  state.ecat.slave_idx[0] = index_to_EtherCAT[0];
  state.ecat.slave_idx[1] = index_to_EtherCAT[1];
  state.ecat.slave_idx[2] = index_to_EtherCAT[2];
  state.ecat.slave_idx[3] = index_to_EtherCAT[3];

  for (int i = 0; i < NUM_DRIVES; i++)
  {
    state.kelo_cmd.ctrl_mode[i] = ROBIF2B_CTRL_MODE_VELOCITY,
    state.kelo_cmd.max_current[i * 2 + 0] = 10;  // [A]
    state.kelo_cmd.max_current[i * 2 + 1] = 10;  // [A]
    state.kelo_cmd.trq_const[i * 2 + 0] = 0.29;  // [Nm/A]
    state.kelo_cmd.trq_const[i * 2 + 1] = 0.29;  // [Nm/A]
  }
  state.kelo_msr.pvt_off[0] = pivot_angles_deviation[0];
  state.kelo_msr.pvt_off[1] = pivot_angles_deviation[1];
  state.kelo_msr.pvt_off[2] = pivot_angles_deviation[2];
  state.kelo_msr.pvt_off[3] = pivot_angles_deviation[3];

  for (int i = 0; i < NUM_DRIVES; i++)
  {
    state.ecat.name[i] = "KELOD105";
    state.ecat.prod_code[i] = 0x02001001;
    state.ecat.input_size[i] = sizeof(state.ecat_comm.drv_msr_pdo[i]);
    state.ecat.output_size[i] = sizeof(state.ecat_comm.drv_cmd_pdo[i]);
  }

  // Connections
  struct robif2b_ethercat ecat;
  ecat.ethernet_if = state.ecat.ethernet_if;
  ecat.num_exposed_slaves = &state.ecat.num_exposed_slaves;
  ecat.slave_idx = state.ecat.slave_idx;
  ecat.name = state.ecat.name;
  ecat.product_code = state.ecat.prod_code;
  ecat.input_size = state.ecat.input_size;
  ecat.output_size = state.ecat.output_size;
  ecat.error_code = &state.ecat.error_code;
  ecat.num_initial_slaves = &state.ecat.num_found_slaves;
  ecat.num_current_slaves = &state.ecat.num_active_slaves;
  ecat.is_connected = state.ecat.is_connected;

  void *ecat_input[NUM_DRIVES] = {&state.ecat_comm.drv_msr_pdo[0], &state.ecat_comm.drv_msr_pdo[1],
                                  &state.ecat_comm.drv_msr_pdo[2],
                                  &state.ecat_comm.drv_msr_pdo[3]};
  const void *ecat_output[NUM_DRIVES] = {
      &state.ecat_comm.drv_cmd_pdo[0], &state.ecat_comm.drv_cmd_pdo[1],
      &state.ecat_comm.drv_cmd_pdo[2], &state.ecat_comm.drv_cmd_pdo[3]};

  ecat.input = ecat_input;
  ecat.output = ecat_output;

  struct robif2b_kelo_drive_encoder drive_enc = {
      .num_drives = &state.num_drives,
      .msr_pdo = &state.ecat_comm.drv_msr_pdo[0],
      .wheel_pos_msr = &state.kelo_msr.whl_pos[0],
      .wheel_vel_msr = &state.kelo_msr.whl_vel[0],
      .pivot_pos_off = &state.kelo_msr.pvt_off[0],
      .pivot_pos_msr = &state.kelo_msr.pvt_pos[0],
      .pivot_vel_msr = &state.kelo_msr.pvt_vel[0],
  };

  struct robif2b_kelo_drive_imu imu = {.num_drives = &state.num_drives,
                                       .msr_pdo = &state.ecat_comm.drv_msr_pdo[0],
                                       .imu_ang_vel_msr = &state.kelo_msr.imu_ang_vel[0],
                                       .imu_lin_acc_msr = &state.kelo_msr.imu_lin_acc[0]};

  struct robif2b_kelo_drive_actuator wheel_act = {.num_drives = &state.num_drives,
                                                  .cmd_pdo = &state.ecat_comm.drv_cmd_pdo[0],
                                                  .ctrl_mode = &state.kelo_cmd.ctrl_mode[0],
                                                  .act_vel_cmd = &state.kelo_cmd.vel[0],
                                                  .act_trq_cmd = &state.kelo_cmd.trq[0],
                                                  .act_cur_cmd = &state.kelo_cmd.cur[0],
                                                  .max_current = &state.kelo_cmd.max_current[0],
                                                  .trq_const = &state.kelo_cmd.trq_const[0]};

  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s
  double control_loop_timestep = desired_period.count();                               // s
  double *control_loop_dt = &control_loop_timestep;                                    // s

  // pid controller variables
  double Kp = 5.0;
  double Ki = 0.5;
  double Kd = 0.0;

  double w1_lin_prev_error = 0.0;
  double w1_lin_error_sum = 0.0;
  double w1_ang_prev_error = 0.0;
  double w1_ang_error_sum = 0.0;

  double w2_lin_prev_error = 0.0;
  double w2_lin_error_sum = 0.0;
  double w2_ang_prev_error = 0.0;
  double w2_ang_error_sum = 0.0;

  double w3_lin_prev_error = 0.0;
  double w3_lin_error_sum = 0.0;
  double w3_ang_prev_error = 0.0;
  double w3_ang_error_sum = 0.0;

  double w4_lin_prev_error = 0.0;
  double w4_lin_error_sum = 0.0;
  double w4_ang_prev_error = 0.0;
  double w4_ang_error_sum = 0.0;

  // Schedule
  robif2b_ethercat_configure(&ecat);
  if (state.ecat.error_code < 0)
    return -1;

  robif2b_ethercat_start(&ecat);
  if (state.ecat.error_code < 0)
    return -1;
  
  get_robot_data(&robot, *control_loop_dt);

  auto pgm_start_time = std::chrono::high_resolution_clock::now();

  int count = 0;

  while (true)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (flag)
    {
      printf("Exiting somewhat cleanly...\n");

      auto pgm_end_time = std::chrono::high_resolution_clock::now();
      printf("Program execution time: %f\n",
             std::chrono::duration<double>(pgm_end_time - pgm_start_time).count());

      free_robot_data(&robot);
      exit(0);
    }

    count++;
    // printf("\n");
    printf("count: %d\n", count);

    robif2b_ethercat_update(&ecat);
    if (state.ecat.error_code < 0)
      return -1;
    robif2b_kelo_drive_encoder_update(&drive_enc);

    for (int i = 0; i < 4; ++i) {
        robot.mobile_base->state->pivot_angles[i] = state.kelo_msr.pvt_pos[i];
    }
    for (int i = 0; i < 8; ++i) {
        robot.mobile_base->state->wheel_encoder_values[i] = state.kelo_msr.whl_pos[i];
        robot.mobile_base->state->qd_wheel[i] = state.kelo_msr.whl_vel[i];
    }

    get_robot_data(&robot, *control_loop_dt);

    // solver
    double platform_force[3] = {pf[0], pf[1], pf[2]};  // [N], [N], [Nm]

    double lin_offsets[robot.mobile_base->mediator->kelo_base_config->nWheels];
    double ang_offsets[robot.mobile_base->mediator->kelo_base_config->nWheels];
    get_pivot_alignment_offsets(&robot, platform_force, lin_offsets, ang_offsets);

    Eigen::Vector2d lin_pf = Eigen::Vector2d(platform_force[0], platform_force[1]);

    double lin_signal_w1 = 0.0;
    double ang_signal_w1 = 0.0;
    double lin_signal_w2 = 0.0;
    double ang_signal_w2 = 0.0;
    double lin_signal_w3 = 0.0;
    double ang_signal_w3 = 0.0;
    double lin_signal_w4 = 0.0;
    double ang_signal_w4 = 0.0;

    double kp[4] = {Kp, Kp, Kp, Kp};
    double ki[4] = {Ki, Ki, Ki, Ki};
    double kd[4] = {Kd, Kd, Kd, Kd};

    pidController(lin_offsets[0], kp[0], ki[0], kd[0], control_loop_timestep, w1_lin_error_sum,
                  10.0, w1_lin_prev_error, lin_signal_w1);
    pidController(ang_offsets[0], kp[0], ki[0], kd[0], control_loop_timestep, w1_ang_error_sum,
                  10.0, w1_ang_prev_error, ang_signal_w1);

    pidController(lin_offsets[1], kp[1], ki[1], kd[1], control_loop_timestep, w2_lin_error_sum,
                  10.0, w2_lin_prev_error, lin_signal_w2);
    pidController(ang_offsets[1], kp[1], ki[1], kd[1], control_loop_timestep, w2_ang_error_sum,
                  10.0, w2_ang_prev_error, ang_signal_w2);

    pidController(lin_offsets[2], kp[2], ki[2], kd[2], control_loop_timestep, w3_lin_error_sum,
                  10.0, w3_lin_prev_error, lin_signal_w3);
    pidController(ang_offsets[2], kp[2], ki[2], kd[2], control_loop_timestep, w3_ang_error_sum,
                  10.0, w3_ang_prev_error, ang_signal_w3);

    pidController(lin_offsets[3], kp[3], ki[3], kd[3], control_loop_timestep, w4_lin_error_sum,
                  10.0, w4_lin_prev_error, lin_signal_w4);
    pidController(ang_offsets[3], kp[3], ki[3], kd[3], control_loop_timestep, w4_ang_error_sum,
                  10.0, w4_ang_prev_error, ang_signal_w4);

    double lin_signals[4] = {lin_signal_w1, lin_signal_w2, lin_signal_w3, lin_signal_w4};
    double ang_signals[4] = {ang_signal_w1, ang_signal_w2, ang_signal_w3, ang_signal_w4};

    printf("pivot angles: ");
    print_array(robot.mobile_base->state->pivot_angles, 4);

    // base solver
    double tau_wheel_c[8]{};
    base_fd_solver_with_alignment(&robot, platform_force, lin_signals, ang_signals, tau_wheel_c);
    printf("torques1: ");
    print_array(tau_wheel_c, 8);

    // set torques
    double tau_limit = 20.0;
    for (size_t i = 0; i < 8; i++)
    {
      if (tau_wheel_c[i] > tau_limit)
      {
        tau_wheel_c[i] = tau_limit;
      }
      else if (tau_wheel_c[i] < -tau_limit)
      {
        tau_wheel_c[i] = -tau_limit;
      }
    }

    // printf("torques3: ");
    // print_array(tau_wheel_c, 8);

    printf("\n");

    // for (size_t i = 0; i < 4; i++)
    // {
    //   state.kelo_cmd.trq[2 * i + 1] = tau_wheel_c[2 * i];
    //   state.kelo_cmd.trq[2 * i]     = tau_wheel_c[2 * i + 1];
    // }

    for (size_t i = 0; i < 4; i++)
    {
      state.kelo_cmd.vel[2 * i + 1] = lin_signals[i];
      state.kelo_cmd.vel[2 * i] = lin_signals[i];
    }

    if (count > 2)
    {
      robif2b_kelo_drive_actuator_update(&wheel_act);
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
  }

  robif2b_kelo_drive_actuator_stop(&wheel_act);
  robif2b_ethercat_stop(&ecat);
  robif2b_ethercat_shutdown(&ecat);
  free_robot_data(&robot);

  return 0;
}