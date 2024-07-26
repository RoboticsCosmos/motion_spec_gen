#include <array>
#include <string>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <csignal>

#include <robif2b/functions/ethercat.h>
#include <robif2b/functions/kelo_drive.h>


#define NUM_DRIVES 4
#define NUM_SLAVES 8

volatile sig_atomic_t flag = 0;

void handle_signal(int sig)
{
  flag = 1;
  printf("Caught signal %d\n", sig);
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
  // struct sigaction sa;
  // sa.sa_handler = handle_signal;
  // sigemptyset(&sa.sa_mask);
  // sa.sa_flags = 0;

  // for (int i = 1; i < NSIG; ++i)
  // {
  //   if (sigaction(i, &sa, NULL) == -1)
  //   {
  //     perror("sigaction");
  //   }
  // }

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
        "Usage: ./freddy_base_align <platform_force_x> <platform_force_y> <platform_torque_z>\n");
    exit(1);
  }

  // Configuration
  state.num_drives = NUM_DRIVES;
  state.ecat.ethernet_if = "eno1";
  state.ecat.num_exposed_slaves = NUM_DRIVES;
  state.ecat.slave_idx[0] = 6;
  state.ecat.slave_idx[1] = 7;
  state.ecat.slave_idx[2] = 3;
  state.ecat.slave_idx[3] = 4;

  for (int i = 0; i < NUM_DRIVES; i++)
  {
    state.kelo_cmd.ctrl_mode[i] = ROBIF2B_CTRL_MODE_FORCE,
    state.kelo_cmd.max_current[i * 2 + 0] = 10;  // [A]
    state.kelo_cmd.max_current[i * 2 + 1] = 10;  // [A]
    state.kelo_cmd.trq_const[i * 2 + 0] = 0.29;  // [Nm/A]
    state.kelo_cmd.trq_const[i * 2 + 1] = 0.29;  // [Nm/A]
  }
  state.kelo_msr.pvt_off[0] = 5.310;
  state.kelo_msr.pvt_off[1] = 5.533;
  state.kelo_msr.pvt_off[2] = 1.563;
  state.kelo_msr.pvt_off[3] = 1.625;

  for (int i = 0; i < NUM_DRIVES; i++)
  {
    state.ecat.name[i] = "KELOD105";
    state.ecat.prod_code[i] = 0x02001001;
    state.ecat.input_size[i] = sizeof(state.ecat_comm.drv_msr_pdo[i]);
    state.ecat.output_size[i] = sizeof(state.ecat_comm.drv_cmd_pdo[i]);
  }

  // Connections
    struct robif2b_ethercat ecat;
    ecat.ethernet_if        = state.ecat.ethernet_if;
    ecat.num_exposed_slaves = &state.ecat.num_exposed_slaves;
    ecat.slave_idx          = state.ecat.slave_idx;
    ecat.name               = state.ecat.name;
    ecat.product_code       = state.ecat.prod_code;
    ecat.input_size         = state.ecat.input_size;
    ecat.output_size        = state.ecat.output_size;
    ecat.error_code         = &state.ecat.error_code;
    ecat.num_initial_slaves = &state.ecat.num_found_slaves;
    ecat.num_current_slaves = &state.ecat.num_active_slaves;
    ecat.is_connected       = state.ecat.is_connected;
    
    void *ecat_input[NUM_DRIVES] = {
        &state.ecat_comm.drv_msr_pdo[0],
        &state.ecat_comm.drv_msr_pdo[1],
        &state.ecat_comm.drv_msr_pdo[2],
        &state.ecat_comm.drv_msr_pdo[3]
    };
    const void *ecat_output[NUM_DRIVES] = {
        &state.ecat_comm.drv_cmd_pdo[0],
        &state.ecat_comm.drv_cmd_pdo[1],
        &state.ecat_comm.drv_cmd_pdo[2],
        &state.ecat_comm.drv_cmd_pdo[3]
    };

    ecat.input  = ecat_input;
    ecat.output = ecat_output;

    struct robif2b_kelo_drive_encoder drive_enc = {
        .num_drives    = &state.num_drives,
        .msr_pdo       = &state.ecat_comm.drv_msr_pdo[0],
        .wheel_pos_msr = &state.kelo_msr.whl_pos[0],
        .wheel_vel_msr = &state.kelo_msr.whl_vel[0],
        .pivot_pos_off = &state.kelo_msr.pvt_off[0],
        .pivot_pos_msr = &state.kelo_msr.pvt_pos[0],
        .pivot_vel_msr = &state.kelo_msr.pvt_vel[0],
    };

    struct robif2b_kelo_drive_imu imu = {
        .num_drives      = &state.num_drives,
        .msr_pdo         = &state.ecat_comm.drv_msr_pdo[0],
        .imu_ang_vel_msr = &state.kelo_msr.imu_ang_vel[0],
        .imu_lin_acc_msr = &state.kelo_msr.imu_lin_acc[0]
    };

    struct robif2b_kelo_drive_actuator wheel_act = {
        .num_drives  = &state.num_drives,
        .cmd_pdo     = &state.ecat_comm.drv_cmd_pdo[0],
        .ctrl_mode   = &state.kelo_cmd.ctrl_mode[0],
        .act_vel_cmd = &state.kelo_cmd.vel[0],
        .act_trq_cmd = &state.kelo_cmd.trq[0],
        .act_cur_cmd = &state.kelo_cmd.cur[0],
        .max_current = &state.kelo_cmd.max_current[0],
        .trq_const   = &state.kelo_cmd.trq_const[0]
    };

  const double desired_frequency = 1000.0;                                             // Hz
  const auto desired_period = std::chrono::duration<double>(1.0 / desired_frequency);  // s
  double control_loop_timestep = desired_period.count();                               // s
  double *control_loop_dt = &control_loop_timestep;                                    // s

  // pid controller variables
  double kp = 5.0;
  double ki = 0.01;
  double kd = 0.5;

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

  for (int i = 0; i < NUM_DRIVES; i++) {
      state.kelo_cmd.trq[i * 2 + 0] = 0.0;
      state.kelo_cmd.trq[i * 2 + 1] = 0.0;
  }

  // Schedule
  robif2b_ethercat_configure(&ecat);
  if (state.ecat.error_code < 0)
    return -1;

  robif2b_ethercat_start(&ecat);
  if (state.ecat.error_code < 0)
    return -1;

  double lin_offsets[NUM_DRIVES];
  double ang_offsets[NUM_DRIVES];

  // Freddy robot;

  // get_pivot_alignment_offsets(&robot, pf, lin_offsets, ang_offsets);

  state.kelo_cmd.trq[0] = 1.0;
  state.kelo_cmd.trq[1] = -1.0;

  int count = 0;

  while (true)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    // if (flag)
    // {
    //   robif2b_kelo_drive_actuator_stop(&wheel_act);
    //   robif2b_ethercat_stop(&ecat);
    //   robif2b_ethercat_shutdown(&ecat);
    //   free_robot_data(&robot);
    //   printf("Exiting somewhat cleanly...\n");
    //   exit(0);
    // }

    count++;
    printf("\n");
    // printf("count: %d\n", count);

    robif2b_ethercat_update(&ecat);
    if (state.ecat.error_code < 0)
      return -1;
    robif2b_kelo_drive_encoder_update(&drive_enc);

    for (int i = 0; i < NUM_DRIVES; i++)
    {
      printf(
          "drive [id=%i, conn=%i]: "
          "w_vel[0]=%5.2f - w_vel[1]=%5.2f - p_pos=%5.2f\n",
          i, state.ecat.is_connected[i + 1], state.kelo_msr.whl_vel[i * 2 + 0],
          state.kelo_msr.whl_vel[i * 2 + 1], state.kelo_msr.pvt_pos[i]);
    }

    // update_base_state(robot.mobile_base->mediator->kelo_base_config,
    //                   robot.mobile_base->mediator->ethercat_config);
    // get_robot_data(&robot, *control_loop_dt);

    // solver
    // double platform_force[3] = {pf[0], pf[1], pf[2]};  // [N], [N], [Nm]

    // compute the weights for the platform force
    // double platform_weights[2];
    // platform_weights[0] = abs(platform_force[2]) < 1e-6
                              // ? 1.0
    //                           : sqrt(pow(platform_force[0], 2) + pow(platform_force[1], 2)) /
    //                                 (sqrt(pow(platform_force[0], 2) + pow(platform_force[1], 2)
    //                                 +
    //                                       pow(platform_force[2], 2)));

    // platform_weights[1] = 1.0 - platform_weights[0];

    // // std::cout << "platform weights: ";
    // // print_array(platform_weights, 2);

    // double lin_offsets[NUM_DRIVES];
    // double ang_offsets[NUM_DRIVES];
    // // get_pivot_alignment_offsets(&robot, platform_force, lin_offsets, ang_offsets);

    // Eigen::Vector2d lin_pf = Eigen::Vector2d(platform_force[0], platform_force[1]);

    // double lin_force_weight = lin_pf.norm() == 0.0 ? 0.0 : platform_weights[0];
    // double moment_weight = platform_force[2] == 0.0 ? 0.0 : platform_weights[1];

    // double lin_signal_w1 = 0.0;
    // double ang_signal_w1 = 0.0;
    // double lin_signal_w2 = 0.0;
    // double ang_signal_w2 = 0.0;
    // double lin_signal_w3 = 0.0;
    // double ang_signal_w3 = 0.0;
    // double lin_signal_w4 = 0.0;
    // double ang_signal_w4 = 0.0;

    // pidController(lin_offsets[0], kp, ki, kd, control_loop_timestep, w1_lin_error_sum, 1.0,
    //               w1_lin_prev_error, lin_signal_w1);
    // pidController(ang_offsets[0], kp, ki, kd, control_loop_timestep, w1_ang_error_sum, 1.0,
    //               w1_ang_prev_error, ang_signal_w1);

    // pidController(lin_offsets[1], kp, ki, kd, control_loop_timestep, w2_lin_error_sum, 1.0,
    //               w2_lin_prev_error, lin_signal_w2);
    // pidController(ang_offsets[1], kp, ki, kd, control_loop_timestep, w2_ang_error_sum, 1.0,
    //               w2_ang_prev_error, ang_signal_w2);

    // pidController(lin_offsets[2], kp, ki, kd, control_loop_timestep, w3_lin_error_sum, 1.0,
    //               w3_lin_prev_error, lin_signal_w3);
    // pidController(ang_offsets[2], kp, ki, kd, control_loop_timestep, w3_ang_error_sum, 1.0,
    //               w3_ang_prev_error, ang_signal_w3);

    // pidController(lin_offsets[3], kp, ki, kd, control_loop_timestep, w4_lin_error_sum, 1.0,
    //               w4_lin_prev_error, lin_signal_w4);
    // pidController(ang_offsets[3], kp, ki, kd, control_loop_timestep, w4_ang_error_sum, 1.0,
    //               w4_ang_prev_error, ang_signal_w4);

    // double lin_signals[4] = {lin_signal_w1, lin_signal_w2, lin_signal_w3, lin_signal_w4};
    // double ang_signals[4] = {ang_signal_w1, ang_signal_w2, ang_signal_w3, ang_signal_w4};

    // double alignment_taus[NUM_DRIVES];
    // for (size_t i = 0; i < NUM_DRIVES; i++)
    // {
    //   alignment_taus[i] = lin_signals[i] * lin_force_weight + ang_signals[i] * moment_weight;
    // }

    // double tau_wheel_ref[NUM_DRIVES * 2];
    // for (size_t i = 0; i < NUM_DRIVES; i++)
    // {
    //   tau_wheel_ref[2 * i] = alignment_taus[i];
    //   tau_wheel_ref[2 * i + 1] = -alignment_taus[i];
    // }

    // double tau_wheel_c[8]{};
    // base_fd_solver_with_alignment(&robot, platform_force, lin_signals, ang_signals,
    //                               platform_weights, tau_wheel_c);

    // base_fd_solver(&robot, platform_force, tau_wheel_c);
    // for (size_t i = 0; i < 2; i++)
    // {
    //   tau_wheel_c[i] += tau_wheel_ref[i];
    // }

    // set torques
    // for (size_t i = 0; i < 8; i++)
    // {
    //   if (tau_wheel_c[i] > 3.0)
    //   {
    //     tau_wheel_c[i] = 3.0;
    //   }
    //   else if (tau_wheel_c[i] < -3.0)
    //   {
    //     tau_wheel_c[i] = -3.0;
    //   }
    // }

    if (count != 1)
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
    // std::cout << "control loop timestep: " << control_loop_timestep << std::endl;
  }

  robif2b_kelo_drive_actuator_stop(&wheel_act);
  robif2b_ethercat_stop(&ecat);
  robif2b_ethercat_shutdown(&ecat);

  // free_robot_data(&robot);

  return 0;
}