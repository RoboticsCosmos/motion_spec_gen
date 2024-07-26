// SPDX-License-Identifier: LGPL-3.0
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>

#include <robif2b/functions/ethercat.h>
#include <robif2b/functions/kelo_drive.h>
#include <robif2b/functions/kelo_power_board.h>


static long timespec_to_usec(const struct timespec *t) {
    const int NSEC_IN_USEC = 1000;
    const int USEC_IN_SEC  = 1000000;

    return t->tv_sec * USEC_IN_SEC + t->tv_nsec / NSEC_IN_USEC;
}

#define NUM_DRIVES 4
#define NUM_SLAVES 7

static struct {
    int num_drives;
    struct {
        struct timespec cycle_start;
        struct timespec cycle_end;
        long cycle_time_msr;      // [us]
        long cycle_time_exp;      // [us]
    } time;
    struct {
        struct robif2b_kelo_drive_api_msr_pdo drv_msr_pdo[NUM_DRIVES];
        struct robif2b_kelo_drive_api_cmd_pdo drv_cmd_pdo[NUM_DRIVES];
    } ecat_comm;
    struct {
        const char   *ethernet_if;
        int          error_code;
        int          num_exposed_slaves;
        int          num_found_slaves;
        int          num_active_slaves;
        int          slave_idx[NUM_DRIVES];
        const char   *name[NUM_DRIVES];
        unsigned int prod_code[NUM_DRIVES];
        size_t       input_size[NUM_DRIVES];
        size_t       output_size[NUM_DRIVES];
        bool         is_connected[NUM_DRIVES];
    } ecat;
    struct {
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
        int    bat_lvl;
    } kelo_msr;
    struct {
        enum robif2b_ctrl_mode ctrl_mode[NUM_DRIVES];
        double vel[NUM_DRIVES * 2];
        double trq[NUM_DRIVES * 2];
        double cur[NUM_DRIVES * 2];
        double max_current[NUM_DRIVES * 2];
        double trq_const[NUM_DRIVES * 2];
    } kelo_cmd;
} state;


int main()
{
    // Configuration
    state.num_drives              = NUM_DRIVES;
    state.time.cycle_time_exp     = 1000;        // [us]
    state.ecat.ethernet_if        = "eno1";
    state.ecat.num_exposed_slaves = NUM_DRIVES;
    state.ecat.slave_idx[0]       = 6;
    state.ecat.slave_idx[1]       = 7;
    state.ecat.slave_idx[2]       = 3;
    state.ecat.slave_idx[3]       = 4;

    for (int i = 0; i < NUM_DRIVES; i++) {
        state.kelo_cmd.ctrl_mode[i]           = ROBIF2B_CTRL_MODE_FORCE,
        state.kelo_cmd.max_current[i * 2 + 0] = 10;     // [A]
        state.kelo_cmd.max_current[i * 2 + 1] = 10;     // [A]
        state.kelo_cmd.trq_const[i * 2 + 0]   = 0.29;   // [Nm/A]
        state.kelo_cmd.trq_const[i * 2 + 1]   = 0.29;   // [Nm/A]
    }
    state.kelo_msr.pvt_off[0] = 5.310;
    state.kelo_msr.pvt_off[1] = 5.533;
    state.kelo_msr.pvt_off[2] = 1.563;
    state.kelo_msr.pvt_off[3] = 1.625;

    for (int i = 0; i < NUM_DRIVES; i++) {
        state.ecat.name[i]        = "KELOD105";
        state.ecat.prod_code[i]   = 0x02001001;
        state.ecat.input_size[i]  = sizeof(state.ecat_comm.drv_msr_pdo[i]);
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

    for (int i = 0; i < NUM_DRIVES; i++) {
        state.kelo_cmd.trq[i * 2 + 0] = 0.0;
        state.kelo_cmd.trq[i * 2 + 1] = 0.0;
    }
    // state.kelo_cmd.trq[0] = 1.0;
    // state.kelo_cmd.trq[1] = -1.0;

    // Schedule
    robif2b_ethercat_configure(&ecat);
    if (state.ecat.error_code < 0) return -1;

    robif2b_ethercat_start(&ecat);
    if (state.ecat.error_code < 0) return -1;

    while (true) {
        clock_gettime(CLOCK_MONOTONIC, &state.time.cycle_start);

        robif2b_ethercat_update(&ecat);
        if (state.ecat.error_code < 0) return -1;

        robif2b_kelo_drive_encoder_update(&drive_enc);
        robif2b_kelo_drive_actuator_update(&wheel_act);

        for (int i = 0; i < NUM_DRIVES; i++) {
            printf("drive [id=%i, conn=%i]: "
                    "w_vel[0]=%5.2f - w_vel[1]=%5.2f - p_pos=%5.2f\n",
                    i, state.ecat.is_connected[i + 1],
                    state.kelo_msr.whl_vel[i * 2 + 0],
                    state.kelo_msr.whl_vel[i * 2 + 1],
                    state.kelo_msr.pvt_pos[i]);
        }

        clock_gettime(CLOCK_MONOTONIC, &state.time.cycle_end);
        state.time.cycle_time_msr = timespec_to_usec(&state.time.cycle_end)
                                  - timespec_to_usec(&state.time.cycle_start);

        usleep(state.time.cycle_time_exp - state.time.cycle_time_msr);
    }

    robif2b_kelo_drive_actuator_stop(&wheel_act);
    robif2b_ethercat_stop(&ecat);
    robif2b_ethercat_shutdown(&ecat);

    return 0;
}
