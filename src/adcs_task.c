#include "adcs.h"
#include "rw_interface.h"

#include "FreeRTOS.h"
#include "task.h"

#define ADCS_TASK_RATE_HZ 1U
#define ADCS_TASK_PERIOD_MS (1000U / ADCS_TASK_RATE_HZ)

static adcs_state_t g_adcs_state;
static pid_controller_t g_pid;
static const adcs_config_t g_adcs_config = {
    .dt_s = 1.0f / ADCS_TASK_RATE_HZ,
    .max_reaction_wheel_torque_nm = 0.001f,
    .max_magnetorquer_dipole_am2 = 0.15f,
    .complementary_filter_alpha = 0.98f,
    .use_simulated_dynamics = false,
};

static bool BMX160_ReadGyroRadS(float *gyro_rad_s) {
    /* TODO: Replace with BMX160 driver call using STM32 Nucleo I2C/SPI HAL. */
    if (gyro_rad_s == NULL) {
        return false;
    }
    *gyro_rad_s = 0.0f;
    return true;
}

static bool AdjacentControl_ReadAbsoluteAngle(float *angle_rad) {
    /* TODO: Replace with sun sensor / star tracker absolute attitude angle. */
    if (angle_rad == NULL) {
        return false;
    }
    return false;
}

static void Magnetorquer_SetDipole(float dipole_am2) {
    /* TODO: Replace with magnetorquer driver on Nucleo GPIO/PWM outputs. */
    (void)dipole_am2;
}

void ADCS_Task(void *argument) {
    (void)argument;

    ADCS_Init(&g_adcs_state, &g_pid, 55.0f);
    g_pid.kp = 0.8f;
    g_pid.ki = 0.03f;
    g_pid.kd = 0.25f;
    g_pid.integral_limit = 0.5f;

    g_adcs_state.target_angle_rad = 0.0f;

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        adcs_disturbance_t disturbance = {
            .internal_torque_nm = 0.0f,
            .external_torque_nm = 0.0f,
        };
        adcs_actuator_cmd_t actuator_cmd = {0};
        (void)ADCS_RunCycle(
            &g_adcs_config,
            &g_adcs_state,
            &g_pid,
            &disturbance,
            BMX160_ReadGyroRadS,
            AdjacentControl_ReadAbsoluteAngle,
            NULL,
            Magnetorquer_SetDipole,
            &actuator_cmd);

        RW_Command_SetTorqueNm(actuator_cmd.reaction_wheel_torque_nm);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(ADCS_TASK_PERIOD_MS));
    }
}
