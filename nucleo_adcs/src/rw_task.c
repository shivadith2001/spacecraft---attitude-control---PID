#include "rw_command_map.h"
#include "rw_controller.h"
#include "rw_interface.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

#define RW_CTRL_RATE_HZ 1000U
#define RW_CTRL_PERIOD_MS (1000U / RW_CTRL_RATE_HZ)
#define RW_CMD_RATE_HZ 1U

static rw_command_map_t g_rw_cmd_map = {
    .wheel_inertia_kg_m2 = 9.35e-7f,
    .max_speed_rad_s = 6000.0f * (2.0f * 3.1415926f / 60.0f),
    .speed_bit_rad_s = 0.25f,
    .omega_set_rad_s = 0.0f,
    .stored_momentum_nms = 0.0f,
};

static rw_pi_controller_t g_rw_controller = {
    .kp = 0.015f,
    .ki = 0.002f,
    .integral = 0.0f,
    .integral_limit = 1.0f,
    .torque_limit_nm = 0.001f,
};

static bool RW_Encoder_ReadRadS(float *omega_rad_s) {
    /* TODO: Replace with encoder or hall sensor read on STM32 Nucleo timers. */
    if (omega_rad_s == NULL) {
        return false;
    }
    *omega_rad_s = 0.0f;
    return true;
}

static void RW_Driver_SetTorqueNm(float torque_nm) {
    /* TODO: Replace with PWM/FOC driver output using Nucleo timer channels. */
    (void)torque_nm;
}

static void RW_Driver_Enable(bool enable) {
    /* TODO: Replace with driver enable pin or fault handling on Nucleo. */
    (void)enable;
}

void RW_Task(void *argument) {
    (void)argument;

    RW_Controller_Reset(&g_rw_controller);
    RW_Driver_Enable(true);

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t cmd_divider = 0U;
    float omega_set = g_rw_cmd_map.omega_set_rad_s;

    for (;;) {
        if (cmd_divider == 0U) {
            float torque_cmd = RW_Command_GetTorqueNm();
            (void)RW_CommandMap_Update(&g_rw_cmd_map, torque_cmd, 1.0f / RW_CMD_RATE_HZ, &omega_set);
        }
        cmd_divider = (cmd_divider + 1U) % (RW_CTRL_RATE_HZ / RW_CMD_RATE_HZ);

        float omega_meas = 0.0f;
        if (RW_Encoder_ReadRadS(&omega_meas)) {
            float torque_out = RW_Controller_Update(
                &g_rw_controller,
                omega_set,
                omega_meas,
                1.0f / RW_CTRL_RATE_HZ);
            RW_Driver_SetTorqueNm(torque_out);
        } else {
            RW_Driver_Enable(false);
            RW_Driver_SetTorqueNm(0.0f);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(RW_CTRL_PERIOD_MS));
    }
}
