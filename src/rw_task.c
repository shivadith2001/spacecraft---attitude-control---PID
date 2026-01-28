#include "rw_command_map.h"
#include "rw_controller.h"
#include "rw_interface.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx_hal.h"

#include <math.h>
#include <stdint.h>

#define RW_CTRL_RATE_HZ 1000U
#define RW_CTRL_PERIOD_MS (1000U / RW_CTRL_RATE_HZ)
#define RW_CMD_RATE_HZ 100U

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
    .duty_limit = 1.0f,
};

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#define RW_ENCODER_PPR 1024.0f
#define RW_ENCODER_DT_S (1.0f / RW_CTRL_RATE_HZ)
#define RW_OMEGA_LPF_ALPHA 0.2f

static uint32_t g_last_encoder_cnt = 0U;
static float g_omega_filt = 0.0f;

static int32_t RW_Encoder_Delta(uint32_t current, uint32_t previous) {
    uint32_t diff = current - previous;
    if (diff > (UINT32_MAX / 2U)) {
        return -((int32_t)(UINT32_MAX - diff + 1U));
    }
    return (int32_t)diff;
}

static bool RW_Encoder_ReadRadS(float *omega_rad_s) {
    if (omega_rad_s == NULL) {
        return false;
    }

    uint32_t count = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t delta = RW_Encoder_Delta(count, g_last_encoder_cnt);
    g_last_encoder_cnt = count;

    float revolutions = (float)delta / RW_ENCODER_PPR;
    float omega_raw = revolutions * (2.0f * 3.1415926f) / RW_ENCODER_DT_S;
    g_omega_filt = (RW_OMEGA_LPF_ALPHA * omega_raw) + ((1.0f - RW_OMEGA_LPF_ALPHA) * g_omega_filt);
    *omega_rad_s = g_omega_filt;
    return true;
}

static void RW_Driver_SetDuty(float duty) {
    if (duty > 1.0f) {
        duty = 1.0f;
    } else if (duty < -1.0f) {
        duty = -1.0f;
    }

    if (duty >= 0.0f) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }

    float magnitude = fabsf(duty);
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;
    uint32_t ccr = (uint32_t)(magnitude * (float)period);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
}

static void RW_Driver_Enable(bool enable) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
            RW_Command_SetWheelSpeedRadS(omega_meas);
            float duty_out = RW_Controller_Update(
                &g_rw_controller,
                omega_set,
                omega_meas,
                1.0f / RW_CTRL_RATE_HZ);
            RW_Driver_SetDuty(duty_out);
        } else {
            RW_Driver_Enable(false);
            RW_Command_SetWheelSpeedRadS(0.0f);
            RW_Driver_SetDuty(0.0f);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(RW_CTRL_PERIOD_MS));
    }
}
