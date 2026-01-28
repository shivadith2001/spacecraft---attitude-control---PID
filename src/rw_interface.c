#include "rw_interface.h"

#include "FreeRTOS.h"
#include "task.h"

static volatile float g_rw_torque_cmd_nm = 0.0f;
static volatile float g_rw_wheel_speed_rad_s = 0.0f;

void RW_Command_SetTorqueNm(float torque_nm) {
    taskENTER_CRITICAL();
    g_rw_torque_cmd_nm = torque_nm;
    taskEXIT_CRITICAL();
}

float RW_Command_GetTorqueNm(void) {
    float value;
    taskENTER_CRITICAL();
    value = g_rw_torque_cmd_nm;
    taskEXIT_CRITICAL();
    return value;
}

void RW_Command_SetWheelSpeedRadS(float omega_rad_s) {
    taskENTER_CRITICAL();
    g_rw_wheel_speed_rad_s = omega_rad_s;
    taskEXIT_CRITICAL();
}

float RW_Command_GetWheelSpeedRadS(void) {
    float value;
    taskENTER_CRITICAL();
    value = g_rw_wheel_speed_rad_s;
    taskEXIT_CRITICAL();
    return value;
}
