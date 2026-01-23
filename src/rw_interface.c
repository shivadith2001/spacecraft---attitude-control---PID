#include "rw_interface.h"

#include "FreeRTOS.h"
#include "task.h"

static volatile float g_rw_torque_cmd_nm = 0.0f;

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
