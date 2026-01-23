#include "rw_controller.h"

#include <stddef.h>

static float RW_Controller_Clamp(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void RW_Controller_Reset(rw_pi_controller_t *controller) {
    if (controller == NULL) {
        return;
    }
    controller->integral = 0.0f;
}

float RW_Controller_Update(
    rw_pi_controller_t *controller,
    float omega_set_rad_s,
    float omega_meas_rad_s,
    float dt_s) {
    if (controller == NULL || dt_s <= 0.0f) {
        return 0.0f;
    }

    float error = omega_set_rad_s - omega_meas_rad_s;
    controller->integral += error * dt_s;
    controller->integral = RW_Controller_Clamp(
        controller->integral,
        -controller->integral_limit,
        controller->integral_limit);

    float torque_cmd = (controller->kp * error) + (controller->ki * controller->integral);
    return RW_Controller_Clamp(torque_cmd, -controller->torque_limit_nm, controller->torque_limit_nm);
}
