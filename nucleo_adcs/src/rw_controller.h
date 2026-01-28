#ifndef RW_CONTROLLER_H
#define RW_CONTROLLER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float integral;
    float integral_limit;
    float duty_limit;
} rw_pi_controller_t;

void RW_Controller_Reset(rw_pi_controller_t *controller);
float RW_Controller_Update(
    rw_pi_controller_t *controller,
    float omega_set_rad_s,
    float omega_meas_rad_s,
    float dt_s);

#ifdef __cplusplus
}
#endif

#endif
