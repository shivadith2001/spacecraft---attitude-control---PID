#ifndef ADCS_H
#define ADCS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float integral_error;
    float previous_error;
    bool has_previous_error;
} pid_controller_t;

typedef struct {
    float gyro_rad_s;
    float angle_rad;
    float rate_rad_s;
    float absolute_angle_rad;
    bool has_absolute_angle;
} adcs_sensor_readings_t;

typedef struct {
    float angle_rad;
    float rate_rad_s;
} adcs_attitude_estimate_t;

typedef struct {
    float reaction_wheel_torque_nm;
    float magnetorquer_dipole_am2;
} adcs_actuator_cmd_t;

typedef struct {
    float internal_torque_nm;
    float external_torque_nm;
} adcs_disturbance_t;

typedef struct {
    float inertia_kg_m2;
    float angle_rad;
    float rate_rad_s;
} adcs_dynamics_state_t;

typedef enum {
    ADCS_MODE_DETUMBLE = 0,
    ADCS_MODE_POINTING = 1
} adcs_mode_t;

typedef struct {
    float target_angle_rad;
    adcs_attitude_estimate_t estimate;
    adcs_dynamics_state_t dynamics;
    float torque_filter_state_nm;
    float wheel_speed_rad_s;
    adcs_mode_t mode;
} adcs_state_t;

typedef struct {
    float dt_s;
    float max_reaction_wheel_torque_nm;
    float max_magnetorquer_dipole_am2;
    float complementary_filter_alpha;
    float torque_filter_tw_s;
    float detumble_rate_threshold_rad_s;
    float detumble_gain_am2_per_rad_s;
    bool use_simulated_dynamics;
} adcs_config_t;

typedef struct {
    float attitude_error_rad;
    float rate_rad_s;
    float torque_cmd_nm;
    float torque_applied_nm;
    float wheel_speed_rad_s;
    float integral_state;
    adcs_mode_t mode;
} adcs_telemetry_t;

typedef bool (*adcs_read_gyro_fn)(float *gyro_rad_s);
typedef bool (*adcs_read_absolute_angle_fn)(float *angle_rad);
typedef void (*adcs_write_reaction_wheel_fn)(float torque_nm);
typedef void (*adcs_write_magnetorquer_fn)(float dipole_am2);

void PID_Reset(pid_controller_t *pid);
float PID_Compute(pid_controller_t *pid, float error, float dt_s);

void ADCS_Init(adcs_state_t *state, pid_controller_t *pid, float inertia_kg_m2);
bool ADCS_ReadSensors(
    adcs_sensor_readings_t *sensors,
    adcs_read_gyro_fn read_gyro,
    adcs_read_absolute_angle_fn read_absolute_angle);
void ADCS_AttitudeDetermine(
    const adcs_config_t *config,
    const adcs_sensor_readings_t *sensors,
    adcs_attitude_estimate_t *estimate);
float ADCS_ControlLaw(
    const adcs_config_t *config,
    adcs_state_t *state,
    pid_controller_t *pid,
    const adcs_disturbance_t *disturbance,
    adcs_actuator_cmd_t *actuator_cmd,
    adcs_telemetry_t *telemetry);
void ADCS_ActuatorApply(
    const adcs_config_t *config,
    const adcs_actuator_cmd_t *actuator_cmd,
    adcs_write_reaction_wheel_fn write_rw,
    adcs_write_magnetorquer_fn write_mtq);
void ADCS_DynamicsUpdate(
    const adcs_config_t *config,
    adcs_dynamics_state_t *dynamics,
    float applied_torque_nm,
    const adcs_disturbance_t *disturbance);
bool ADCS_RunCycle(
    const adcs_config_t *config,
    adcs_state_t *state,
    pid_controller_t *pid,
    const adcs_disturbance_t *disturbance,
    adcs_read_gyro_fn read_gyro,
    adcs_read_absolute_angle_fn read_absolute_angle,
    adcs_write_reaction_wheel_fn write_rw,
    adcs_write_magnetorquer_fn write_mtq,
    adcs_actuator_cmd_t *actuator_cmd,
    adcs_telemetry_t *telemetry);

float ADCS_Clamp(float value, float min_value, float max_value);

#ifdef __cplusplus
}
#endif

#endif
