#include "adcs.h"

#include <stddef.h>
#include <math.h>

void PID_Reset(pid_controller_t *pid) {
    if (pid == NULL) {
        return;
    }
    pid->integral_error = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous_error = false;
}

float PID_Compute(pid_controller_t *pid, float error, float dt_s) {
    if (pid == NULL || dt_s <= 0.0f) {
        return 0.0f;
    }

    pid->integral_error += error * dt_s;
    pid->integral_error = ADCS_Clamp(pid->integral_error, -pid->integral_limit, pid->integral_limit);

    float derivative = 0.0f;
    if (pid->has_previous_error) {
        derivative = (error - pid->previous_error) / dt_s;
    }

    pid->previous_error = error;
    pid->has_previous_error = true;

    return (pid->kp * error) + (pid->ki * pid->integral_error) + (pid->kd * derivative);
}

void ADCS_Init(adcs_state_t *state, pid_controller_t *pid, float inertia_kg_m2) {
    if (state == NULL) {
        return;
    }

    state->target_angle_rad = 0.0f;
    state->estimate.angle_rad = 0.0f;
    state->estimate.rate_rad_s = 0.0f;
    state->dynamics.inertia_kg_m2 = inertia_kg_m2;
    state->dynamics.angle_rad = 0.0f;
    state->dynamics.rate_rad_s = 0.0f;

    PID_Reset(pid);
}

bool ADCS_ReadSensors(
    adcs_sensor_readings_t *sensors,
    adcs_read_gyro_fn read_gyro,
    adcs_read_absolute_angle_fn read_absolute_angle) {
    if (sensors == NULL || read_gyro == NULL) {
        return false;
    }

    sensors->gyro_rad_s = 0.0f;
    sensors->absolute_angle_rad = 0.0f;
    sensors->has_absolute_angle = false;

    if (!read_gyro(&sensors->gyro_rad_s)) {
        return false;
    }

    if (read_absolute_angle != NULL) {
        float angle_rad = 0.0f;
        if (read_absolute_angle(&angle_rad)) {
            sensors->absolute_angle_rad = angle_rad;
            sensors->has_absolute_angle = true;
        }
    }

    return true;
}

void ADCS_AttitudeDetermine(
    const adcs_config_t *config,
    const adcs_sensor_readings_t *sensors,
    adcs_attitude_estimate_t *estimate) {
    if (config == NULL || sensors == NULL || estimate == NULL || config->dt_s <= 0.0f) {
        return;
    }

    estimate->rate_rad_s = sensors->gyro_rad_s;
    float gyro_angle = estimate->angle_rad + (sensors->gyro_rad_s * config->dt_s);

    if (sensors->has_absolute_angle) {
        float alpha = ADCS_Clamp(config->complementary_filter_alpha, 0.0f, 1.0f);
        estimate->angle_rad = (alpha * gyro_angle) + ((1.0f - alpha) * sensors->absolute_angle_rad);
    } else {
        estimate->angle_rad = gyro_angle;
    }
}

float ADCS_ControlLaw(
    const adcs_config_t *config,
    adcs_state_t *state,
    pid_controller_t *pid,
    const adcs_disturbance_t *disturbance,
    adcs_actuator_cmd_t *actuator_cmd) {
    if (config == NULL || state == NULL || pid == NULL || actuator_cmd == NULL || config->dt_s <= 0.0f) {
        return 0.0f;
    }

    float error = state->target_angle_rad - state->estimate.angle_rad;
    float reaction_wheel_torque = PID_Compute(pid, error, config->dt_s);

    float total_disturbance = 0.0f;
    if (disturbance != NULL) {
        total_disturbance = disturbance->internal_torque_nm + disturbance->external_torque_nm;
    }

    reaction_wheel_torque = ADCS_Clamp(
        reaction_wheel_torque,
        -config->max_reaction_wheel_torque_nm,
        config->max_reaction_wheel_torque_nm);

    actuator_cmd->reaction_wheel_torque_nm = reaction_wheel_torque;
    actuator_cmd->magnetorquer_dipole_am2 = ADCS_Clamp(
        -total_disturbance,
        -config->max_magnetorquer_dipole_am2,
        config->max_magnetorquer_dipole_am2);

    return reaction_wheel_torque;
}

void ADCS_ActuatorApply(
    const adcs_config_t *config,
    const adcs_actuator_cmd_t *actuator_cmd,
    adcs_write_reaction_wheel_fn write_rw,
    adcs_write_magnetorquer_fn write_mtq) {
    if (config == NULL || actuator_cmd == NULL) {
        return;
    }

    if (write_rw != NULL) {
        write_rw(ADCS_Clamp(
            actuator_cmd->reaction_wheel_torque_nm,
            -config->max_reaction_wheel_torque_nm,
            config->max_reaction_wheel_torque_nm));
    }

    if (write_mtq != NULL) {
        write_mtq(ADCS_Clamp(
            actuator_cmd->magnetorquer_dipole_am2,
            -config->max_magnetorquer_dipole_am2,
            config->max_magnetorquer_dipole_am2));
    }
}

void ADCS_DynamicsUpdate(
    const adcs_config_t *config,
    adcs_dynamics_state_t *dynamics,
    float applied_torque_nm,
    const adcs_disturbance_t *disturbance) {
    if (config == NULL || dynamics == NULL || config->dt_s <= 0.0f) {
        return;
    }

    float total_disturbance = 0.0f;
    if (disturbance != NULL) {
        total_disturbance = disturbance->internal_torque_nm + disturbance->external_torque_nm;
    }

    float net_torque = applied_torque_nm + total_disturbance;
    float angular_accel = 0.0f;
    if (dynamics->inertia_kg_m2 > 0.0f) {
        angular_accel = net_torque / dynamics->inertia_kg_m2;
    }

    dynamics->rate_rad_s += angular_accel * config->dt_s;
    dynamics->angle_rad += dynamics->rate_rad_s * config->dt_s;
}

bool ADCS_RunCycle(
    const adcs_config_t *config,
    adcs_state_t *state,
    pid_controller_t *pid,
    const adcs_disturbance_t *disturbance,
    adcs_read_gyro_fn read_gyro,
    adcs_read_absolute_angle_fn read_absolute_angle,
    adcs_write_reaction_wheel_fn write_rw,
    adcs_write_magnetorquer_fn write_mtq,
    adcs_actuator_cmd_t *actuator_cmd) {
    if (config == NULL || state == NULL || pid == NULL || actuator_cmd == NULL) {
        return false;
    }

    adcs_sensor_readings_t sensors;
    if (!ADCS_ReadSensors(&sensors, read_gyro, read_absolute_angle)) {
        return false;
    }

    ADCS_AttitudeDetermine(config, &sensors, &state->estimate);
    float applied_torque = ADCS_ControlLaw(config, state, pid, disturbance, actuator_cmd);
    ADCS_ActuatorApply(config, actuator_cmd, write_rw, write_mtq);

    if (config->use_simulated_dynamics) {
        ADCS_DynamicsUpdate(config, &state->dynamics, applied_torque, disturbance);
    }

    return true;
}

float ADCS_Clamp(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}
