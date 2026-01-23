#include "rw_command_map.h"

#include <math.h>
#include <stddef.h>

bool RW_CommandMap_Update(
    rw_command_map_t *map,
    float torque_cmd_nm,
    float dt_s,
    float *omega_set_rad_s) {
    if (map == NULL || omega_set_rad_s == NULL || dt_s <= 0.0f) {
        return false;
    }

    if (map->wheel_inertia_kg_m2 <= 0.0f) {
        return false;
    }

    float delta_momentum = torque_cmd_nm * dt_s;
    float delta_omega = delta_momentum / map->wheel_inertia_kg_m2;

    float omega_set = map->omega_set_rad_s + delta_omega;
    omega_set = RW_CommandMap_Clamp(omega_set, -map->max_speed_rad_s, map->max_speed_rad_s);

    if (map->speed_bit_rad_s > 0.0f) {
        float quantized = nearbyintf(omega_set / map->speed_bit_rad_s) * map->speed_bit_rad_s;
        omega_set = RW_CommandMap_Clamp(quantized, -map->max_speed_rad_s, map->max_speed_rad_s);
    }

    map->omega_set_rad_s = omega_set;
    map->stored_momentum_nms = map->wheel_inertia_kg_m2 * omega_set;
    *omega_set_rad_s = omega_set;

    return true;
}

float RW_CommandMap_Clamp(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}
