#ifndef RW_COMMAND_MAP_H
#define RW_COMMAND_MAP_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float wheel_inertia_kg_m2;
    float max_speed_rad_s;
    float speed_bit_rad_s;
    float omega_set_rad_s;
    float stored_momentum_nms;
} rw_command_map_t;

bool RW_CommandMap_Update(
    rw_command_map_t *map,
    float torque_cmd_nm,
    float dt_s,
    float *omega_set_rad_s);

float RW_CommandMap_Clamp(float value, float min_value, float max_value);

#ifdef __cplusplus
}
#endif

#endif
