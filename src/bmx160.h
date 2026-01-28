#ifndef BMX160_H
#define BMX160_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool BMX160_Init(void);
bool BMX160_ReadGyroZRadS(float *gyro_z_rad_s);

#ifdef __cplusplus
}
#endif

#endif
