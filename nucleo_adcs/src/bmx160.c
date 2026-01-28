#include "bmx160.h"

#include "stm32f4xx_hal.h"

#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define BMX160_I2C_ADDR (0x68U << 1)
#define BMX160_REG_CHIP_ID 0x00U
#define BMX160_REG_GYRO_DATA 0x0CU
#define BMX160_REG_CMD 0x7EU

#define BMX160_CMD_GYRO_NORMAL 0x15U

#define BMX160_GYRO_LSB_PER_DPS 16.4f
#define BMX160_DPS_TO_RAD_S (3.1415926f / 180.0f)

static bool BMX160_ReadRegister(uint8_t reg, uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U) {
        return false;
    }
    return HAL_I2C_Mem_Read(&hi2c1, BMX160_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100U) == HAL_OK;
}

static bool BMX160_WriteRegister(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, BMX160_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) == HAL_OK;
}

bool BMX160_Init(void) {
    uint8_t chip_id = 0U;
    if (!BMX160_ReadRegister(BMX160_REG_CHIP_ID, &chip_id, 1U)) {
        return false;
    }
    if (chip_id != 0xD8U) {
        return false;
    }
    if (!BMX160_WriteRegister(BMX160_REG_CMD, BMX160_CMD_GYRO_NORMAL)) {
        return false;
    }
    HAL_Delay(50U);
    return true;
}

bool BMX160_ReadGyroZRadS(float *gyro_z_rad_s) {
    if (gyro_z_rad_s == NULL) {
        return false;
    }

    uint8_t raw[6] = {0};
    if (!BMX160_ReadRegister(BMX160_REG_GYRO_DATA, raw, sizeof(raw))) {
        return false;
    }

    int16_t gyro_z_raw = (int16_t)((uint16_t)raw[5] << 8U | raw[4]);
    float gyro_dps = (float)gyro_z_raw / BMX160_GYRO_LSB_PER_DPS;
    *gyro_z_rad_s = gyro_dps * BMX160_DPS_TO_RAD_S;
    return true;
}
