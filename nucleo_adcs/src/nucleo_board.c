#include "nucleo_board.h"

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void Nucleo_Board_Init(void) {
    HAL_Init();
    Nucleo_Clock_Init();
    Nucleo_GPIO_Init();
    Nucleo_I2C1_Init();
    Nucleo_PWM_Init();
    Nucleo_Encoder_Init();

    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void Nucleo_Clock_Init(void) {
    SystemClock_Config();
}

void Nucleo_GPIO_Init(void) {
    MX_GPIO_Init();
}

void Nucleo_I2C1_Init(void) {
    MX_I2C1_Init();
}

void Nucleo_PWM_Init(void) {
    MX_TIM3_Init();
}

void Nucleo_Encoder_Init(void) {
    MX_TIM2_Init();
}
