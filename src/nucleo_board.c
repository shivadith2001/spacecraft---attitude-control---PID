#include "nucleo_board.h"

void Nucleo_Board_Init(void) {
    /* TODO: Call HAL_Init() and initialize the STM32 Nucleo board support package. */
    Nucleo_Clock_Init();
    Nucleo_GPIO_Init();
    Nucleo_I2C1_Init();
    Nucleo_PWM_Init();
}

void Nucleo_Clock_Init(void) {
    /* TODO: Configure Nucleo system clocks (HSE/PLL) for the target MCU. */
}

void Nucleo_GPIO_Init(void) {
    /* TODO: Configure GPIOs for BMX160 chip select/INT and reaction wheel driver enable. */
}

void Nucleo_I2C1_Init(void) {
    /* TODO: Configure I2C1 peripheral for BMX160 communications. */
}

void Nucleo_PWM_Init(void) {
    /* TODO: Configure timers/PWM for reaction wheel motor driver. */
}
