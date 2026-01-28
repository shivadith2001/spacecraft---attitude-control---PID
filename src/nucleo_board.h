#ifndef NUCLEO_BOARD_H
#define NUCLEO_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

void Nucleo_Board_Init(void);
void Nucleo_Clock_Init(void);
void Nucleo_GPIO_Init(void);
void Nucleo_I2C1_Init(void);
void Nucleo_PWM_Init(void);
void Nucleo_Encoder_Init(void);

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);

#ifdef __cplusplus
}
#endif

#endif
