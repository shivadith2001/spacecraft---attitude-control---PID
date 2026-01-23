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

#ifdef __cplusplus
}
#endif

#endif
