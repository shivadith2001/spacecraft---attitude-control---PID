#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);

extern void ADCS_Task(void *argument);
extern void RW_Task(void *argument);

int main(void) {
    /* HAL_Init(); */
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    xTaskCreate(ADCS_Task, "ADCS", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(RW_Task, "RW", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
    vTaskStartScheduler();

    while (1) {
    }
}

void SystemClock_Config(void) {
    /* TODO: Configure STM32H7 clocks. */
}

void MX_GPIO_Init(void) {
    /* TODO: Configure GPIOs used for BMX160 and reaction wheel drivers. */
}

void MX_I2C1_Init(void) {
    /* TODO: Configure I2C for BMX160. */
}
