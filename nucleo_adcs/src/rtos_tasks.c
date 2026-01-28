#include "rtos_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

extern void ADCS_Task(void *argument);
extern void RW_Task(void *argument);

void ADCS_RTOS_Init(void) {
    xTaskCreate(ADCS_Task, "ADCS", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(RW_Task, "RW", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
}
