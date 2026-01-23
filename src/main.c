#include "nucleo_board.h"

#include "FreeRTOS.h"
#include "task.h"

extern void ADCS_Task(void *argument);
extern void RW_Task(void *argument);

int main(void) {
    Nucleo_Board_Init();

    xTaskCreate(ADCS_Task, "ADCS", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(RW_Task, "RW", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
    vTaskStartScheduler();

    while (1) {
    }
}
