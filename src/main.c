#include "nucleo_board.h"
#include "rtos_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

int main(void) {
    Nucleo_Board_Init();

    ADCS_RTOS_Init();
    vTaskStartScheduler();

    while (1) {
    }
}
