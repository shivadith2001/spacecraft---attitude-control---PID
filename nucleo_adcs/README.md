# ADCS Reaction Wheel Control – STM32 Nucleo

This folder is a standalone snapshot of the embedded ADCS + reaction wheel control stack for STM32 Nucleo boards. It mirrors the root `src/` implementation so you can initialize this directory as its own repository and drop it into a FreeRTOS project.

## Contents
- `src/adcs.h` / `src/adcs.c`: PID + attitude state update core with block-diagram stages.
- `src/adcs_task.c`: 1 Hz FreeRTOS task that reads BMX160 gyro data, fuses an adjacent attitude source, and publishes reaction wheel torque requests.
- `src/rw_command_map.h` / `src/rw_command_map.c`: Torque-to-speed mapping with quantization and saturation (1 Hz).
- `src/rw_controller.h` / `src/rw_controller.c`: 1 kHz PI speed controller for the reaction wheel.
- `src/rw_interface.h` / `src/rw_interface.c`: FreeRTOS-safe torque command handoff.
- `src/rw_task.c`: 1 kHz reaction wheel control task (encoder read + torque driver).
- `src/nucleo_board.h` / `src/nucleo_board.c`: Nucleo board initialization stubs (clock/GPIO/I2C/PWM).
- `src/main.c`: Minimal Nucleo entry point showing task creation.

## Using as a standalone repo
```bash
git init

git add .
git commit -m "Initial STM32 Nucleo ADCS reaction wheel stack"
```
Replace the TODO stubs with your Nucleo BSP/HAL calls and driver hooks.
