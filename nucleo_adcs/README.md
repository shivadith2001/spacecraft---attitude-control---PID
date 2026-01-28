# ADCS Reaction Wheel Control – STM32 Nucleo

This folder is a standalone snapshot of the embedded ADCS + reaction wheel control stack for STM32 Nucleo boards. It mirrors the root `src/` implementation so you can initialize this directory as its own repository and drop it into a FreeRTOS project.

## Contents
- `src/adcs.h` / `src/adcs.c`: PID + attitude state update core with block-diagram stages.
- `src/adcs_task.c`: 100 Hz FreeRTOS task that reads BMX160 gyro data, fuses an adjacent attitude source, and publishes reaction wheel torque requests.
- `src/bmx160.h` / `src/bmx160.c`: BMX160 I2C driver for gyro Z-axis reads.
- `src/rw_command_map.h` / `src/rw_command_map.c`: Torque-to-speed mapping with quantization and saturation (1 Hz).
- `src/rw_controller.h` / `src/rw_controller.c`: 1 kHz PI speed controller for the reaction wheel.
- `src/rw_interface.h` / `src/rw_interface.c`: FreeRTOS-safe torque command handoff.
- `src/rw_task.c`: 1 kHz reaction wheel control task (encoder read + torque driver).
- `src/nucleo_board.h` / `src/nucleo_board.c`: Nucleo board initialization stubs (clock/GPIO/I2C/PWM).
- `src/rtos_tasks.h` / `src/rtos_tasks.c`: FreeRTOS task creation helpers for ADCS/RW.
- `src/main.c`: Minimal Nucleo entry point showing task creation.

## Paper-aligned control updates
- **Detumble → pointing modes**: start in detumble using a magnetorquer rate damper until |ω| falls below the configured threshold, then switch to reaction-wheel pointing.
- **Outer P+PI control law**: attitude PI with rate feedback (Kd * ω) and anti-windup handling when torque saturates.
- **Torque low-pass**: first-order discrete filter to model actuator torque bandwidth (τw).

## NUCLEO-F446RE wiring plan (default)
- **BMX160 IMU on I2C1**: PB8 (SCL), PB9 (SDA), 3.3V, GND, optional INT on PC4 (EXTI).
- **Reaction wheel driver (PWM + GPIO)**: PA6 (TIM3_CH1 PWM ~20 kHz), PB0 (DIR), PB1 (EN), optional FAULT on PC13.
- **Quadrature encoder**: PA0 (TIM2_CH1 A), PA1 (TIM2_CH2 B), optional index Z on PA2 (EXTI).

## CubeMX configuration checklist (NUCLEO-F446RE)
- SYSCLK ≈ 84 MHz (HSE or HSI).
- FreeRTOS (CMSIS-V2) enabled.
- I2C1 at 400 kHz on PB8/PB9.
- TIM3 PWM on CH1 (PA6), PWM frequency ≈ 20 kHz.
- TIM2 encoder interface mode on PA0/PA1 (32-bit counter).
- GPIO outputs: PB0/PB1; GPIO input: PC13 (optional).
- NVIC priorities for TIM2/TIM3 below `configMAX_SYSCALL_INTERRUPT_PRIORITY`.

## Using as a standalone repo
```bash
git init

git add .
git commit -m "Initial STM32 Nucleo ADCS reaction wheel stack"
```
Replace the TODO stubs with your Nucleo BSP/HAL calls and driver hooks.
