# Spacecraft Attitude Control – PID Demo

This repository provides a lightweight demonstration of a single-axis Attitude and Orbit Control System (AOCS) using a PID controller. Run the simulation to see a spacecraft rotate from its current orientation to a commanded attitude while logging telemetry to CSV.

## Requirements
- Python 3.10+

## Quick start
```bash
python src/aocs_demo.py --duration 20 --target 60
```
The script prints a short performance summary and writes telemetry (time, angle, rate, error, and torque command) to `telemetry.csv`.

## Testing
See `TESTING.md` for instructions on running the automated edge-case tests and the list of covered scenarios.

## How it works
- **Dynamics:** A rigid body with a scalar moment of inertia integrates applied torque into angular rate and angle. You can set initial attitude and body rate with `--initial-angle` and `--initial-rate`.
- **Controller:** A PID loop with integral windup protection and derivative term based on the previous error generates torque commands to drive attitude error toward zero. Negative or zero time steps are rejected to avoid invalid physics.
- **Metrics:** The simulation reports final angle/rate, peak rate, peak tracking error, and an approximate settling time threshold (|error| < 0.5°).

Tune the controller gains or simulation parameters in `src/aocs_demo.py` to explore different behaviors. Telemetry is written to CSV for plotting or further analysis.

## Embedded ADCS starter (STM32 Nucleo + BMX160)
This repo also includes a C starter implementation suitable for integrating a single-axis ADCS loop into a FreeRTOS task list on STM32 Nucleo boards. The files are:
- `src/adcs.h` / `src/adcs.c`: PID + attitude state update core.
- `src/adcs_task.c`: 100 Hz FreeRTOS task that reads BMX160 gyro data, blends adjacent actuator torque, and publishes a reaction wheel torque request.
- `src/bmx160.h` / `src/bmx160.c`: BMX160 I2C driver for gyro Z-axis reads.
- `src/rw_command_map.h` / `src/rw_command_map.c`: Torque-to-speed setpoint mapping with quantization and saturation (1 Hz).
- `src/rw_controller.h` / `src/rw_controller.c`: Inner-loop PI speed controller for the reaction wheel (1 kHz).
- `src/rw_interface.h` / `src/rw_interface.c`: FreeRTOS-safe torque command handoff between tasks.
- `src/rw_task.c`: 1 kHz reaction wheel task that converts torque requests into speed setpoints and driver commands.
- `src/nucleo_board.h` / `src/nucleo_board.c`: Nucleo board init stubs (clock/GPIO/I2C/PWM).
- `src/rtos_tasks.h` / `src/rtos_tasks.c`: FreeRTOS task creation helpers for ADCS/RW.
- `src/main.c`: Minimal STM32 Nucleo entry point showing task creation.

The implementation mirrors the block diagram flow (sensors → attitude determination → control laws → actuators → dynamics with disturbance torques) through `ADCS_RunCycle`, which wires together `ADCS_ReadSensors`, `ADCS_AttitudeDetermine`, `ADCS_ControlLaw`, `ADCS_ActuatorApply`, and optional `ADCS_DynamicsUpdate`. Reaction wheel torque requests are mapped to wheel speed setpoints at 1 Hz and regulated by a 1 kHz PI controller per the thesis brief. Replace the TODO stubs with your Nucleo board support package (HAL) and device drivers, then tune gains/inertia for your hardware.

### Paper-aligned control updates
- **Detumble → pointing modes**: start in detumble using a magnetorquer rate damper until |ω| falls below the configured threshold, then switch to reaction-wheel pointing.
- **Outer P+PI control law**: attitude PI with rate feedback (Kd * ω) and anti-windup handling when torque saturates.
- **Torque low-pass**: first-order discrete filter to model actuator torque bandwidth (τw).

### NUCLEO-F446RE wiring plan (default)
- **BMX160 IMU on I2C1**: PB8 (SCL), PB9 (SDA), 3.3V, GND, optional INT on PC4 (EXTI).
- **Reaction wheel driver (PWM + GPIO)**: PA6 (TIM3_CH1 PWM ~20 kHz), PB0 (DIR), PB1 (EN), optional FAULT on PC13.
- **Quadrature encoder**: PA0 (TIM2_CH1 A), PA1 (TIM2_CH2 B), optional index Z on PA2 (EXTI).

### CubeMX configuration checklist (NUCLEO-F446RE)
- SYSCLK ≈ 84 MHz (HSE or HSI).
- FreeRTOS (CMSIS-V2) enabled.
- I2C1 at 400 kHz on PB8/PB9.
- TIM3 PWM on CH1 (PA6), PWM frequency ≈ 20 kHz.
- TIM2 encoder interface mode on PA0/PA1 (32-bit counter).
- GPIO outputs: PB0/PB1; GPIO input: PC13 (optional).
- NVIC priorities for TIM2/TIM3 below `configMAX_SYSCALL_INTERRUPT_PRIORITY`.

## Nucleo reaction wheel repo snapshot
If you want a separate repo layout dedicated to STM32 Nucleo reaction wheel control, a full copy of the embedded sources lives under `nucleo_adcs/`. You can initialize it as its own Git repository and integrate directly into your FreeRTOS project.
