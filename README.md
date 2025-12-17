# Spacecraft Attitude Control – PID Demo

This repository provides a lightweight demonstration of a single-axis Attitude and Orbit Control System (AOCS) using a PID controller. Run the simulation to see a spacecraft rotate from its current orientation to a commanded attitude while logging telemetry to CSV.

## Requirements
- Python 3.10+

## Quick start
```bash
python src/aocs_demo.py --duration 20 --target 60
```
The script prints a short performance summary and writes telemetry (time, angle, rate, error, and torque command) to `telemetry.csv`.

## How it works
- **Dynamics:** A rigid body with a scalar moment of inertia integrates applied torque into angular rate and angle. You can set initial attitude and body rate with `--initial-angle` and `--initial-rate`.
- **Controller:** A PID loop with integral windup protection and derivative term based on the previous error generates torque commands to drive attitude error toward zero. Negative or zero time steps are rejected to avoid invalid physics.
- **Metrics:** The simulation reports final angle/rate, peak rate, peak tracking error, and an approximate settling time threshold (|error| < 0.5°).

Tune the controller gains or simulation parameters in `src/aocs_demo.py` to explore different behaviors. Telemetry is written to CSV for plotting or further analysis.
