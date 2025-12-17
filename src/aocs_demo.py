"""Simple single-axis AOCS demo with PID attitude control.

Run the module to simulate a spacecraft rotating to a commanded attitude.
Results are printed to the console and also written to CSV for inspection.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import math
from typing import Iterable, List, Sequence


@dataclasses.dataclass
class PIDController:
    """Lightweight PID controller for torque command.

    The controller uses a simple integral windup guard and forward difference
    for the derivative term. All inputs and outputs are scalar values
    representing one rotational axis.
    """

    kp: float
    ki: float
    kd: float
    integral_limit: float = 1.0
    _integral_error: float = 0.0
    _previous_error: float | None = None

    def reset(self) -> None:
        """Clear stored error state for reuse between runs."""
        self._integral_error = 0.0
        self._previous_error = None

    def compute(self, error: float, dt: float) -> float:
        """Return the control torque for the current error."""
        if dt <= 0:
            raise ValueError("dt must be positive for PID computation")

        self._integral_error += error * dt
        self._integral_error = max(min(self._integral_error, self.integral_limit), -self.integral_limit)

        derivative = 0.0 if self._previous_error is None else (error - self._previous_error) / dt
        self._previous_error = error

        return (self.kp * error) + (self.ki * self._integral_error) + (self.kd * derivative)


@dataclasses.dataclass
class Spacecraft:
    """Single-axis rigid body with simple rotational dynamics."""

    inertia: float  # kg*m^2
    angle: float = 0.0  # rad
    rate: float = 0.0  # rad/s

    def apply_torque(self, torque: float, dt: float) -> None:
        """Integrate state forward one time step."""
        if dt <= 0:
            raise ValueError("dt must be positive for integration")

        angular_acceleration = torque / self.inertia
        self.rate += angular_acceleration * dt
        self.angle += self.rate * dt


@dataclasses.dataclass
class SimulationConfig:
    duration: float = 30.0
    dt: float = 0.1
    target_angle_deg: float = 45.0
    initial_angle_deg: float = 0.0
    initial_rate_deg_s: float = 0.0

    @property
    def target_angle_rad(self) -> float:
        return math.radians(self.target_angle_deg)

    @property
    def initial_angle_rad(self) -> float:
        return math.radians(self.initial_angle_deg)

    @property
    def initial_rate_rad_s(self) -> float:
        return math.radians(self.initial_rate_deg_s)

    @property
    def steps(self) -> int:
        return int(self.duration / self.dt)


def simulate_attitude_control(config: SimulationConfig) -> List[dict[str, float]]:
    """Run the attitude control simulation.

    Returns a list of dicts with per-step telemetry for easy serialization.
    """
    if config.dt <= 0:
        raise ValueError("dt must be positive")
    if config.duration <= 0:
        raise ValueError("duration must be positive")

    spacecraft = Spacecraft(
        inertia=55.0,
        angle=config.initial_angle_rad,
        rate=config.initial_rate_rad_s,
    )  # Representative for a small satellite
    pid = PIDController(kp=0.8, ki=0.03, kd=0.25, integral_limit=0.5)
    pid.reset()

    telemetry: List[dict[str, float]] = []
    for step in range(config.steps + 1):
        time = step * config.dt
        error = config.target_angle_rad - spacecraft.angle
        torque_command = pid.compute(error, config.dt)
        spacecraft.apply_torque(torque_command, config.dt)

        telemetry.append(
            {
                "time": time,
                "angle_deg": math.degrees(spacecraft.angle),
                "rate_deg_s": math.degrees(spacecraft.rate),
                "error_deg": math.degrees(error),
                "torque_command": torque_command,
            }
        )

    return telemetry


def _write_csv(rows: Iterable[dict[str, float]], path: str) -> None:
    fieldnames: Sequence[str] = ["time", "angle_deg", "rate_deg_s", "error_deg", "torque_command"]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _print_summary(telemetry: Sequence[dict[str, float]], target_angle_deg: float) -> None:
    if not telemetry:
        return

    final = telemetry[-1]
    peak_error = max(abs(entry["error_deg"]) for entry in telemetry)
    peak_rate = max(abs(entry["rate_deg_s"]) for entry in telemetry)
    settling_threshold = 0.5  # degrees
    settled_at = next(
        (entry["time"] for entry in telemetry if abs(entry["error_deg"]) < settling_threshold),
        None,
    )

    print("Attitude control demo complete:\n")
    print(f"  Commanded angle: {target_angle_deg:.2f} deg")
    print(f"  Final angle:     {final['angle_deg']:.2f} deg")
    print(f"  Final rate:      {final['rate_deg_s']:.3f} deg/s")
    print(f"  Peak rate:       {peak_rate:.3f} deg/s")
    print(f"  Peak error:      {peak_error:.2f} deg")
    if settled_at is None:
        print("  Settling time:   Not settled within simulation window")
    else:
        print(f"  Settling time:   {settled_at:.2f} s (|error| < 0.5 deg)")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a single-axis AOCS PID demo")
    parser.add_argument("--duration", type=float, default=30.0, help="Simulation duration in seconds")
    parser.add_argument("--dt", type=float, default=0.1, help="Simulation time step in seconds")
    parser.add_argument("--target", type=float, default=45.0, help="Commanded angle in degrees")
    parser.add_argument("--initial-angle", type=float, default=0.0, help="Initial angle in degrees")
    parser.add_argument("--initial-rate", type=float, default=0.0, help="Initial body rate in deg/s")
    parser.add_argument(
        "--output",
        default="telemetry.csv",
        help="CSV file to write telemetry (time, angles, rates, torque)",
    )
    args = parser.parse_args()

    config = SimulationConfig(
        duration=args.duration,
        dt=args.dt,
        target_angle_deg=args.target,
        initial_angle_deg=args.initial_angle,
        initial_rate_deg_s=args.initial_rate,
    )
    telemetry = simulate_attitude_control(config)
    _write_csv(telemetry, args.output)
    _print_summary(telemetry, target_angle_deg=args.target)
    print(f"\nTelemetry written to {args.output}")


if __name__ == "__main__":  # pragma: no cover
    main()
