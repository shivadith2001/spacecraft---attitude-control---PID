import math
import unittest

from src import aocs_demo


class TestAocsDemoEdgeCases(unittest.TestCase):
    def test_pid_compute_rejects_nonpositive_dt(self) -> None:
        controller = aocs_demo.PIDController(kp=1.0, ki=0.1, kd=0.01, integral_limit=0.5)
        with self.assertRaises(ValueError):
            controller.compute(error=1.0, dt=0.0)

    def test_spacecraft_apply_torque_rejects_nonpositive_dt(self) -> None:
        spacecraft = aocs_demo.Spacecraft(inertia=55.0)
        with self.assertRaises(ValueError):
            spacecraft.apply_torque(torque=0.1, dt=-0.1)

    def test_simulation_rejects_nonpositive_dt(self) -> None:
        config = aocs_demo.SimulationConfig(dt=0.0)
        with self.assertRaises(ValueError):
            aocs_demo.simulate_attitude_control(config)

    def test_simulation_rejects_nonpositive_duration(self) -> None:
        config = aocs_demo.SimulationConfig(duration=-1.0)
        with self.assertRaises(ValueError):
            aocs_demo.simulate_attitude_control(config)

    def test_simulation_respects_step_count(self) -> None:
        config = aocs_demo.SimulationConfig(duration=1.0, dt=0.25)
        telemetry = aocs_demo.simulate_attitude_control(config)
        expected_steps = config.steps + 1
        self.assertEqual(len(telemetry), expected_steps)

    def test_simulation_handles_large_target_angle(self) -> None:
        config = aocs_demo.SimulationConfig(duration=2.0, dt=0.1, target_angle_deg=180.0)
        telemetry = aocs_demo.simulate_attitude_control(config)
        final_angle = telemetry[-1]["angle_deg"]
        self.assertTrue(math.isfinite(final_angle))


if __name__ == "__main__":
    unittest.main()
