import math
import pathlib
import sys
import unittest

DEMOS = pathlib.Path(__file__).resolve().parents[1] / "demos"
sys.path.insert(0, str(DEMOS))

from neck_model import NeckModel, response_metrics, simulate_head_response


class NeckModelTest(unittest.TestCase):
    def test_zero_input_stays_at_rest(self):
        response = simulate_head_response([0.0] * 100, 0.01)
        metrics = response_metrics(response)
        self.assertEqual(metrics["peak_angle_deg"], 0.0)
        self.assertEqual(metrics["peak_angular_accel_deg_s2"], 0.0)

    def test_constant_lateral_acceleration_approaches_static_equilibrium(self):
        model = NeckModel()
        ay = 1.0
        response = simulate_head_response([ay] * 2000, 0.005, model)
        expected = (
            -model.head_mass_kg
            * model.com_lever_arm_m
            * ay
            / model.stiffness_nm_per_rad
        )
        self.assertAlmostEqual(response[-1]["theta_rad"], expected, places=3)

    def test_model_is_underdamped_by_default(self):
        self.assertGreater(NeckModel().damping_ratio, 0.0)
        self.assertLess(NeckModel().damping_ratio, 1.0)


if __name__ == "__main__":
    unittest.main()
