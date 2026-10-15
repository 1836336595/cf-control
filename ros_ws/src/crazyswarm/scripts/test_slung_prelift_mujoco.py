"""Optional 70 Hz regression of the real controller with sampled, delayed I/O.

Run in the MuJoCo/Udaan environment: python test_slung_prelift_mujoco.py
"""

import tempfile
import unittest
from pathlib import Path

import yaml

try:
    import mujoco
except ImportError:
    mujoco = None

from simulate_slung_handoff import run_simulation


@unittest.skipIf(mujoco is None, "MuJoCo is not installed in this Python environment")
class SlungPreliftMujocoTest(unittest.TestCase):
    def test_real_pipeline_aborts_before_vehicle_collision_when_height_is_lost(self):
        with tempfile.TemporaryDirectory() as directory:
            candidate = yaml.safe_load((Path(__file__).parent / "simulation_configs" /
                                        "slung_70hz_candidate.yaml").read_text())
            result = run_simulation(directory, candidate, pose_delay_s=0.02, command_delay_s=0.01)
        self.assertEqual(result["failure"], "emergency_landing", result)
        self.assertIn(result["payload_safety_reason"],
                      ("height_drop", "distance", "rate_saturation"), result)
        self.assertGreater(result["minimum_vehicle_distance_m"], 0.20)
        self.assertGreaterEqual(result["minimum_clearance_m"], 0.15)
        self.assertLess(result["maximum_prelift_xy_error_m"], 0.25)
        self.assertEqual(result["phases"][-1], "emergency_landing")


if __name__ == "__main__":
    unittest.main()
