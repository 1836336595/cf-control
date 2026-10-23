"""Optional 70 Hz regression of the real controller with sampled, delayed I/O.

Run in the MuJoCo/Udaan environment: python test_slung_prelift_mujoco.py
"""

import csv
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml

try:
    import mujoco
except ImportError:
    mujoco = None

import simulate_slung_handoff as replay


@unittest.skipIf(mujoco is None, "MuJoCo is not installed in this Python environment")
class SlungPreliftMujocoTest(unittest.TestCase):
    def test_real_height_loss_triggers_supported_descent_before_vehicle_collision(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[5] / "simulate_uav" / "udaan"))
        from udaan.models.mujoco import crazyswarm_slung as plant

        candidate = yaml.safe_load((Path(__file__).parent / "simulation_configs" /
                                    "slung_70hz_candidate.yaml").read_text())
        # Exercise a stationary handoff case independently of the live mission.
        candidate["slung_payload"]["trajectory"] = {"mode": "hover"}
        original_manager = replay._slung_manager
        real_model = plant.CrazySwarmSlungModel

        def run_case(directory, inject):
            managers, models = [], []

            def capture_manager(*args, **kwargs):
                manager, anchors = original_manager(*args, **kwargs)
                managers.append(manager)
                return manager, anchors

            class DisturbedModel(real_model):
                injection_time = None

                def _apply_commands(self, commands, dt):
                    super()._apply_commands(commands, dt)
                    manager = managers[0]
                    start = manager.payload_transport_start_time
                    if not inject or start is None or self.data.time - start < 22.:
                        return
                    if self.injection_time is None:
                        self.injection_time = float(self.data.time)
                        models.append(self)
                    if self.data.time - self.injection_time < 4.:
                        # A brief external vertical spring/damper pulls the
                        # physical assembly below hover, long enough for the
                        # unchanged 2.20 s height guard to confirm the fault.
                        # A common acceleration avoids directly disturbing the
                        # relative formation. Measurements and guards stay real.
                        acceleration = (-64.*(self.data.xpos[self._load_body_id, 2] - .40)
                                        - 16.*self.data.qvel[self._load_dof + 2])
                        for body in [self._load_body_id] + self._vehicle_body_ids:
                            self.data.xfrc_applied[body, 2] += self.model.body_mass[body]*acceleration

            with patch.object(replay, "_slung_manager", capture_manager), patch.object(
                    plant, "CrazySwarmSlungModel", DisturbedModel):
                result = replay.run_simulation(directory, candidate,
                    pose_delay_s=.02, command_delay_s=.01)
            with (Path(directory) / "simulation_trace.csv").open(newline="") as stream:
                rows = [row for row in csv.DictReader(stream) if row["vehicle_id"] == "3"]
            return result, managers[0], models, rows

        with tempfile.TemporaryDirectory() as directory:
            stable, _, no_injection, _ = run_case(Path(directory) / "baseline", False)
            self.assertTrue(stable["success"], stable)
            self.assertIsNone(stable["failure"], stable)
            self.assertEqual(stable["payload_safety_reason"], "", stable)
            self.assertEqual(stable["payload_trajectory_mode"], "hover")
            self.assertGreaterEqual(stable["observed_post_handoff_s"], 60.)
            self.assertFalse(no_injection)

            result, manager, injected, rows = run_case(Path(directory) / "height_fault", True)
        self.assertEqual(len(injected), 1, result)
        self.assertFalse(result["success"], result)
        self.assertEqual(result["failure"], "controlled_landing_fault", result)
        self.assertEqual(result["payload_safety_reason"], "height_drop", result)
        self.assertTrue(result["terminal_reason"].startswith("payload_load: height_drop:"), result)
        self.assertEqual(result["phases"][-1], "landing", result)
        self.assertEqual(manager.payload_validation_landing_stage, "payload_lowering")
        self.assertTrue(manager.payload_transport_active)
        self.assertTrue(manager._payload_ready(result["final_time_s"]))
        self.assertGreater(injected[0].injection_time, result["handoff_time_s"] + 17.)
        self.assertGreater(result["minimum_vehicle_distance_m"], .30)
        self.assertGreaterEqual(result["minimum_clearance_m"], .15)
        self.assertLess(result["maximum_prelift_xy_error_m"], .25)
        # Check the final uninterrupted physical deficit, not merely the time
        # elapsed since an earlier transient below the reference.
        fallen = []
        for row in reversed(rows):
            now = float(row["control_time_s"])
            height_error = float(row["payload_target_position_z"]) - float(row["payload_position_z"])
            if now < injected[0].injection_time or height_error <= manager.payload_transport_guard.height_error_m:
                break
            fallen.append(now)
        self.assertTrue(fallen)
        self.assertGreaterEqual(result["final_time_s"] - fallen[-1],
                                manager.payload_transport_guard.confirm_s)
        self.assertTrue(np.all(np.isfinite(manager.latest_payload_state["velocity"])))
        self.assertTrue(np.all(np.isfinite(manager.latest_payload_state["acceleration"])))


if __name__ == "__main__":
    unittest.main()
