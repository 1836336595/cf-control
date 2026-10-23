"""TAKEUP geometry and calibration must use their quality-checked snapshot."""

from copy import deepcopy
import math
from types import SimpleNamespace

import numpy as np

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from test_slung_link_calibration_integration import setup_manager


class _ReceiveAfterUnlock:
    """Deliver one callback after a payload snapshot has been selected."""

    def __init__(self, original, receive):
        self.original = original
        self.receive = receive

    def __enter__(self):
        return self.original.__enter__()

    def __exit__(self, *args):
        result = self.original.__exit__(*args)
        receive, self.receive = self.receive, None
        if receive is not None:
            receive()
        return result


def test_takeup_cannot_use_rejected_geometry_just_because_a_new_good_frame_arrives(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path, {
        "slung_payload": {"link_calibration_enabled": False}})
    try:
        good = deepcopy(manager.latest_payload_state)
        rejected = dict(good, position=np.array([0., 0., .10]), derivatives_valid=False)
        manager.latest_payload_state = rejected
        manager.payload_takeup_start_positions = anchors.copy()
        manager.payload_takeup_target_positions = anchors.copy()
        manager.payload_takeup_start_time = 0.
        manager.payload_lock = _ReceiveAfterUnlock(
            manager.payload_lock, lambda: setattr(manager, "latest_payload_state", good))
        overrides = manager._payload_takeup_overrides(0.)
        assert len(overrides) == 3
        assert math.isnan(manager.payload_ground_clearance_m)
        assert not any(item["prelift_vertical_integral_active"] for item in overrides.values())
    finally:
        manager.logger.close()


def test_link_calibration_rejects_supplied_bad_snapshot_even_when_latest_is_good(tmp_path):
    module, manager, _ = setup_manager(tmp_path)
    try:
        now = 9.
        module.rospy.Time.now = lambda: SimpleNamespace(to_sec=lambda: 100. + now)
        manager.latest_payload_state.update(received_time=now, sample_time=100. + now)
        states = []
        for vehicle in manager.vehicles:
            vehicle.latest_state.update(received_time=now, mocap_sample_time_s=100. + now)
            vehicle.latest_ekf_state = dict(
                position=vehicle.latest_state["position"], received_time=now,
                filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3),
                filter_derivatives_valid=True)
            states.append(vehicle.latest_state)
        measured = np.asarray([state["position"] for state in states])
        rejected = dict(manager.latest_payload_state, derivatives_valid=False,
                        observer_updated=False, observer_quality_valid=False, source_valid=True)
        reference = dict(stage="takeup", failed=False, positions=measured.copy())
        manager._update_payload_link_calibration(now, measured, rejected, reference, 1., states)
        assert manager.payload_link_calibrator.sample_count == 0
        assert not manager.payload_link_calibration_applied
    finally:
        manager.logger.close()


def test_takeup_handoff_checks_and_uses_one_deepcopied_geometry_snapshot(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        raised = anchors + [0., 0., .22]
        manager.latest_payload_state["position"] += [0., 0., .22]
        manager.latest_payload_state["received_time"] = 2.2
        for vehicle, position in zip(manager.vehicles, raised):
            vehicle.latest_state["position"] = position.copy()
        manager.payload_prelift_confirm_s = manager.payload_takeup_confirm_s = 0.
        manager.payload_prelift_reference = SimpleNamespace(stage="holding")

        def receive_rejected_frame():
            # Also mutate the old array to prove the selected snapshot owns
            # its geometry rather than borrowing a callback-owned array.
            manager.latest_payload_state["position"][:] = [7., 8., 9.]
            manager.latest_payload_state = dict(manager.latest_payload_state, derivatives_valid=False)

        manager.payload_lock = _ReceiveAfterUnlock(manager.payload_lock, receive_rejected_frame)
        assert manager._payload_takeup_ready(2.2)
        assert manager.payload_prelift_ready
        assert manager.payload_ground_clearance_m < .3
    finally:
        manager.logger.close()
