"""Exercise the real fleet manager's moving load and formation references."""

import numpy as np
import pytest

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from test_slung_attitude_reference import sample


def circle_case(directory, **changes):
    payload = dict(
        trajectory=dict(mode="circle", radius_m=.5, speed_mps=.1,
                        ramp_s=5., settle_s=3., revolutions=1., direction="ccw"),
        transport_force_mode="reference_cascade",
        transport_attitude_trim_enabled=False,
        flight_validation_enabled=False,
    )
    payload.update(changes)
    manager, anchors = _slung_manager(
        _controller_module(), directory, {"slung_payload": payload})
    manager.payload_transport_active = True
    manager.payload_transport_start_time = 0.
    manager.payload_transport_start_position = manager.latest_payload_state["position"].copy()
    manager.payload_transport_vehicle_start_positions = anchors.copy()
    manager.payload_handoff_reference_positions = anchors.copy()
    sample(manager, 0.)
    return manager


def test_payload_follows_circle_at_fixed_height_with_body_x_inward(tmp_path):
    manager = circle_case(tmp_path)
    try:
        entry = next(iter(sample(manager, 19.).values()))["payload_target_position"]
        center = entry + [.5, 0., 0.]
        for now in [20., 25., 32., 40., 50., 57.]:
            item = next(iter(sample(manager, now).values()))
            p, r = item["payload_target_position"], item["payload_target_rotation"]
            assert p[2] == pytest.approx(.6)
            assert np.linalg.norm(p[:2]-center[:2]) == pytest.approx(.5)
            np.testing.assert_allclose(r[:, 0], (center-p)/.5, atol=1e-12)
            assert item["payload_trajectory_mode"] == "circle"
        assert np.linalg.norm(entry-next(iter(sample(manager, 58.).values()))["payload_target_position"]) < 1e-12
    finally:
        manager.logger.close()


def test_circle_rotates_full_formation_and_preserves_reference_rope_lengths(tmp_path):
    manager = circle_case(tmp_path)
    try:
        base = sample(manager, 19.)
        rho = manager.slung_controller.config.attachment_points_m
        initial_lengths = []
        for i, v in enumerate(manager.vehicles):
            b = base[id(v)]
            initial_lengths.append(np.linalg.norm(
                b["payload_target_position"]+b["payload_target_rotation"]@rho[:, i]
                -b["position_hold_position"]))
        for now in [25., 33., 43., 54.]:
            values = sample(manager, now)
            for i, v in enumerate(manager.vehicles):
                b = values[id(v)]
                length = np.linalg.norm(b["payload_target_position"]
                    +b["payload_target_rotation"]@rho[:, i]-b["position_hold_position"])
                assert length == pytest.approx(initial_lengths[i], abs=1e-12)
    finally:
        manager.logger.close()


@pytest.mark.parametrize("now", [21.5, 30., 54.9])
def test_circle_payload_and_both_aircraft_reference_derivatives_are_consistent(tmp_path, now):
    manager = circle_case(tmp_path)
    try:
        step = 1e-3
        left, middle, right = [sample(manager, t) for t in [now-step, now, now+step]]
        for vehicle in manager.vehicles:
            a, b, c = [values[id(vehicle)] for values in [left, middle, right]]
            for p, v, acc in [("payload_target_position", "payload_target_velocity", "payload_target_acceleration"),
                              ("position", "velocity", "acceleration"),
                              ("position_hold_position", "position_hold_velocity", "position_hold_acceleration")]:
                np.testing.assert_allclose((c[p]-a[p])/(2*step), b[v], atol=2e-8)
                np.testing.assert_allclose((c[p]-2*b[p]+a[p])/step**2, b[acc], atol=3e-8)
            rdot = (c["payload_target_rotation"]-a["payload_target_rotation"])/(2*step)
            w = b["payload_target_body_rate"]
            hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
            np.testing.assert_allclose(rdot, b["payload_target_rotation"]@hat, atol=2e-8)
            np.testing.assert_allclose((c["payload_target_body_rate"]-a["payload_target_body_rate"])/(2*step),
                                      b["payload_target_body_rate_dot"],atol=2e-8)
            assert np.linalg.norm(b["payload_target_velocity"]) > .005
            assert w[2] > .01
    finally:
        manager.logger.close()


def test_hover_mode_keeps_static_pose_and_no_circle_acceleration(tmp_path):
    manager = circle_case(tmp_path, trajectory={"mode": "hover"})
    try:
        a = next(iter(sample(manager, 20.).values()))
        b = next(iter(sample(manager, 35.).values()))
        np.testing.assert_array_equal(a["payload_target_position"], b["payload_target_position"])
        np.testing.assert_array_equal(b["payload_target_rotation"], np.eye(3))
        np.testing.assert_array_equal(b["payload_target_body_rate"], np.zeros(3))
    finally:
        manager.logger.close()


def test_circle_cannot_be_silently_cut_off_by_validation_deadline(tmp_path):
    with pytest.raises(RuntimeError, match="circle|圆周"):
        circle_case(tmp_path, flight_validation_enabled=True, flight_validation_transport_s=30.)


def test_circle_requires_payload_yaw_control_for_inward_heading(tmp_path):
    with pytest.raises(RuntimeError, match="yaw|航向"):
        circle_case(tmp_path, yaw_channel_enabled=False)


def test_circle_also_requires_enough_ordinary_hover_time_when_validation_is_off(tmp_path):
    with pytest.raises(RuntimeError, match="circle|圆周"):
        circle_case(tmp_path, trajectory={"mode":"circle", "speed_mps":.001})


def test_circle_feedforward_accounts_for_rotating_cables_without_double_centripetal(tmp_path):
    manager = circle_case(tmp_path)
    try:
        targets = sample(manager, 30.)
        for i, vehicle in enumerate(manager.vehicles):
            item = targets[id(vehicle)]
            q = item["link_direction"]
            correction = item["circle_formation_acceleration_feedforward"]
            # Rotating formation produces a tangential cable acceleration,
            # not another copy of its centripetal acceleration.
            assert abs(float(correction @ item["circle_reference_link_direction"])) < 1e-12
            assert np.linalg.norm(correction) > 1e-4
    finally:
        manager.logger.close()


def test_circle_force_at_exact_tracking_equals_support_plus_full_aircraft_inertia(tmp_path):
    manager = circle_case(tmp_path)
    try:
        overrides = sample(manager, 30.)
        first = next(iter(overrides.values()))
        payload = dict(valid=True, position=first["payload_target_position"],
            velocity=first["payload_target_velocity"], acceleration=first["payload_target_acceleration"],
            rotation=first["payload_target_rotation"], body_rate=first["payload_target_body_rate"])
        target = dict(position=payload["position"], velocity=payload["velocity"],
            acceleration=payload["acceleration"], rotation=payload["rotation"],
            body_rate=payload["body_rate"], body_rate_dot=first["payload_target_body_rate_dot"],
            payload_attitude_gain=manager.payload_attitude_gain, payload_rate_gain=manager.payload_rate_gain)
        vehicles = [dict(position=overrides[id(v)]["position_hold_position"],
            velocity=overrides[id(v)]["position_hold_velocity"], mass=v.controller.config.mass)
            for v in manager.vehicles]
        manager.slung_controller.reset()
        computed = manager.slung_controller.compute(payload, vehicles, target, 1/70)
        for index, vehicle in enumerate(manager.vehicles):
            overrides[id(vehicle)]["desired_force_override"] = computed["vehicles"][index]["force"].copy()
        manager._apply_circle_formation_feedforward(overrides, manager.payload_circle_state, 1/70)
        for index, vehicle in enumerate(manager.vehicles):
            output = computed["vehicles"][index]
            item = overrides[id(vehicle)]
            expected = (-output["desired_tension"]*output["link_direction"]
                + vehicle.controller.config.mass*(item["position_hold_acceleration"]+[0.,0.,9.80665]))
            np.testing.assert_allclose(item["desired_force_override"], expected, atol=1e-12)
    finally:
        manager.logger.close()


def test_learned_trim_rotates_with_circle_without_learning_tracking_error(tmp_path):
    manager = circle_case(tmp_path, transport_attitude_trim_enabled=True)
    try:
        trim = manager.payload_attitude_trim
        angle = .12
        tilted = np.array([[np.cos(angle), 0., np.sin(angle)], [0.,1.,0.],
                           [-np.sin(angle),0.,np.cos(angle)]])
        trim.update(tilted, np.eye(3), 18., learning=True)
        integral = trim.integral.copy()
        step, now = 1e-3, 30.
        left, middle, right = [sample(manager, t) for t in [now-step, now, now+step]]
        np.testing.assert_array_equal(trim.integral, integral)
        for vehicle in manager.vehicles:
            a,b,c = [items[id(vehicle)] for items in [left,middle,right]]
            assert b["transport_attitude_trim_freeze_reason"] == "reference_moving"
            for p,v,acc in [("position","velocity","acceleration"),
                            ("position_hold_position","position_hold_velocity","position_hold_acceleration")]:
                np.testing.assert_allclose((c[p]-a[p])/(2*step),b[v],atol=3e-8)
                np.testing.assert_allclose((c[p]-2*b[p]+a[p])/step**2,b[acc],atol=4e-8)
    finally:
        manager.logger.close()


@pytest.mark.parametrize("trim_enabled", [False, True])
@pytest.mark.parametrize("now", [22., 30.])
def test_interrupted_circle_landing_preserves_orientation_rate_and_vehicle_pva(tmp_path, trim_enabled, now):
    manager = circle_case(tmp_path, transport_attitude_trim_enabled=trim_enabled)
    try:
        previous = sample(manager, now)
        first = next(iter(previous.values()))
        manager.last_transport_overrides = previous
        manager.latest_payload_state.update(position=first["payload_target_position"].copy(),
            rotation=first["payload_target_rotation"].copy(), velocity=first["payload_target_velocity"].copy(),
            acceleration=first["payload_target_acceleration"].copy(), body_rate=first["payload_target_body_rate"].copy())
        for v in manager.vehicles:
            v.latest_state["position"] = previous[id(v)]["position"].copy()
            v.trajectory.begin_controlled_landing(now, v.latest_state["position"], 0., "test_circle_exit")
        current = sample(manager, now)
        for v in manager.vehicles:
            old,new = previous[id(v)],current[id(v)]
            for field in ["payload_target_rotation","payload_target_body_rate","payload_target_body_rate_dot",
                          "position","velocity","acceleration","position_hold_position",
                          "position_hold_velocity","position_hold_acceleration"]:
                np.testing.assert_allclose(new[field],old[field],atol=1e-10,err_msg=field)
        later = sample(manager, now+6.)
        np.testing.assert_allclose(next(iter(later.values()))["payload_target_body_rate"],0.,atol=1e-12)
    finally:
        manager.logger.close()


def test_circle_landing_bridge_advances_during_cached_gap_with_trim_disabled(tmp_path):
    manager = circle_case(tmp_path)
    try:
        now = 30.
        previous = sample(manager, now)
        first = next(iter(previous.values()))
        manager.last_transport_overrides = previous
        manager.latest_payload_state.update(position=first["payload_target_position"].copy(),
            rotation=first["payload_target_rotation"].copy(), velocity=first["payload_target_velocity"].copy(),
            acceleration=first["payload_target_acceleration"].copy())
        for v in manager.vehicles:
            v.latest_state["position"] = previous[id(v)]["position"]+[.01,0.,0.]
            v.trajectory.begin_controlled_landing(now,v.latest_state["position"],0.,"test_circle_exit")
        cached = sample(manager,now)
        expected = manager._evaluate_attitude_trim_bridge(manager.payload_attitude_trim_landing_bridge,now+.2)
        manager._refresh_cached_attitude_trim(now+.2,cached,"payload_cached_hold")
        for i,v in enumerate(manager.vehicles):
            for order,name in enumerate(["position","velocity","acceleration"]):
                np.testing.assert_allclose(cached[id(v)]["transport_attitude_trim_bridge_"+name],
                                           expected[order][i],atol=1e-12)
    finally:
        manager.logger.close()
