#!/usr/bin/env python3
"""Exercise the real early-distance exit in MuJoCo; no physical close approach.

This injects only the guard's reason at six seconds after handoff. Measured
positions, actual pair distances, and all plant/controller parameters remain
unchanged. It establishes branch reachability, not collision avoidance or a
successful fault landing. Production source files are not modified.
"""
import json
from pathlib import Path
import sys

import yaml

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parents[1]
sys.path.insert(0, str(SCRIPTS))

import simulate_slung_handoff as replay


def main():
    output = HERE / "mass23_rope830_injected_distance6"
    output.mkdir(exist_ok=True)
    candidate = yaml.safe_load(
        (HERE / "mass23_rope830_nominal60" / "candidate.yaml").read_text()
    )
    candidate["slung_payload"]["flight_validation_enabled"] = False
    candidate["slung_payload"]["post_handoff_s"] = 60.0
    original_factory = replay._slung_manager
    audit = dict(
        method="guard reason injection in actual MuJoCo replay",
        physical_close_approach_injected=False,
        production_source_changed=False,
        expected_result="early distance fault enters independent emergency_landing",
        injected=False,
    )
    managers = []

    def factory(*args, **kwargs):
        manager, anchors = original_factory(*args, **kwargs)
        managers.append(manager)
        original_check = manager.payload_transport_guard.check

        def guard_check(now, positions, velocities, height_error, rate_saturated):
            result = original_check(now, positions, velocities, height_error, rate_saturated)
            start = manager.payload_transport_start_time
            if (not audit["injected"] and manager.payload_transport_active
                    and start is not None and now - start >= 6.0):
                audit.update(
                    injected=True, injected_time_s=float(now), handoff_time_s=float(start),
                    actual_minimum_distance_m=float(result["minimum_distance_m"]),
                    actual_predicted_distance_m=float(result["predicted_distance_m"]),
                    original_guard_reason=result["reason"], injected_reason="distance",
                )
                result = dict(result, reason="distance")
            return result

        manager.payload_transport_guard.check = guard_check
        return manager, anchors

    replay._slung_manager = factory
    try:
        result = replay.run_simulation(output, parameter_overrides=candidate)
        manager = managers[0]
        audit.update(
            replay_success=bool(result["success"]), replay_failure=result["failure"],
            terminal_phases=[v.trajectory.phase for v in manager.vehicles],
            controlled_exit_reason=manager.payload_controlled_exit_reason,
            payload_transport_active=bool(manager.payload_transport_active),
            rope_preserving_initialized=manager.payload_landing_rope_directions is not None,
        )
        audit["expected_branch_reproduced"] = bool(
            audit["injected"] and not audit["replay_success"]
            and all(p == "emergency_landing" for p in audit["terminal_phases"])
            and not audit["rope_preserving_initialized"]
        )
        (output / "injection_audit.json").write_text(json.dumps(audit, indent=2))
        print(json.dumps(audit, indent=2))
        return 0 if audit["expected_branch_reproduced"] else 1
    finally:
        replay._slung_manager = original_factory


if __name__ == "__main__":
    raise SystemExit(main())
