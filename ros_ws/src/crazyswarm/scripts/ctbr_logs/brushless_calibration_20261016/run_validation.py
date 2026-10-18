"""Compare unchanged controller gains against the two motor plants."""
import json
from pathlib import Path
import sys
import yaml

SCRIPTS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCRIPTS))
from simulate_slung_handoff import run_simulation

if __name__ == "__main__":
    mode = sys.argv[1]
    root = Path(__file__).resolve().parent
    payload = yaml.safe_load((SCRIPTS.parent / "config" / "slung_payload.yaml").read_text())["slung_payload"]
    simulation = dict(payload["simulation"])
    simulation["onboard_controller"] = "pid" if mode == "pid_paper" else "mellinger"
    if mode == "mellinger_legacy":
        simulation.update(motor_model="legacy_quadratic", vehicle_inertia=[2.395e-5,2.395e-5,3.234e-5],arm_length=.0465)
    overrides = {"slung_payload": {"simulation": simulation, "post_handoff_s":60.0}}
    result = run_simulation(root / mode, parameter_overrides=overrides)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result["success"] else 1)
