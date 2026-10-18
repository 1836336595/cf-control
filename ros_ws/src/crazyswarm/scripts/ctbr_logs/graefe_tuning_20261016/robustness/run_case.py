"""Stress a frozen candidate without changing its controller or guards."""
import json
import csv
from pathlib import Path
import sys
import yaml
ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT.parents[1]
sys.path.insert(0,str(SCRIPTS))
from simulate_slung_handoff import run_simulation

if __name__ == "__main__":
    name=sys.argv[1]
    source=Path(sys.argv[2]) if len(sys.argv)>2 else ROOT/"rate_scan/r6_d025/input_parameters.yaml"
    snapshot=yaml.safe_load(source.read_text())
    overrides={key:snapshot[key] for key in ("ctbr_controller","ctbr_trajectory","slung_payload","ctbr_controller_cf3","ctbr_controller_cf4","ctbr_controller_cf5") if key in snapshot}
    simulation=overrides["slung_payload"]["simulation"]
    if "delay" in name:
        simulation.update(pose_delay_s=.02,command_delay_s=.01)
    if "eff85" in name:
        simulation["motor_thrust_efficiency"]=[.85]*3
    if "eff80" in name:
        simulation["motor_thrust_efficiency"]=[.8]*3
    if "cap75" in name:
        simulation["motor_available_command_fraction"]=[.75]*3
    if "rawscale" in name:
        with (SCRIPTS/"ctbr_logs/multi_ctbr_20261016_110152.csv").open() as stream:
            scales={}
            for row in csv.DictReader(stream):
                scales.setdefault(row["vehicle_id"],float(row["thrust_raw_scale"]))
        simulation["bridge_thrust_raw_scale"]=[scales[str(i)] for i in (3,4,5)]
    if "long" in name:
        overrides["slung_payload"]["post_handoff_s"]=120.0
        simulation["time_limit_s"]=240.0
    tilt=2.0 if "tilt" in name else 0.0
    out=ROOT/"robustness"/name
    out.mkdir(parents=True,exist_ok=True)
    (out/"overrides.yaml").write_text(yaml.safe_dump(overrides,sort_keys=False))
    result=run_simulation(out,overrides,initial_tilt_deg=tilt)
    print(json.dumps(result,indent=2))
    sys.exit(0 if result["success"] else 1)
