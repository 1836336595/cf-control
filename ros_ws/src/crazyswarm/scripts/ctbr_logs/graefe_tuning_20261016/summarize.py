"""Export tuning metrics and compare old/new gains with fixed paper physics."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parent

def read_trace(path):
    with path.open() as stream: return list(csv.DictReader(stream))

def vector(rows,field):
    return np.array([float(row[field]) for row in rows])

def main():
    results=[]
    for path in sorted(ROOT.rglob("summary.json")):
        value=json.loads(path.read_text())
        with (path.parent/"input_parameters.yaml").open() as stream:
            import yaml
            snapshot=yaml.safe_load(stream)
        row=dict(case=str(path.parent.relative_to(ROOT)),attitude=str(snapshot["ctbr_controller_cf3"]["transport_attitude_gain"]),rate_gain=str(snapshot["ctbr_controller_cf3"]["transport_rate_gain"]))
        for name in ("success","failure","payload_safety_reason","observed_post_handoff_s","minimum_vehicle_distance_m","minimum_clearance_m","maximum_body_rate_rps","maximum_tilt_deg","rate_saturation_fraction","final_payload_position_error_m"):
            row[name]=value[name]
        if value["success"]:
            trace=read_trace(path.parent/"simulation_trace.csv")
            tail=[item for item in trace if float(item["control_time_s"])>value["final_time_s"]-20]
            load=[item for item in tail if item["vehicle_id"]=="3"]
            row["last20s_payload_height_std_m"]=float(np.std(vector(load,"payload_height_m")))
            row["last20s_max_abs_roll_pitch_deg"]=float(np.degrees(max(abs(float(item["payload_"+axis+"_rad"])) for item in load for axis in ("roll","pitch"))))
            row["last20s_body_rate_rms_rps"]=float(np.sqrt(np.mean([float(item["body_rate_"+axis])**2 for item in tail for axis in "xyz"])))
        results.append(row)
    fields=list(dict.fromkeys(key for row in results for key in row))
    with (ROOT/"comparison.csv").open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fields); writer.writeheader();writer.writerows(results)
    (ROOT/"comparison.json").write_text(json.dumps(results,indent=2)+"\n")
    cases=[("Old gains [6,6,3] / [2,2,1]",ROOT.parent/"brushless_calibration_20261016/mellinger_paper"),
           ("Retuned [12,12,6] / [0.25,0.25,0.125]",ROOT/"final_default")]
    fig,axes=plt.subplots(4,2,figsize=(14,13),layout="constrained")
    colors={"3":"#9467bd","4":"#1f77b4","5":"#2ca02c"}
    for col,(title,directory) in enumerate(cases):
        summary=json.loads((directory/"summary.json").read_text())
        trace=read_trace(directory/"simulation_trace.csv")
        groups={cf:[r for r in trace if r["vehicle_id"]==cf and float(r["control_time_s"])>=summary["handoff_time_s"]-1] for cf in colors}
        time=vector(groups["3"],"control_time_s")-summary["handoff_time_s"]
        axes[0,col].plot(time,vector(groups["3"],"payload_height_m"),c="black",label="Payload height")
        axes[0,col].axhline(.6,ls="--",c="gray",label="Final target")
        for a,b in (("3","4"),("3","5"),("4","5")):
            pos=lambda cf: np.column_stack([vector(groups[cf],"position_"+axis) for axis in "xyz"])
            axes[1,col].plot(time,np.linalg.norm(pos(a)-pos(b),axis=1),label="CF"+a+"-CF"+b)
        axes[1,col].axhline(.3,ls=":",c="black",label="Guard")
        for cf,color in colors.items():
            rows=groups[cf]
            rate=np.column_stack([vector(rows,"body_rate_"+axis) for axis in "xyz"])
            axes[2,col].plot(time,np.max(np.abs(rate),axis=1),c=color,label="CF"+cf)
            axes[3,col].plot(time,vector(rows,"command_thrust_n"),c=color,label="CF"+cf+" command")
            axes[3,col].plot(time,vector(rows,"actual_thrust_n"),c=color,ls="--",label="CF"+cf+" actual")
        axes[0,col].set_title(title)
        for row,label in enumerate(("Payload center height (m)","Vehicle center distance (m)","Max absolute body rate (rad/s)","Thrust (N)")):
            ax=axes[row,col]; ax.set_ylabel(label);ax.grid(alpha=.25);ax.legend(fontsize=7,loc="best");ax.axvline(0,c="gray",ls=":")
            ax.set_xlim(-1,8 if col==0 else 60.1)
        axes[-1,col].set_xlabel("Time since handoff (s)")
    fig.suptitle("Same Graefe2026 motor model and Mellinger: old gains vs retuned gains")
    fig.savefig(ROOT/"comparison.png",dpi=150)
    print("cases",len(results),"saved",ROOT/"comparison.png")

if __name__=="__main__": main()
