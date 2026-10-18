#!/usr/bin/env python3
"""Run frozen physical/controller rope pairs in isolated single-thread processes."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from time import perf_counter


ROOT = Path(__file__).resolve().parent
SCRIPTS = ROOT.parents[1]
PYTHON = '/home/csy/miniconda3/envs/udaan/bin/python'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def child(case):
    import yaml
    sys.path.insert(0, str(SCRIPTS))
    from simulate_slung_handoff import run_simulation
    params = yaml.safe_load(Path(case['parameters_path']).read_text())
    if digest(Path(case['parameters_path'])) != case['parameters_sha256']:
        raise ValueError('frozen case parameters changed')
    directory = Path(case['output_directory'])
    directory.mkdir(parents=True, exist_ok=True)
    started = perf_counter()
    result = run_simulation(directory, parameter_overrides=params,
                            post_handoff_s=60., target_payload_height_m=.60)
    metadata = dict(case=case['name'], case_index=case['index'], worker=case['worker'],
                    wall_time_s=perf_counter()-started, parameters_sha256=case['parameters_sha256'],
                    source_sha256={name:digest(SCRIPTS/name) for name in (
                        'simulate_slung_handoff.py', 'ctbr_controller.py', 'slung_load_controller.py',
                        'slung_link_calibration.py')},
                    blas_environment={name:os.environ.get(name) for name in (
                        'OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}, summary=result)
    (directory / 'run_metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    keys=('success','failure','handoff_time_s','observed_post_handoff_s','final_time_s',
          'minimum_clearance_m','minimum_vehicle_distance_m','maximum_body_rate_rps',
          'maximum_tilt_deg','rate_saturation_fraction','final_payload_position_error_m',
          'link_calibration_applied','payload_safety_reason')
    print(json.dumps(dict(case=case['name'], wall_time_s=metadata['wall_time_s'],
                          **{key:result.get(key) for key in keys})),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker',type=int,choices=(0,1,2))
    parser.add_argument('--case')
    parser.add_argument('--child',action='store_true')
    args=parser.parse_args()
    manifest=json.loads((ROOT/'cases.json').read_text())
    cases=[case for case in manifest['cases'] if
           (args.worker is None or case['worker']==args.worker) and
           (args.case is None or case['name']==args.case)]
    if not cases:
        parser.error('no matching cases')
    if args.child:
        if len(cases)!=1:
            parser.error('--child requires one --case')
        child(cases[0]);return
    if args.worker is None and args.case is None:
        parser.error('select --worker 0/1/2 or a single --case')
    results=[]
    for case in cases:
        output=Path(case['output_directory']);metadata=output/'run_metadata.json'
        if metadata.exists():
            results.append(json.loads(metadata.read_text()));
            print(json.dumps(dict(case=case['name'],status='already_finished')),flush=True);continue
        output.mkdir(parents=True,exist_ok=True)
        environment=os.environ.copy()
        for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
            environment[name]='1'
        command=[PYTHON,str(Path(__file__).resolve()),'--child','--case',case['name']]
        started=perf_counter()
        with (output/'console.txt').open('w') as console:
            process=subprocess.run(command,env=environment,stdout=console,stderr=subprocess.STDOUT,text=True)
        if process.returncode or not metadata.exists():
            record=dict(case=case['name'],status='process_error',returncode=process.returncode,
                        wall_time_s=perf_counter()-started)
            (output/'process_error.json').write_text(json.dumps(record,indent=2)+'\n')
            print(json.dumps(record),flush=True);results.append(record);continue
        record=json.loads(metadata.read_text());results.append(record)
        print(json.dumps(dict(case=case['name'],wall_time_s=record['wall_time_s'],
                              success=record['summary']['success'],failure=record['summary']['failure'],
                              observed_post_handoff_s=record['summary']['observed_post_handoff_s'])),flush=True)
        destination=ROOT/('worker_%s_results.json' % (args.worker if args.worker is not None else args.case))
        destination.write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':
    main()
