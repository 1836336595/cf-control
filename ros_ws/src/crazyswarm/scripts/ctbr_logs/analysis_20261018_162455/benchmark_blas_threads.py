#!/usr/bin/env python3
"""Sequential fresh-process ROS-Python controller benchmark; no ROS/radio I/O."""

import argparse
from copy import deepcopy
import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys
from time import perf_counter


SCRIPTS = Path(__file__).resolve().parents[2]


def describe_blas():
    libraries = []
    for line in Path('/proc/self/maps').read_text().splitlines():
        if 'openblas' not in line.lower() or '.so' not in line:
            continue
        path = line.split()[-1]
        if path in [item['path'] for item in libraries]:
            continue
        lib = ctypes.CDLL(path)
        count = None
        for name in ('openblas_get_num_threads64_', 'openblas_get_num_threads',
                     'openblas_get_num_threads_'):
            if hasattr(lib, name):
                function = getattr(lib, name)
                function.restype = ctypes.c_int
                count = function()
                break
        libraries.append(dict(path=path, threads=count))
    return libraries


def worker(path, label):
    import numpy as np
    sys.path.insert(0, str(SCRIPTS))
    from test_slung_load_controller import _link_force_compute_case
    from test_ctbr_controller_v2 import _controller_module, _config, _state, _target

    payload_controller, payload, vehicles, target = _link_force_compute_case()
    theta = .10
    payload['rotation'] = np.array([[np.cos(theta), 0., np.sin(theta)], [0., 1., 0.],
                                    [-np.sin(theta), 0., np.cos(theta)]])
    payload['velocity'] = np.array([.03, -.02, .01])
    payload['body_rate'] = np.array([.15, -.08, .2])
    for index, vehicle in enumerate(vehicles):
        vehicle['velocity'] = np.array([.02 * (index - 1), .01, -.02])
    inertia = .054 / 12 * np.array([.06**2 + .05**2, .08**2 + .05**2, .08**2 + .06**2])
    bandwidth = np.array([1., 1., .45])
    target['payload_attitude_gain'] = inertia * (2 * np.pi * bandwidth)**2
    target['payload_rate_gain'] = 2 * .90 * inertia * 2 * np.pi * bandwidth
    target['link_gain_scale'] = .4
    payload_fixture = (deepcopy(payload), deepcopy(vehicles), deepcopy(target))

    module = _controller_module()
    geo_controller = module.GeometricCtbrController(_config(module))
    state = _state(rotation=payload['rotation'].copy())
    state['body_rate'] = np.array([.15, -.08, .2])
    reference = _target()
    reference.update(transport_mode=True, independent_mode=False,
                     transport_blend=1., transport_attitude_gain=[12., 12., 6.],
                     transport_rate_gain=[.25, .25, .125],
                     desired_force_override=np.array([.04, -.02, .8]),
                     desired_force_dot_override=np.array([.01, .02, -.01]))
    dt = 1 / 70.

    def flatten(value):
        if isinstance(value, dict):
            return np.concatenate([flatten(value[key]) for key in sorted(value)])
        if isinstance(value, (list, tuple)):
            return np.concatenate([flatten(item) for item in value])
        if isinstance(value, str) or value is None:
            return np.empty(0)
        return np.asarray(value, dtype=float).ravel()

    def measure(name, compute):
        for _ in range(100):
            compute()
        durations = []
        outputs = []
        for _ in range(500):
            start = perf_counter()
            result = compute()
            durations.append(perf_counter() - start)
            outputs.append(flatten(result))
        times = np.array(durations)
        np.savez(path / (label + '_' + name + '.npz'), times_s=times, outputs=np.array(outputs))
        return dict(samples=500, warmup=100, mean_ms=float(times.mean() * 1000),
                    p50_ms=float(np.percentile(times, 50) * 1000),
                    p95_ms=float(np.percentile(times, 95) * 1000),
                    p99_ms=float(np.percentile(times, 99) * 1000),
                    maximum_ms=float(times.max() * 1000),
                    over_14_286_ms_count=int(np.sum(times > 1/70)))

    result = dict(label=label, python=sys.version, numpy=np.__version__,
                  environment={key: os.environ.get(key) for key in
                               ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')},
                  blas=describe_blas())
    result['slung_controller'] = measure('slung', lambda: payload_controller.compute(*payload_fixture, dt))
    result['geometric_controller'] = measure('geometric', lambda: geo_controller.compute(state, reference, dt))
    (path / (label + '.json')).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', default=None)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'blas_benchmark')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.worker:
        worker(args.output, args.worker)
        return
    # Alternate order on the second pair, always sequential, to expose gross
    # warm-machine/order effects without CPU contention from parallel workers.
    results = []
    for label, threads in [('default_1', None), ('single_1', '1'),
                           ('single_2', '1'), ('default_2', None)]:
        environment = os.environ.copy()
        for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
            environment.pop(name, None)
            if threads is not None:
                environment[name] = threads
        process = subprocess.run(['/usr/bin/python3', str(Path(__file__).resolve()), '--worker', label,
                                  '--output', str(args.output.resolve())], env=environment,
                                 text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if process.returncode:
            raise RuntimeError(process.stderr)
        result = json.loads((args.output / (label + '.json')).read_text())
        results.append(result)
        print(process.stdout.strip(), flush=True)
    import numpy as np
    comparison = {}
    for kind in ('slung', 'geometric'):
        baseline = np.load(args.output / ('default_1_' + kind + '.npz'))['outputs']
        maximum_error = 0.
        for label in ('single_1', 'single_2', 'default_2'):
            candidate = np.load(args.output / (label + '_' + kind + '.npz'))['outputs']
            np.testing.assert_allclose(candidate, baseline, rtol=1e-11, atol=1e-12)
            maximum_error = max(maximum_error, float(np.max(np.abs(candidate - baseline))))
        comparison[kind] = dict(maximum_absolute_output_difference=maximum_error,
                                rtol=1e-11, atol=1e-12, outputs_equivalent=True)
    combined = dict(
        experiment='sequential fresh ROS Python 3.8 processes, unbounded BLAS vs one thread',
        no_ros_radio_io=True, controller_sources_unchanged=True,
        compute_inputs='Identical copied deterministic fixture, fixed 1/70 s dt, no per-call input copying in timed section',
        output_comparison=comparison, runs=results,
        limitation='A standalone controller microbenchmark, not a full ROS callback benchmark; scheduler, plotting, logging and other process contention remain unmeasured.')
    (args.output / 'comparison.json').write_text(json.dumps(combined, indent=2) + '\n')


if __name__ == '__main__':
    main()
