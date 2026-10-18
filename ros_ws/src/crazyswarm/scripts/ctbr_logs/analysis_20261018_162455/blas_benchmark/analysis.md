# OpenBLAS thread count microbenchmark

Sequential fresh `/usr/bin/python3` processes (Python 3.8.10, NumPy 1.24.4) ran the same copied payload-controller fixture with 100 warmup and 500 measured calls each. Order was default, one thread, one thread, default. The loaded OpenBLAS library reported 16 default threads and one thread when all three environment variables were set to 1. No ROS callbacks, radio traffic, live plot, or flight was started.

| Controller | Threads | Mean (ms) | p50 | p95 | p99 | Max |
|---|---:|---:|---:|---:|---:|---:|
| slung | 16 | 0.8559 | 0.8598 | 0.9383 | 1.1575 | 1.4297 |
| slung | 1 | 0.8227 | 0.8146 | 0.9023 | 1.0343 | 1.9054 |
| geometric | 16 | 0.1734 | 0.1625 | 0.2693 | 0.3095 | 0.6493 |
| geometric | 1 | 0.1547 | 0.1514 | 0.1776 | 0.2051 | 0.2448 |

All numeric controller outputs agree across all 500 corresponding measured calls within rtol=1e-11, atol=1e-12. No measured compute call exceeded the 14.286 ms nominal control budget. The payload controller mean and tails do not show a repeatable large reduction from limiting BLAS threads. These results do not demonstrate that default BLAS threading explains the observed 40–58 ms fleet callback overruns.

The geometric controller first default run was modestly slower; the second default run overlapped one-thread timings. Host scheduling and other load are absent from this microbenchmark. Disabling the real-time CSV plot while keeping controller gains fixed is a stronger next diagnostic comparison. Thread limiting may be a reasonable resource bound, but should not be described as a measured fix for this flight.

Files: `comparison.json` includes environment, confirmed library/thread counts, four run distributions and numeric equivalence checks. NPZ files retain every wall duration and flattened controller output. Reproduce using `../benchmark_blas_threads.py`.
