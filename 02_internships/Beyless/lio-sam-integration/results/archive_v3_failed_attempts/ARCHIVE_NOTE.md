# v3 failed attempt preservation

First batch: full six runs PASS, even_loop_on failed sanity (1380.69 m, max speed 175.20 m/s, 14 IMU velocity resets, loops 0). Preserved intact. Retry only even_loop_on, identical params SHA256 and rate 1.0, isolated from evaluation workload. Failure and retry must be disclosed in TASK-05 log; this is not a robustness success claim. No parameter tuning or failed-result deletion.
