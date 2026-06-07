# PID Tuning

This folder now has a simpler tracking-only step-response runner so we can inspect one PID setting at a time before attempting any autotuning.

Recommended script:

- `tune_tracking.py`

What it does:

- sets one tracking PID configuration from the CLI
- rebases the logical position to `0` before each test
- runs tracking-only step tests with very wide bounds
- captures telemetry for the requested turn steps, defaulting to `+/-0.5` and `+/-5.0`
- prints key response metrics including settle time, settle distance, overshoot, rise time, final error, speed, and torque
- writes one combined CSV log under `pid_tuning/logs/`
- can also write a PNG plot with `--plot`

Example usage:

```powershell
conda activate game
python pid_tuning\tune_tracking.py --port COM4 --tracking-kp 2.5 --tracking-kd 0.002 --tracking-max-torque 6.0 --plot
```

Useful knobs:

- `--turns`: comma-separated turn amplitudes to test
- `--settle-band-deg`: angle band used for settled detection
- `--settle-speed-deg-s`: speed band used for settled detection
- `--small-step-capture-s`: capture window for the smaller step tests
- `--large-step-capture-s`: capture window for the larger step tests
- `--wide-bound-turns`: very wide half-bound used to isolate tracking from bounds behavior
- `--preview-rows`: how many telemetry rows to print per test as a console preview

The older combined experiment is still in this folder as `tune_tracking_and_bounds.py`, but the simpler tracking script should be the default path until the tracking behavior is understood.