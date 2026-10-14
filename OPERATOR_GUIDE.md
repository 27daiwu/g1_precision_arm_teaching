# Operator Guide

## Teaching and Recording

Run `python scripts/dual_arm_teach.py <interface>`. Press `F` to enter DRAG, `M` to lock and save a waypoint, `L` to list waypoint count, and `Q` to release and exit. Saved trajectory and log files remain under `data/trajectories/` and `logs/`.

## Replay

Run an offline validation with:

```bash
PYTHONPATH=src python -m g1_dual_arm_teaching.replay_cli <trajectory.npz> --mode absolute --speed 1.0
```

Use `--mode relative` for acquire-relative planning. Omit `--speed` to use the value in `configs/replay.yaml`. The network interface comes from `configs/robot.yaml`, or from unique live-interface detection when that field is empty. An ambiguous interface is an error.

Adding `--play` requests one hardware playback. The checked-in runtime configuration keeps `hardware_execute_enabled: false`; the request is rejected before DDS initialization until actual device-specific limits, gains, feedback bounds and stop constraints are established. No runtime defaults fill unknown values.
