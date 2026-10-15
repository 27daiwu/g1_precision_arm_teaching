# G1 Dual Arm Teaching and Replay

Pure C++17 applications using Unitree SDK2. The only command publisher is
`rt/arm_sdk`; feedback is read from `rt/lowstate`. Motors 0-11 are never
commanded. Motors 12-14 hold a fixed reference captured before ownership, and
motors 15-28 are the two 7-DoF arms. Motor 29 carries Arm SDK ownership weight.

## Build

Install Unitree SDK2 and JsonCpp, then build with CMake:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

The build uses the installed SDK2 CMake package. To use a source checkout,
configure with its exported package path, for example:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/hebe/unitree_workspace/unitree_sdk2/build
```

## Record

```sh
./build/g1_calligraphy_record 福
```

`--interface` defaults to `eth0`. `--waist-pitch-extra-bias RAD` adds an offset
to motor14 in the common waist hold command path. It defaults to +0.050 rad on
top of the existing +0.010 rad baseline; the accepted extra-bias range is
currently +/-0.050 rad.
The extra offset ramps in during Golden acquire and out during release. Verify
the direction on the robot with +0.005 rad before increasing it. Waist logs
include the extra bias, motor14 reference and measured position, tracking error,
and acquire/release progress.

Commands are line based:

- `F`: toggle DRAG and LOCK.
- `C`: start/stop whole-character 17-DoF q/dq capture; starting is accepted only in DRAG.
- `L`: show recording status, duration, and frame count.
- `S`: save a stopped character recording under `data/trajectories/calligraphy/`.
- `Q`: release; an unsaved recording requires a second confirmation.

## Replay

```sh
./build/g1_replay --list
./build/g1_replay my_track --preview
./build/g1_replay my_track --interface eth0 --cycles 3 --hold 2
./build/g1_replay my_track --interface eth0 --forever --speed 0.5
./build/g1_replay my_track --continuous --preview
./build/g1_replay my_track --continuous --interface eth0 --cycles 1 \
  --speed-scale 0.25 --motion-scale 1.0
./build/g1_replay my_track --continuous --high-precision \
  --interface eth0 --cycles 1 --speed-scale 0.25 \
  --position-tolerance 0.02 --node-timeout 5
```

Calligraphy replay uses the Phase 1 17-DoF JSON directly:

```sh
./build/g1_calligraphy_replay --file data/trajectories/calligraphy/FU_2026_10_15_11_20_04_597.json --validate-only
./build/g1_calligraphy_replay --file data/trajectories/calligraphy/FU_2026_10_15_11_20_04_597.json \
  --interface eth0 --speed-scale 0.25 --waist-pitch-extra-bias 0.05 \
  --allow-unknown-waist-metadata
```

The replay validates the complete 17-DoF file, but commands only arm motors
15-28. Waist motors 12-14 remain under the shared fixed WaistHold. MOVE_TO_START
is smoothed and speed-limited, then `READY_START` waits for Enter. The full
timestamped path plays once; `HOLD_END` waits for Enter before Golden release.
Files without recorded waist bias metadata report UNKNOWN and require an
explicit override for execution. `--validate-only` never creates a DDS channel.
Replay CSV logs include per-arm-joint target, measured angle, error, control rate,
and state.

Before either command takes Arm SDK ownership, it waits for a 1 second stable
motion window. `--wait-timeout SEC` changes the default 10 second read-only wait.
On timeout the program exits without creating an Arm SDK publisher.

`--hold` overrides the hold at every point. Without it, each point's `hold_s`
is used. `--speed` is the maximum joint-space speed in rad/s. Replay first
moves smoothly from the measured pose to A, then follows saved point order,
including the final point back to A. `--dir` changes the trajectory directory.

Continuous capture uses a separate version 1 JSON schema with motor IDs 15-28,
the captured start pose, and per-frame monotonic relative time plus measured q
and dq. `--continuous` selects this format without changing point JSON handling.
Relative replay anchors the recording's initial pose at the live measured pose
captured by WAIT_STABLE/Golden acquire, interpolates using saved timestamps, and
scales elapsed playback time by `--speed-scale` (default 0.25) and displacement
by `--motion-scale` (default 1.0). Each completed cycle returns smoothly to its
relative zero pose before repeating. Waist hold remains independent of arm data.

Adding `--high-precision` selects feedback-driven path progression while leaving
the original continuous replay as the baseline. The processor preserves the
shared 14-joint path with piecewise linear, no-overshoot resampling, retains
detected turning/pause points, writes `<name>_processed.json`, and rejects
geometric deviation above its 0.01 rad default bound. The retimer uses shared
progress, a 0.7 rad/s joint velocity limit, 1.5 rad/s^2 joint acceleration
limit, and the selected speed scale. Feedback uses 0.04 rad normal and 0.15 rad
pause error thresholds with a 0.12 s low-pass filter. Important nodes require
position error <= `--position-tolerance` (default 0.02 rad), dq <= 0.04 rad/s,
and 0.20 s stable feedback. `--node-timeout` defaults to 5 s; failure logs
`NOT_REACHED` and ends replay through the shared release path. Runtime feedback
CSV is written as `logs/g1_high_precision_<local-time>.csv`; node outcomes and
per-joint node errors are appended as comments in the same file. Each cycle
plays the forward path and then feedback-controls a smooth return to its relative
zero pose. Ctrl+C uses the existing controller release path.

## Control values

- Rate: 50 Hz. Acquire and release use the Golden 2 second smoothstep behavior.
- Golden first frame: motor 12-28 q values are captured from live LowState and
  ownership weight is 1.0. The motor 14 +0.010 rad bias fades in during acquire
  and fades out during release. A final weight 0 frame is always attempted.
- LOCK: arm Kp 40, Kd 1.5, tau_ff 0.
- DRAG: shoulder/elbow Kp 8, wrist Kp 6, Kd 1.5, tau_ff 0. Measured-q follow
  updates a reference by 0.25 of the error when error exceeds 0.015 rad.
- Waist: motor 12/13/14 Kp 80/80/120, Kd 1.5, tau_ff 0. Its reference is
  captured once and never follows arm motion; motor 14 uses the static +0.010 rad
  baseline plus the optional CLI bias. No BODY IMU posture outer loop runs in this
  command path; IMU data is used for WAIT_STABLE only.
- Joint sanity limits use the local G1 29-DoF URDF at
  `/home/hebe/unitree_workspace/unitree_ros/robots/g1_description/g1_29dof.urdf`.
  Confirm the model revision and field limits against the deployed robot before
  enabling motion.

## Exit and logs

`S`, `Q`, Ctrl+C, normal completion, and recoverable application errors share
the release path. Release sends a 2 second smooth weight reduction and a final
weight 0 frame. CSV logs are written under `logs/`, including waist measured q,
dq, reference, and maximum absolute reference error.
Log filenames use local wall time, for example `g1_arm_2026-10-14_15-30-00-123.csv`.
The paired JSONL event log records `WAIT_STABLE`, `STABLE_CONFIRMED`,
`ACQUIRE_START`, and `RELEASE_COMPLETE` with filtered motion indicators and
the exact captured references.

The initial stability limits are filtered roll/pitch gyro magnitude <= 0.30
rad/s, max waist motor dq <= 0.15 rad/s, leg motor dq RMS <= 0.30 rad/s, and
leg motor dq peak <= 0.80 rad/s. A 0.20 second low-pass filter is applied before
requiring the full 1 second window. The archived field logs do not contain raw
LowState IMU/leg dq samples, so these thresholds should be evaluated from the
new JSONL event logs during the first supervised field tests. A brief filtered
limit excursion is tolerated; three consecutive over-limit samples reset the
continuous window.

The program requires exclusive ownership of the robot's Arm SDK channel. Keep
the native locomotion controller responsible for the legs. No real-robot test
is part of the offline build or test suite.

Existing NPZ sessions, candidate data, waypoint YAML, and historical logs are
retained as research artifacts. The executable does not load NPZ or require a
Python runtime. Point files use JSON version 1 with 14 absolute arm angles in
radians and the three captured waist reference angles.
