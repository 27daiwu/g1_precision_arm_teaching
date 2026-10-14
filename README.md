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
./build/g1_record my_track --interface eth0
```

Commands are line based:

- `F`: toggle DRAG and LOCK.
- `M`: capture the measured 14-joint pose, record the next point, and enter LOCK.
- `L`: list points and joint angles.
- `R B`: replace point B with the current measured pose and enter LOCK.
- `D B`: delete point B.
- `N new_name`: rename the trajectory.
- `S`: save to `trajectories/my_track.json`, then release.
- `Q`: discard unsaved edits, then release.

## Replay

```sh
./build/g1_replay --list
./build/g1_replay my_track --preview
./build/g1_replay my_track --interface eth0 --cycles 3 --hold 2
./build/g1_replay my_track --interface eth0 --forever --speed 0.5
```

`--hold` overrides the hold at every point. Without it, each point's `hold_s`
is used. `--speed` is the maximum joint-space speed in rad/s. Replay first
moves smoothly from the measured pose to A, then follows saved point order,
including the final point back to A. `--dir` changes the trajectory directory.

## Control values

- Rate: 50 Hz. Acquire and release use the Golden 2 second smoothstep behavior.
- Golden first frame: motor 12-28 q values are captured from live LowState and
  ownership weight is 1.0. The motor 14 +0.010 rad bias fades in during acquire
  and fades out during release. A final weight 0 frame is always attempted.
- LOCK: arm Kp 40, Kd 1.5, tau_ff 0.
- DRAG: shoulder/elbow Kp 8, wrist Kp 6, Kd 1.5, tau_ff 0. Measured-q follow
  updates a reference by 0.25 of the error when error exceeds 0.015 rad.
- Waist: motor 12/13/14 Kp 80/80/120, Kd 1.5, tau_ff 0. Its reference is
  captured once and never follows arm motion; motor 14 uses the static bias above.
- Joint sanity limits use the local G1 29-DoF URDF at
  `/home/hebe/unitree_workspace/unitree_ros/robots/g1_description/g1_29dof.urdf`.
  Confirm the model revision and field limits against the deployed robot before
  enabling motion.

## Exit and logs

`S`, `Q`, Ctrl+C, normal completion, and recoverable application errors share
the release path. Release sends a 2 second smooth weight reduction and a final
weight 0 frame. CSV logs are written under `logs/`, including waist measured q,
dq, reference, and maximum absolute reference error.

The program requires exclusive ownership of the robot's Arm SDK channel. Keep
the native locomotion controller responsible for the legs. No real-robot test
is part of the offline build or test suite.

Existing NPZ sessions, candidate data, waypoint YAML, and historical logs are
retained as research artifacts. The executable does not load NPZ or require a
Python runtime. Point files use JSON version 1 with 14 absolute arm angles in
radians and the three captured waist reference angles.
