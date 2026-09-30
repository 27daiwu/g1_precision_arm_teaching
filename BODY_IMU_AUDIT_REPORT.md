# BODY_IMU_AUDIT_REPORT

Date: 2026-09-29. Scope: static source audit and offline analysis only. No DDS connection or robot command was made.

| Item | Finding |
| --- | --- |
| BODY_IMU_SOURCE | `rt/lowstate` -> `LowState_.imu_state` (IDL confirmed); physical sensor/link **unconfirmed**. |
| QUATERNION_ORDER | `[w, x, y, z]`, confirmed by matching both recorded quaternion and `imu_state.rpy` with standard ZYX Euler conversion. |
| REFERENCE_FRAME | **Unconfirmed** on hardware. Recorded quaternion and RPY agree as ZYX orientation angles, but the IDL does not define world axes or heading convention. |
| BODY_FRAME | **Unconfirmed** on hardware. G1 MuJoCo model places an inertial site in pelvis and another in torso; this does not identify which physical sensor feeds `LowState_.imu_state`. |
| ROLL_SIGN | Mathematical positive rotation about quaternion body X; physical left/right tilt sign **unconfirmed**. |
| PITCH_SIGN | Mathematical positive rotation about quaternion body Y; physical forward/backward tilt sign **unconfirmed**. |
| GYRO_ROLL_AXIS | Likely body X (`gyroscope[0]`), but hardware correspondence/sign **unconfirmed**. Euler roll rate is not generally equal to gyro X when pitched/yawed. |
| GYRO_PITCH_AXIS | Likely body Y (`gyroscope[1]`), but hardware correspondence/sign **unconfirmed**. |
| UPDATE_RATE | DDS callback mean: 1017 Hz and 1129 Hz in two old recordings. Distinct IMU frame rate **unconfirmed** because 16.4% and 16.7% of adjacent quaternion samples repeat exactly; `tick` also repeats. |
| STATIC_NOISE | In old 10 s recording, for frames with all 29 `abs(dq)<0.03 rad/s`: RPY standard deviation `[0.000119, 0.000288, 0.000184] rad`, gyro standard deviation `[0.00442, 0.00467, 0.00945]` in SDK units. This is an approximate quiet-segment statistic, not a controlled static test. |
| OLD_PROJECT_IMU_LOGIC_FOUND | **YES, raw reading/recording only**. No validated quaternion conversion, coordinate convention, attitude filter, or gyro interpretation found. |
| READY_FOR_SHADOW_ATTITUDE_CONTROLLER | **NO**. Physical IMU link and movement signs need confirmation, and fresh stationary/stale-data evidence is missing. |

## Evidence and limits

- Local SDK IDL: `/home/hebe/unitree_workspace/unitree_sdk2_python/unitree_sdk2py/idl/unitree_hg/msg/dds_/_LowState_.py` and `_IMUState_.py` define `imu_state` with `quaternion[4]`, `gyroscope[3]`, `accelerometer[3]`, and `rpy[3]`. The IDL provides no frame, units, timestamp, or validity flag.
- Existing teaching transport, `src/g1_dual_arm_teaching/sdk/transport.py`, currently extracts joints and callback receipt time only. Its `LOWSTATE_FREQUENCY` counts DDS callbacks, not distinct IMU measurements. It does not expose IMU state to the controller.
- Old project requested at `/home/hebe/zjy_ws/src/unitree_g1_control` is absent; the available tree is `/home/hebe/unitree_g1_control`. Its `src/g1_piano/recorder/state_recorder.py` copies IMU arrays and monotonic receipt time, and `src/g1_piano/monitor/state_monitor.py` prints raw values. Neither implements a validated attitude controller.
- Offline files `/home/hebe/unitree_g1_control/data/raw/g1_test_001.npz` (10,165 samples, 9.992 s) and `g1_demo_001.npz` (22,572 samples, 20.000 s) have finite IMU arrays and quaternion norms near 1. ZYX conversion treating index 0 as `w` matches recorded RPY with RMS errors below `7e-8 rad` on each axis. Treating index 3 as `w` fails markedly. These files establish numerical convention for those recordings, not physical mounting.
- `g1_test_001.npz` quaternion/RPY repeat on 1,670 of 10,164 adjacent callback pairs; `g1_demo_001.npz` repeats on 3,776 of 22,571. Maximum callback gaps are about 12 ms and 10 ms respectively. Neither file contains independent IMU generation timestamps, so stale-versus-held samples cannot be separated reliably.
- The G1 MuJoCo `assets/robots/unitree_g1/xmls/g1.xml` defines `imu_in_pelvis` and `imu_in_torso`, but its gyro/accelerometer sensors reference `imu_in_pelvis`. The model is suggestive, not proof of the real LowState source. The SDK H2 example calls its LowState IMU `pelvis`, but that is a different robot.

## Required before shadow mode

Use a **read-only** LowState capture on the actual G1, performed by the onsite operator. Record quaternion, RPY, gyro, accelerometer, `tick`, receipt monotonic time, motor12..14 q/dq, and operating mode in each callback. Confirm the IMU's physical mounting from G1 hardware documentation or a controlled pelvis-versus-torso motion observation. With the robot supported and no controller changes, capture clearly labeled small positive/negative roll and pitch motions to establish physical signs and gyro correlation, then a stationary interval to measure noise, repeated frames, gaps, and invalid/stale behavior. Check whether torso attitude must be derived from a pelvis IMU plus waist joint kinematics. Only then decide whether a roll/pitch shadow calculation is meaningful. No correction should be sent to motor13/14 in this audit stage.

## WAIST_KINEMATICS_AUDIT

Source: `/home/hebe/unitree_workspace/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/g1.xml`, lines 61-69 and 154-170. This is a model audit (`MODEL_ONLY`), not validation of hardware geometry or the LowState IMU mounting (`HARDWARE_UNCONFIRMED`). The current project contains no URDF/MJCF robot description.

| Motor | Joint | Parent -> child | Child/joint origin relative to parent | Axis in joint frame |
| --- | --- | --- | --- | --- |
| 12 | `waist_yaw_joint` | `pelvis` -> `waist_yaw_link` | `pos=(0,0,0)`, identity rotation (omitted attributes) | `(0,0,1)` |
| 13 | `waist_roll_joint` | `waist_yaw_link` -> `waist_roll_link` | `pos=(-0.0039635,0,0.044)`, identity rotation | `(1,0,0)` |
| 14 | `waist_pitch_joint` | `waist_roll_link` -> `torso_link` | `pos=(0,0,0)`, identity rotation (omitted attributes) | `(0,1,0)` |

The exact model chain is `pelvis -> waist_yaw_link -> waist_roll_link -> torso_link`. With model joint frames, `R_pelvis_torso = Rz(q12) Rx(q13) Ry(q14)`. Translation includes the motor13 origin offset. The model pelvis and torso IMU sites have positions `(0.04525,0,-0.08339)` and `(-0.03959,-0.00224,0.14792)` respectively; their omitted `quat` attributes mean identity site rotation in their parent body frames. Thus pelvis-to-torso rotation is derivable from the model by three-dimensional rotation composition. `R_world_torso = R_world_imu_link R_imu_link_to_torso(q12,q13,q14)` is usable only after the physical LowState IMU link, its axes, and its fixed mounting rotation are established. Euler-angle addition is invalid for this chain. The model's `imu_in_pelvis` sensor declaration does not establish the real LowState source.

## Hardware Semantics Capture

The onsite operator may run `python scripts/body_imu_semantics_capture.py eth0 --label static --duration 30`; other labels are `whole_body_roll`, `whole_body_pitch`, `torso_relative_roll`, and `torso_relative_pitch`. Labels describe observations, never actions. The tool only subscribes to `rt/lowstate`, writes raw `.npz` plus `.json` under `logs/body_imu_audit/`, and closes the subscriber on Ctrl+C. `python scripts/analyze_body_imu_semantics.py logs/body_imu_audit/<capture>.npz` reads saved data only. Callback frequency is not an independent IMU update rate. Exact repeats may be held frames or a stationary/quantized sensor; without an IMU generation timestamp, a stale/fresh conclusion needs controlled motion evidence and callback/tick analysis.

For whole-body roll and pitch, independently note the physical positive/negative movement and compare signed RPY change with the gyro response. Euler derivatives and body angular velocity differ outside small-angle, low yaw/pitch conditions; use their correlation as limited directional evidence. For torso-relative roll/pitch, independently verify that the pelvis stayed nearly fixed while motor13/14 changed noticeably. If IMU RPY tracks this relative torso motion, the evidence favors `BODY_IMU_LINK=TORSO`; if IMU RPY stays approximately fixed, it favors `BODY_IMU_LINK=PELVIS`. If pelvis movement or sensor drift is comparable to the observed change, the result is inconclusive. Do not classify hardware from the model site name alone.

Hardware link, physical roll/pitch signs, independent IMU rate, and fresh/held/stale behavior remained unconfirmed at the capture-tool stage. The new onsite analysis follows below.

## New Hardware Semantics Capture (2026-09-30)

The five files below are one complete onsite sequence, distinct from the old recordings above. UTC timestamps and counts come from their JSON metadata. Only their saved `.npz` data were analyzed; no DDS connection was made in this analysis.

| Label | File stem | Captured UTC | Duration (s) | Samples |
| --- | --- | --- | ---: | ---: |
| static | `20260930T035802Z_static` | 03:58:02 | 29.935 | 32,552 |
| whole_body_roll | `20260930T035846Z_whole_body_roll` | 03:58:46 | 19.939 | 21,705 |
| whole_body_pitch | `20260930T035931Z_whole_body_pitch` | 03:59:31 | 19.921 | 20,751 |
| torso_relative_roll | `20260930T040228Z_torso_relative_roll` | 04:02:28 | 19.923 | 20,754 |
| torso_relative_pitch | `20260930T040251Z_torso_relative_pitch` | 04:02:51 | 19.947 | 20,779 |

Analysis command: `python3 scripts/analyze_body_imu_semantics.py logs/body_imu_audit/*.npz`. The analyzer was extended to calculate peak-to-peak ranges, excursions relative to the initial 0.5 s median, full-series joint/IMU correlations, and 100 ms Euler-rate/gyro correlations. All five arrays are finite. The endpoint deltas in analyzer output are descriptive only; movement conclusions below use the time series.

### Static measurement

The explicitly labeled `static` capture lasted 29.935 s and had 32,552 callbacks (1087.4 Hz mean). Callback interval mean/std/p95/p99/max: `0.920/0.195/1.158/1.367/12.522 ms`. Quaternion norm mean/std/min/max: `1.000000025/3.79e-8/0.999999925/1.000000129`.

| Signal XYZ / RPY | Mean | Std | P95 | P99 |
| --- | --- | --- | --- | --- |
| RPY (rad) | `[-.010177,.017285,-.402557]` | `[.000252,.000372,.000231]` | `[-.009818,.017789,-.402122]` | `[-.009739,.017858,-.401945]` |
| Gyro (SDK units) | `[-.001599,.006990,-.000598]` | `[.004478,.004545,.006634]` | `[.005326,.014914,.010653]` | `[.008522,.017044,.015979]` |
| Accelerometer | `[-.266674,.019759,9.935426]` | `[.037669,.040585,.041210]` | - | - |

Adjacent exact duplicates: quaternion and RPY `17.047%` each, gyro `17.984%`; tick repeats `4.018%`. DDS callbacks exceed the distinct quaternion-value rate, but identical values alone cannot establish a fresh IMU generation rate. `HELD_OR_REPEATED_SAMPLE_OBSERVED=YES`; `TRUE_STALE_BEHAVIOR_CONFIRMED=NO`. The SDK IDL has no IMU generation timestamp.

### Motion evidence

The following are peak-to-peak ranges in radians for RPY and waist q, and SDK units for gyro. Correlation rows are from the entire time series; return toward center can make start/end delta small despite substantial excursions. Smoothed rate/gyro correlation uses 20 ms interpolation and a 100 ms derivative interval. Body gyro angular velocity is not generally an Euler derivative, especially when yaw and pitch also move.

| Label | RPY peak-to-peak | Gyro XYZ peak-to-peak | q12/13/14 peak-to-peak | Target joint vs target IMU angle corr | Target Euler rate vs gyro XYZ corr |
| --- | --- | --- | --- | ---: | --- |
| whole_body_roll | `[.154,.090,.478]` | `[.309,.471,.800]` | `[.095,.276,.172]` | q13/roll `+.727` | roll `[+.969,+.168,+.794]` |
| whole_body_pitch | `[.226,.927,.666]` | `[.523,1.357,1.121]` | `[.259,.186,.478]` | q14/pitch `-.814` | pitch `[-.055,+.991,-.232]` |
| torso_relative_roll | `[.051,.036,.139]` | `[.214,.271,.388]` | `[.147,.378,.080]` | q13/roll `-.310`; q13/pitch `-.281` | roll `[+.946,+.307,+.765]` |
| torso_relative_pitch | `[.026,.105,.070]` | `[.197,.405,.325]` | `[.053,.023,.356]` | q14/pitch `-.443`; q14/roll `+.883` | pitch `[-.140,+.945,+.150]` |

Whole-body roll excursions from initial median were roll `+0.121/-0.033 rad`, pitch `+0.000/-0.090`, yaw `+0.014/-0.465`; pitch capture excursions were pitch `+0.286/-0.640 rad`, roll `+0.206/-0.020`, yaw `+0.615/-0.051`. Roll derivative tracks gyro X most strongly (`+.969`), pitch derivative tracks gyro Y (`+.991`). These are strong *numerical axis/sign* relationships in these recordings, not physical left/right or forward/backward sign labels. The logs contain no timestamped physical direction annotations, and both whole-body trials include substantial yaw and waist motion. `LEFT_TILT_ROLL_SIGN`, `RIGHT_TILT_ROLL_SIGN`, `FORWARD_TILT_PITCH_SIGN`, and `BACKWARD_TILT_PITCH_SIGN` remain `UNCONFIRMED`.

In torso-relative roll, q13 spans `0.378 rad` while IMU roll spans `0.051 rad`; q12 also spans `0.147 rad` and IMU yaw `0.139 rad`. In torso-relative pitch, q14 spans `0.356 rad` while IMU pitch spans `0.105 rad`; q12 spans `0.053 rad`. The target q/angle correlations (`-.310`, `-.443`) are not compelling evidence of direct torso tracking. The pitch run also has strong q14/roll correlation (`+.883`), indicating coupled motion or changing pelvis orientation. No independent pelvis pose was logged. These trials show meaningful relative waist motion, but do not verify a fixed pelvis or isolate the IMU response. `TORSO_RELATIVE_ROLL_VALID=NO` and `TORSO_RELATIVE_PITCH_VALID=NO` for a definitive hardware-link classification. `HARDWARE_IMU_LINK=UNCONFIRMED`; `BODY_ATTITUDE_SOURCE=UNCONFIRMED`; `WAIST_FK_REQUIRED=UNCONFIRMED`. No pelvis-based torso attitude estimate was constructed because the pelvis IMU premise is unproven.

`READY_FOR_SHADOW_ATTITUDE_CONTROLLER=NO`; `READY_FOR_CLOSED_LOOP_ATTITUDE_CONTROLLER=NO`. The onsite operator would need to provide timestamped physical direction annotations and independent evidence that the pelvis stayed fixed during relative motions to resolve the remaining hardware semantics.

## PELVIS_VS_TORSO_IMU_DISAMBIGUATION preparation

No new robot run was performed for this preparation. The read-only capture tool now accepts `pelvis_fixed_torso_roll` and `pelvis_fixed_torso_pitch` labels. During capture, typing `LEFT`, `RIGHT`, `FORWARD`, `BACKWARD`, or `CENTER` followed by Enter records `{monotonic_timestamp, event}` in the companion JSON `event_markers` list. Timestamps are assigned when the terminal line is received, so operator reaction/typing delay must be considered. The original `.npz` IMU and joint arrays are unchanged. These markers annotate observations only and cannot prove pelvis fixation.

The offline analyzer adds `pelvis_fixed_experiment` output for those labels: target q13/roll or q14/pitch peak-to-peak, target correlation, q12/other-waist-joint coupling, and gyro XYZ ranges. Optionally pass `--pelvis-attitude external_pelvis.npz` for a single capture; the external file must contain `monotonic_receipt_time` (strictly increasing, same host clock, covering the full capture) and `rpy` with shape `(N,3)`. The analyzer reports aligned pelvis world-attitude peak-to-peak. This external record's sensor frame and calibration need separate verification. A label, marker, or q/IMU correlation alone never validates a fixed pelvis.

Classification rule for later onsite data: first establish approximately fixed pelvis world attitude independently and clear target waist-joint motion with limited coupling. Then compare the measured IMU orientation *relative to the measured pelvis orientation* against the waist motion over time. Consistent following in both roll and pitch experiments supports torso; approximately unchanged relative orientation supports pelvis. Contradictory or insufficient evidence remains `HARDWARE_IMU_LINK=UNCONFIRMED`. No automatic hardware-link classification or controller integration is enabled by these tools. `READY_FOR_SHADOW_ATTITUDE_CONTROLLER=NO`.

## PELVIS_VS_TORSO_HARDWARE_DISAMBIGUATION (2026-09-30)

### Experiment condition

The onsite operator reports that the robot was suspended, but the lower body may move slightly while the waist moves. Suspension is not a rigid pelvis world-attitude constraint. No independent pelvis attitude trace or measured bound on that motion was supplied. This is an experiment-condition report, not evidence contained in LowState. `PELVIS_FIXATION_SOURCE=SUSPENDED_ROBOT_WITH_POSSIBLE_LOWER_BODY_MOTION`; `PELVIS_FIXATION_INDEPENDENTLY_VERIFIED=NO`. The `pelvis_fixed_*` filenames do not establish fixation.

### Hardware log evidence

The latest complete pair is `20260930T041835Z_pelvis_fixed_torso_roll.npz` (metadata timestamp 04:18:35.645 UTC, 19.932 s, 21,646 samples) and `20260930T041903Z_pelvis_fixed_torso_pitch.npz` (04:19:03.956 UTC, 19.939 s, 21,349 samples). Both have companion JSON metadata, finite arrays, near-unit quaternion norms, and **no event markers**. Marker-based physical direction and event-window analysis are unavailable. Joint and IMU ranges below are peak-to-peak radians; gyro ranges are SDK units.

| Trial | Waist q12/q13/q14 p-p | IMU roll/pitch/yaw p-p | Gyro X/Y/Z p-p | Target q/angle corr | Cross-axis target corr | Follow ratio |
| --- | --- | --- | --- | ---: | ---: | ---: |
| Roll | `[.1724,.2511,.1280]` | `[.0495,.0583,.0700]` | `[.2812,.2418,.2684]` | q13/roll `-.090` | q13/pitch `+.015` | `.197` |
| Pitch | `[.0399,.0973,.5574]` | `[.0635,.1439,.2417]` | `[.1928,.3739,.4932]` | q14/pitch `-.649` | q14/roll `+.295` | `.258` |

Roll IMU excursions from the initial 0.5 s median: roll `+0.0141/-0.0355`, pitch `+0.0542/-0.0041` rad. Pitch IMU excursions: pitch `+0.1367/-0.0072`, roll `+0.0631/-0.0003` rad. At roll q13 extrema (6.09 and 8.90 s), 0.5 s local-window median q13 changes from `-0.0676` to `+0.1750` rad while IMU roll changes from `-0.0277` to `-0.0081` rad. At pitch q14 extrema (9.58 and 6.46 s), local-window median q14 changes from `-0.3688` to `+0.1852` rad while IMU pitch changes from `+0.0608` to `+0.0106` rad. These local comparisons show substantial relative-joint excursion without relying on endpoint delta. Smoothed Euler-roll-rate/gyro-X correlation is `+.969` in the roll trial; smoothed Euler-pitch-rate/gyro-Y correlation is `+.961` in the pitch trial. Physical tilt signs remain unconfirmed without markers.

The roll trial has substantial q12 and q14 motion relative to q13 (69% and 51% of q13 p-p). The pitch trial has smaller q13 motion (17% of q14 p-p) but IMU yaw changes by `.242 rad`. A small IMU target-axis response alone cannot identify the mounted link when the pelvis may move and joint axes couple. `ROLL_TEST_SIGNAL_QUALITY=WEAK` and `PITCH_TEST_SIGNAL_QUALITY=WEAK` for hardware-link discrimination.

### Model evidence and final conclusion

The MuJoCo model supplies the Z-X-Y waist chain audited above, but does not identify the hardware LowState IMU link. The hardware logs establish clear q13/q14 excursions and smaller corresponding IMU-angle excursions; they do not establish pelvis fixation or a unique physical IMU link. `HARDWARE_IMU_LINK=UNCONFIRMED`; `BODY_ATTITUDE_SOURCE=UNCONFIRMED`; `WAIST_FK_REQUIRED=UNCONFIRMED`. A pelvis-IMU-based torso FK estimate was not computed because the pelvis IMU premise is unproven. `TORSO_ATTITUDE_ESTIMATOR_OFFLINE_TESTED=NO`; `TORSO_ATTITUDE_ESTIMATOR_STATUS=UNCONFIRMED`; `READY_FOR_SHADOW_ATTITUDE_CONTROLLER=NO`.

# OFFICIAL_UNITREE_G1_IMU_SOFTWARE_SEMANTICS

Local official source audit: `/home/hebe/unitree_workspace/unitree_rl_mjlab/deploy/include/unitree_articulation.h:29-34` maps LowState IMU quaternion indices 0..3 to Eigen `root_quat_w` in WXYZ order. `deploy/robots/g1/src/State_Mimic.cpp:15-24` computes `root_quat * Rz(q12) * Rx(q13) * Ry(q14)` as torso quaternion. The G1 model `src/assets/robots/unitree_g1/xmls/g1.xml` confirms the Z-X-Y waist axes. These establish official software semantics; physical sensor PCB mounting is not independently confirmed.

```makefile
PHYSICAL_SENSOR_MOUNTING = HARDWARE_UNCONFIRMED
LOWSTATE_IMU_SOFTWARE_SEMANTICS = ROOT/PELVIS
LOWSTATE_QUATERNION_ORDER = WXYZ
EVIDENCE = UNITREE_OFFICIAL_DEPLOYMENT_CODE
BODY_ATTITUDE_SOURCE = PELVIS_IMU_PLUS_WAIST_FK
WAIST_ROTATION_ORDER = Z_X_Y
WAIST_FK_REQUIRED = YES
DIRECT_LOWSTATE_IMU_AS_TORSO = NO
```

Earlier `BODY_ATTITUDE_SOURCE=UNCONFIRMED` entries above record the pre-audit historical conclusion. The estimator and an independent rotation-matrix reference agree across seven saved captures, with maximum orientation error `2.9802322387695312e-08 rad`. All replays were offline, with no DDS or robot connection.

| Saved capture | Raw IMU target P2P (rad) | Estimated torso target P2P (rad) |
|---|---:|---:|
| torso_relative_roll (roll) | 0.050617 | 0.381262 |
| torso_relative_pitch (pitch) | 0.104503 | 0.381858 |
| pelvis_fixed_torso_roll (roll) | 0.049528 | 0.287101 |
| pelvis_fixed_torso_pitch (pitch) | 0.143886 | 0.511916 |

Also replayed static and whole-body roll/pitch captures from `logs/body_imu_audit/`. The named pelvis-fixed captures do not independently prove pelvis fixation. `READY_FOR_BODY_ATTITUDE_SHADOW_MODE=YES` applies only to eligibility for a future shadow stage. `READY_FOR_CLOSED_LOOP_ATTITUDE_CONTROLLER=NO`. No motor command path was modified.
