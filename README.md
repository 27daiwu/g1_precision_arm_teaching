# G1 Dual Arm Teaching

```text
Current Stage:
Phase 0 / Phase 1

Control Backend:
Unitree Arm SDK / rt/arm_sdk

Control Space:
Joint Space

Cartesian Control:
NOT IMPLEMENTED

Gravity Compensation:
DISABLED

tau_ff:
0

Waist Policy:
Golden Phase 0: HOLD_AT_ACQUIRE_POSE (motor12..28, kp=40, kd=1.5)
```

这是独立的 `g1_dual_arm_teaching` 工程，位于当前 `g1_precision_arm_teaching` 目录。
本工程不是 LowCmd / UserCtrl controller，不使用 UserCtrl ownership，也不修改 seated-controller。
Unitree 官方 Arm SDK 本身使用 `LowCmd_` **消息类型**，本工程唯一命令 topic 是 `rt/arm_sdk`，
状态从 `rt/lowstate` 读取。未实现 Cartesian、FK/IK、QP、动力学补偿或 raw trajectory 回放。

## 离线运行

Python 3.10+；安装 `numpy`、`PyYAML`，测试需要 `pytest`。以下命令均在工程根目录执行：

```bash
python3 -m pip install -e '.[test]'
python3 scripts/phase0_arm_sdk_probe.py
python3 scripts/phase0_acquire_hold_test.py --duration 1
python3 scripts/phase1_joint_hold.py --duration 1
python3 scripts/phase1_joint_hold.py --interactive
python3 scripts/phase1_waypoint_demo.py --waypoint-a data/waypoints/point_001.yaml --waypoint-b data/waypoints/point_002.yaml
python3 -m pytest -q
python3 -m compileall -q src scripts tests
```

默认运行理想位置跟随模拟器，**不会连接机器人**。模拟器仅验证软件行为，不证明真实精度或稳定性。
示例 waypoint 仅用于模拟，不是安全 HOME 姿态。四个脚本共享配置、控制周期和 SDK wrapper。
交互模式持续 HOLD，可输入 waypoint 文件路径平滑移动；输入 `capture data/waypoints/my_point.yaml`
保存当前实测关节；输入 `quit` 或 EOF 渐变释放，Ctrl+C 进入 ABORT。
采点功能不会自动切换为可拖动/零力矩模式，人工摆位需由操作者在合适的机器人模式下完成。

## Phase 0 当前接管语义

当前默认基线为 `GOLDEN_UPPER_BODY_CURRENT_POSE_HOLD`，严格参考 `scripts/test_armsdk.py`：
冻结 motor12..28 当前位置，50 Hz、Kp=40、Kd=1.5、dq/tau=0；motor29 仅作 ownership weight。
接管使用 2 秒 smoothstep 0→1，默认 full-weight hold 0.5 秒，随后 2 秒 smoothstep 释放到 0，
补发明确零权重帧并等待 0.05 秒。全过程复用一个 LowCmd，所有字段写完后计算 CRC，再 Write。

```bash
# 离线模拟，不连接机器人
python3 scripts/phase0_acquire_hold_test.py --golden --hold-time 0.5
# 只读 LowState 与实际 SDK 首帧审计，不创建 publisher
python3 scripts/phase0_acquire_hold_test.py --real --interface eth0 --golden --dump-first-command
# 由操作者手动执行真机回归；interface 替换为实际网卡
python3 scripts/phase0_acquire_hold_test.py --real --interface eth0 --golden --execute --hold-time 0.5
```

`--real` 不等于执行许可；只有 `--real --execute` 才允许创建命令 publisher。
Golden 手臂 motor15..28 的 `abs(dq)>1.0 rad/s`、`abs(q-q_acquire)>0.05 rad` 现仅作诊断 warning：
不 latch abort，不提前 release，按 joint + reason 只打印首次告警，结束输出 `ARM_WARNING_SUMMARY`。
正常流程仍在完成 hold 后释放。stale、NaN/Inf、DDS 失败、SIGINT/SIGTERM 和异常保留中止/释放；
腰部 watchdog 与首次采集/首帧 PRE_ACQUIRE_MOTION 检查保持原样。详见
[手臂 warning 行为说明](docs/GOLDEN_ARM_WARNING_POLICY.md)。
日志记录 motor12..28（含重点 motor14）的 acquire/latest q、delta、dq、最大偏移和最大速度及每帧命令/CRC。
默认 Golden 参数冻结，不使用旧 `arm/waist/phase0_hold.kp/kd` 增益；配置中的 watchdog 阈值仍有效。

原 `ARM_ONLY_CURRENT_POSE_HOLD` 保留为 `EXPERIMENTAL_ARM_ONLY_VARIANT`，通过
`--experimental-arm-only`（或旧 `--official-style`）显式选择，不再是默认或 Golden。
其旧增益及首帧 full-weight 语义尚未改造为严格单变量 A/B，当前不用于 A/B 验收。

主动写 motor12..14 本身不是已确认的后倾根因。旧报告仅作历史记录。
最新复测已完成完整 acquire、满权重 HOLD 和 release，无 hard safety abort / DDS 失败。
当前阶段：`PHASE0 = CONTROL_FLOW_PASS / CONTROL_QUALITY_OPEN`。
motor14 HOLD 静态偏差约 −2.6°，实际发送约 34.7 Hz、最大间隔 109 ms，二者独立调查。
`WAIST_POSE_HOLD_QUALITY = NOT_PASS`、`COMMAND_RATE_50HZ = NOT_PASS`，暂不进入 waypoint。
已加入原始脚本 motor14 诊断与项目逐周期 profiling；最新版本采用首帧完整校验、逐帧轻量 guard 和退出审计，普通日志释放后写盘。
固定 sleep(0.02)、增益与控制曲线不变，优化后真实频率尚待复测，详见 [热路径优化报告](docs/GOLDEN_HOT_PATH_OPTIMIZATION_REPORT.md)。
原始脚本零偏移同姿态对照命令、字段口径和下一步顺序见
[控制质量 profiling 与 A/B 说明](docs/GOLDEN_CONTROL_QUALITY_PROFILING.md)。


## 数据和测试

`logs/*.jsonl` 保存时间、阶段、实测 q、参考 q、acquire 腰部 q 以及
`WAIST_COMMAND_SENT`、`WAIST_STATE_RESPONSE`、`WAIST_HOLD_BEHAVIOR`；可通过 `--log` 指定新文件。
`evaluation/joint_error_metrics.py` 提供编码器空间 bias、MAE、RMSE、最大绝对误差，
及多次到达同一点后每次稳定位置的标准差/极差；它不代表外部测量的末端绝对精度。

测试覆盖 SDK wire mapping（fake DDS，不导入真机 SDK）、接管/释放、单 publisher、watchdog、
DDS 异常、waypoint 序列化/验证、速度/步长/限位、腰部监测、精确轨迹端点和四个离线 CLI。
后续 Phase 3 真机精度、稳定性、重复性和安全验证通过前，不扩展 Cartesian 或补偿控制。

Golden Reference 的来源与限制见
[WORKING_TEST_ARMSDK_GOLDEN.md](docs/reference/WORKING_TEST_ARMSDK_GOLDEN.md)。
