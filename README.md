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
HOLD_AT_ACQUIRE_POSE
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

## Phase 0 真机验证准备（当前阶段）

当前阶段为 **PHASE_0_HARDWARE_VALIDATION_PREP**。未进入 Phase 2，未启动真机。
腰部构型默认 UNKNOWN，正式限位默认未审核，真实 acquire 和 Phase 1 motion 分别由门禁阻止。
新增只读入口 `scripts/phase0_hardware_audit.py`，HOLD 入口要求显式
`--acquire-hold-only`，默认单次目标权重 0.10。

完整人工审核步骤、分级权重命令、腰部 A/B 实验及 telemetry 字段见
[Phase 0 硬件验证准备](docs/PHASE_0_HARDWARE_VALIDATION_PREP.md)。
当前结果见 [PHASE_0_HARDWARE_VALIDATION_REPORT.md](PHASE_0_HARDWARE_VALIDATION_REPORT.md)。

## 数据和测试

`logs/*.jsonl` 保存时间、阶段、实测 q、参考 q、acquire 腰部 q 以及
`WAIST_COMMAND_SENT`、`WAIST_STATE_RESPONSE`、`WAIST_HOLD_BEHAVIOR`；可通过 `--log` 指定新文件。
`evaluation/joint_error_metrics.py` 提供编码器空间 bias、MAE、RMSE、最大绝对误差，
及多次到达同一点后每次稳定位置的标准差/极差；它不代表外部测量的末端绝对精度。

测试覆盖 SDK wire mapping（fake DDS，不导入真机 SDK）、接管/释放、单 publisher、watchdog、
DDS 异常、waypoint 序列化/验证、速度/步长/限位、固定腰部、精确轨迹端点和四个离线 CLI。
后续 Phase 3 真机精度、稳定性、重复性和安全验证通过前，不扩展 Cartesian 或补偿控制。

历史基线见 [PHASE_0_1_IMPLEMENTATION_REPORT.md](PHASE_0_1_IMPLEMENTATION_REPORT.md)；当前状态见 [硬件验证报告](PHASE_0_HARDWARE_VALIDATION_REPORT.md)。
