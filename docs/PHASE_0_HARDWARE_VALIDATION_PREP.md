# PHASE_0_HARDWARE_VALIDATION_PREP

本阶段只准备 G1 29DoF Arm SDK 接管验证，不进入 Phase 2，不执行 target motion。
本轮没有连接真机；所有配置中的确认项必须由操作者依据目标硬件填写。

## 1. 只读核对

```bash
python3 scripts/phase0_hardware_audit.py --real --interface <iface> --duration 5 --log logs/audit_001.jsonl
```

该入口仅订阅 `rt/lowstate`，不创建 `rt/arm_sdk` publisher。打印并记录网卡、收包状态、
DDS callback 统计频率（非 Python polling 频率）、mode_machine、motor 12/13/14 q、
15–28 q/dq、配置 DoF 及人工审核状态。单个样本无法计算频率时为 null。
模拟模式明确标记 simulation，LOWSTATE_RECEIVED 为 false，频率为 null。
连接或收包失败返回非零退出码；不能解释为硬件审核成功。

motor 12=Yaw、13=Roll、14=Pitch；15–21 左臂、22–28 右臂、29 权重。
DoF=29 并不证明 motor 13/14 可用，静止或零读数也不证明锁定。

人工检查后在 `robot.yaml` 记录：

- `robot.model_confirmed: true`、`robot.arm_joint_mapping_verified: true`。
- `waist.configuration_verified: true`，并填写完整且无重叠的 active/locked 分区。
- ACTIVE_3DOF：active `[12,13,14]`，locked `[]`。
- YAW_ONLY：active `[12]`，locked `[13,14]`。
- LOCKED_VARIANT：13/14 必须在 locked 中；12 是否有效也需人工确认并归入相应列表。
- 默认 UNKNOWN、空列表、verified=false；真实 acquire 被拒绝。

核对型号、网卡、固件、增益、50 Hz 周期及基本 sanity 阈值后，才设置
`robot.hardware_reviewed: true`。这不能替代腰部审核或正式限位审核。

## 2. 限位与两种授权范围

`arm_limits.yaml` 的 `metadata.source: UNVERIFIED`、`hardware_verified: false` 和每个
joint 的 `verified: false` 不得用于真实 waypoint。min/max 默认 null，不猜测硬件角度限位。
14 个臂关节和实际参与控制的腰部关节均需有已审核的唯一 motor_id、有限且 min < max 的边界。
真实 motion 使用这些审核条目，不使用旧 q_min/q_max 模拟范围。

CURRENT_POSE_ONLY 是明确受限的 Phase 0 例外：可在正式限位尚未审核时进行 HOLD，
但仍要求型号/映射/腰部/基础硬件审核和独占控制确认。sanity_abs_rad 只是错误数据过滤阈值，
**不是物理安全运动限位**，需结合目标传感器单位审核。
此模式禁止传入任何目标 reference、waypoint 或轨迹；发布边界强制 q=acquire_q、dq=0、tau_ff=0。

Phase 1 真实运动还需 `ready_for_phase1_real_joint_motion: true`，仅在人工审查 Phase 0 报告且
`READY_FOR_PHASE1_REAL_JOINT_MOTION = YES` 后设置。程序不会根据一次实验自动修改此值。

## 3. 单级权重 HOLD

以下命令只供人工操作，未自动执行：

```bash
python3 scripts/phase0_acquire_hold_test.py \
  --real --interface <iface> --exclusive-control-confirmed \
  --acquire-hold-only --weight 0.10 --duration 3 \
  --waist-mode SEND_ACQUIRE_REFERENCE --log logs/hold_A_w010_001.jsonl
```

四个显式参数 `--real --interface --exclusive-control-confirmed --acquire-hold-only` 缺一不可。
默认目标 weight=0.1，限制 0 < weight <= 1。一次只运行一个级别。
依次 0.10、0.25、0.50、1.00 **由人工逐轮检查日志后决定**；代码不会连续执行四级。

等待稳定 lowstate：至少三个不同时间戳样本，持续 stable_state_s，位置跨度与速度均低于配置阈值，
超时退出。随后重新读取 acquire_q，固定整个 ramp/HOLD/release 的 q，smoothstep 增加权重，
HOLD 后 smoothstep 降至零。没有 HOME、zero posture 或预定义姿态指令。

`PHASE0_SAFE_BASELINE_GAIN` 是本阶段配置档案名，不是已认证安全的增益。
Kp/Kd 沿用原保守配置，未提高，全部写入日志。不调参、不自动 gain tuning。

## 4. 两个独立腰部实验

A：`--waist-mode SEND_ACQUIRE_REFERENCE`，只对人工确认的 active joints 发送 acquire 腰部 q 和配置增益。
B：`--waist-mode ZERO_GAIN_NO_COMMAND`，所有腰部 slot 维持零增益/零字段，不主动 HOLD。
分别使用不同日志文件。B 不保证腰部维持姿态；A/B 都不能主动指令锁定的 13/14。
程序不自动比较、不推断构型、不永久更改默认配置，也不因锁定关节没有响应判定 Arm SDK 失败。
旧 `send_commands: false` 也会关闭腰部命令；实验 A 应保持 true。

## 5. 日志与 release 验证

每个成功发布周期记录 timestamp、weight、q_measured/q_command、dq_measured、command-measured error、
waist_q_measured/waist_q_command、state_age、loop_dt。非主动控制的腰部 command 项为 null。
第一帧没有上一周期，loop_dt=null。初始 config 事件包含增益、构型、审核项和权重级别。

同名 `.report.json` 计算接管最大误差/速度、最大命令步长、周期均值/最大值及最大状态年龄。
ACQUIRE_POSITION_JUMP_MAX_RAD 表示接管期实测 q 相对 acquire_q 的最大绝对偏移，不是逐样本差分。
发布日志不是机器人接管 ACK，不会将 DDS Write 成功等同于获得控制权。

释放前记录 WEIGHT_BEFORE_RELEASE、Q_BEFORE_RELEASE，权重为零后继续只读观察 release_observe_s，
记录 WEIGHT_AFTER_RELEASE、Q_AFTER_RELEASE、DQ_PEAK_AFTER_RELEASE。
RELEASE_CAUSES_LARGE_TRANSIENT、HOLD_STABLE、RELEASE_STABLE 保留 UNKNOWN，由人工结合轨迹判定。
UNKNOWN 不得当作 YES。异常报告含 ABORT_REASON；不自动放行下一阶段。

Ctrl+C、超时、校验或 DDS 故障停止轨迹，尽力发送一次零权重 disable 后关闭 publisher。
失联/强杀时不能保证 disable 送达。真实释放行为由 Phase 0 人工确认。

## 工程边界

`test_armsdk.py` 原始腕部摆动代码保存在 `docs/reference/test_armsdk.py.txt`，仅供参考。
其可执行入口已停用，避免绕过单 publisher、审核门禁及 HOLD-only 限制。
所有实际发布仍通过 `sdk/transport.py` 的唯一 publisher；单机协作锁不能排斥外部控制程序。

离线验证：`python3 -m pytest -q`、`python3 -m compileall -q src scripts tests`。
不加 `--real` 的 audit/HOLD 可离线验证日志格式，但任何模拟数据都不是硬件证据。
