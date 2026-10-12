# BODY IMU 低增益闭环离线审计（2026-10-10）

审计日志：`logs/dual_arm_teach_1791638375433505367.jsonl`。
本次仅读取日志、修改代码并运行离线测试，没有启动机器人或发送 DDS 指令。

## 现场运行证据

- 4023 次 TEACH DDS 写入全部 SUCCESS，3 个 waypoint。
- 运行摘要：无硬安全中止、无 DDS 写入失败，最后一次释放写入成功且 weight=0。
- 事后完整 wire audit 为 PASS。
- 403 条 BODY_ATTITUDE_SHADOW_V1：两个轴 candidate 均等于 `0.5 * attitude_error`。
- 403 条日志中 motor13 指令等于 acquire_q13 + candidate，最大残差 0；
  motor14 指令等于 acquire_q14 + 0.010 + candidate，最大残差 1.73e-18 rad。
- 这些采样记录中最大修正绝对值：motor13 0.01738705524 rad，motor14 0.03427817288 rad。
  以上最大值仅代表 403 条约 5 Hz 姿态日志，不声称覆盖所有控制周期。

## 实际 q_ref 写入路径

`run_teach` 在进入 TEACH 时用同步 LowState IMU/腰关节计算 torso roll/pitch 参考。
每周期 `BodyAttitudeShadow.update(state)` 使用同一输入帧计算姿态误差、P 目标、限幅和限速候选；
闭环通过 `set_body_attitude_correction` 把两个候选存入 controller。
`_frame` 先执行 `fill_golden_frame`，再显式覆盖：

```text
motor13.q = q_hold[1] + body_attitude_correction[0]
motor14.q = q_hold[2] + waist_pitch_hold_bias * bias_progress
motor14.q += body_attitude_correction[1]
```

TEACH 的 bias_progress 为 1。后续 arm override 仅处理 motor15..28，未覆盖 13/14。
随后执行 runtime guard、CRC、状态检查，并把同一 LowCmd 交给 publisher.Write。
RELEASE 继续使用最后修正以及已达到的 pitch bias progress，修正不会清零。
完整 guard 和历史快照事后审计使用对应的修正值。

旧 STATE 在命令准备后、Write 前记录 motor14_command_ref，可以按时间关联 TEACH 写入；
不能直接用 STATE 数组索引当作 DDS sequence，因为存在额外的捕获状态日志。
旧日志未逐周期记录 motor13 q_ref，不能声称已从历史日志完整核验 motor13 的每次实际写入。

## DDS sequence 3616 → 3617

| 项目 | 数值 |
| --- | --- |
| motor14 q_ref（3616） | 0.0032666785293713405 rad |
| motor14 q_ref（3617） | 0.001143117041527912 rad |
| 指令差 | −0.0021235614878434284 rad |
| Write start 差 | 0.02087597699573962 s |
| 按 Write start 计算的绝对变化率 | 0.10172273557672562 rad/s |
| 满足 0.1 rad/s 所需的最小 limiter dt | 0.021235614878434284 s |

现有 limiter dt 是本次 `update` 输入 `state.timestamp` 减上次 `update` 输入时间戳，
不是固定 0.02 s，也不是 Write start 差。`_frame` 内的 `_observe(self._state())`
再次读取 LowState，旧 STATE 的 state_timestamp 属于这次读取，不一定是 limiter 输入帧。
两条相关 STATE 的 state_timestamp 差为 0.020821572994464077 s，不能代替 limiter dt。
采样与 Write 之间延迟的变化可以造成两种变化率不同；这是与代码一致的解释，
但旧日志缺少本次/上次 limiter 输入时间戳，无法精确验证该步是否满足源采样时间限速。
因此此处结论是“Write 时间变化率略超目标，源时间限速证据不足”，不是“限速已通过”。

## 新增证据与冻结范围

每周期 DDS_WRITE_RESULT 记录实际 LowCmd 的 motor13/14_command_ref 和 applied correction，
覆盖 ACQUIRE/HOLD/TEACH/RELEASE，并保留写入结果、sequence 和 Write 起止时间。
启用 Shadow 或 Closed-loop 后，每次 TEACH 写入额外携带独立复制的 `body_attitude`：
源 state_timestamp、上次源时间戳、limiter_dt_s、时间基准、torso 姿态/参考/误差、
candidate、限幅后的 P target、P 增益、clamp、rate limit 和 closed_loop 标志。
Shadow 的实际 applied correction 保持 0，便于与候选对照。
RELEASE 不附加过期的姿态样本，只记录实际保持的修正和命令。
STATE 增加 command_sequence 与 motor13_command_ref；原 5 Hz BODY_ATTITUDE_SHADOW_V1 保留。
上述记录仍走内存缓冲，不新增逐周期磁盘写入。

闭环启动/DRAG 提示明确显示 acquire reference + low-gain BODY attitude correction，
不再声称 reference unchanged。
P=0.5、无 D、±0.05 rad 限幅、0.1 rad/s 源时间限速、所有 Kp、安全边界和 arm teaching 行为均未改动。
本轮不会把源时间限速改成 Write 时间限速；若要求实际 Write 时间变化率严格不超过 0.1，
需要另行明确控制时间基准与实现设计。

## 复现与验证

```bash
python3 scripts/analyze_body_attitude_closed_loop.py logs/dual_arm_teach_1791638375433505367.jsonl
PYTHONPATH=src:tests python3 -m pytest -q
git diff --check
```

审计脚本不导入机器人 SDK。旧日志输出 limiter_audit=UNAVAILABLE_IN_LEGACY_LOG，
支持时间关联的 motor14 写入变化率；新日志支持逐周期源 dt、P/限幅/限速递推和 applied correction 检查。
29 项离线测试通过，包含真实 controller 到模拟 publisher 的命令/日志一致性与释放连续性、
重复采样时间戳、Shadow/Closed-loop 每周期证据及提示、源时间和 Write 时间差异及错误 dt 检出。
本轮新增日志及提示尚未经真机运行验证。
