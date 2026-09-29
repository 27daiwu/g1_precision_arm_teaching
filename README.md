# G1 Dual Arm Teaching

## Overview

Current Mainline: Dual-arm explicit lock + joint waypoint teaching MVP.
按 `F` 进入双臂拖动模式，人工托住目标姿态，再按 `M` 锁定并记录 waypoint。

## Control Scope

- 命令后端：Unitree Arm SDK `rt/arm_sdk`；实测状态：`rt/lowstate`。
- motor12..14 保持接管时的腰部姿态；motor15..28 统一使用 LOCKED / DRAG 模式。
- motor29 用于 ownership weight；正式控制链不使用 UserCtrl。
- Gravity compensation: DISABLED。所有下发的 `tau_ff = 0`。

## Current Control Parameters

| 关节与状态 | Kp | Kd | 位置参考 |
| --- | --- | --- | --- |
| motor12 waist_yaw | 80 | 1.5 | 接管时腰部 q |
| motor13 waist_roll | 80 | 1.5 | 接管时腰部 q |
| motor14 waist_pitch | 120 | 1.5 | 接管时腰部 q + 0.010 rad |
| motor15..28 LOCKED | 40 | 1.5 | 锁定时捕获的 q |
| 肩肘 DRAG | 8 | 1.5 | 每周期实测 q |
| 手腕 DRAG | 6 | 1.5 | 每周期实测 q |

启动后双臂处于 LOCKED。`F` 是进入 DRAG 的唯一方式；`M` 将双臂重新锁定。
机器人运动、dq 和 position error 都不会自动切换模式。DRAG 中每个周期直接使用
`q_ref = measured_q`。按 `F` 仅降低双臂阻抗，不释放 Arm SDK ownership。
腰部 Kp80/80/120 贯穿 ACQUIRE、LOCKED、DRAG 和 RELEASE，直到 ownership
weight 降为 0。motor14 的 +0.010 rad reference bias 随 2 s acquire smoothstep
平滑进入，接管完成后在 HOLD、LOCKED、DRAG 和 RELEASE 中保持不变。
motor12/13 reference 和腰部 safety 基准仍是启动时的 acquire q；F/M 不重新采样腰部。

## Teaching Workflow

1. 启动程序，机器人接管当前姿态，以 Kp40 进入 LOCKED。
2. 按 `F` 进入双臂 DRAG，人工拖动并托住目标姿态。
3. 按 `M`。系统使用同一次 LowState capture 保存 motor15..28 waypoint、冻结双臂 reference，并统一切换为 LOCKED；Kp 在 0.4 s 内通过 smoothstep 恢复至 40。
4. 松手检查是否稳定保持。下一点再次按 `F`，拖动并按 `M`。
5. 按 `L` 查看 waypoint 数量；按 `Q` 正常 release 并退出。

## Run on Robot

仅由现场操作者在确认网卡后执行：

```bash
python scripts/dual_arm_teach.py eth0
```

## Controls

| 按键 | 操作 |
| --- | --- |
| `F` | 全部双臂关节进入低 Kp DRAG |
| `M` | 锁定双臂并保存 waypoint |
| `L` | 显示 waypoint 数量 |
| `Q` | 正常释放并退出 |

## Safety and Release

接管使用 2 s smoothstep。腰部 warning delta 为 0.05 rad，hard delta 为
0.08 rad，hard dq 为 0.5 rad/s。保留 arm safety、首帧保护、DDS/CRC 校验及
限时关闭。正常结束或中止时渐变释放 ownership，最终明确写入 weight=0。

## Logs and Waypoints

运行响应写入 `logs/dual_arm_teach_*.jsonl`；每次按 `M` 的 motor15..28
实测 waypoint 记录为 `JOINT_WAYPOINT_MARK` 事件。

## Offline Validation

```bash
python3 -m pip install -e '.[test]'
python3 -m pytest -q
python3 -m compileall -q src scripts tools tests
```

## Repository Layout

`scripts/` 只有正式示教入口。`tools/` 是历史、诊断和基线工具，不属于示教流程。
`docs/` 保留历史报告，`tests/` 提供离线与 mock 验证。

## Current Limitations

- 无 gravity compensation、Cartesian IK 或 playback。
- 腰部是阻抗保持，不是机械锁定。
- DRAG 状态下的手臂需要人工托住后再按 `M`。
