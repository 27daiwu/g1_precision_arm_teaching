"""Per-command telemetry and conservative hardware experiment reports."""
import json
import time
from pathlib import Path
import numpy as np
from ..utils.hardware import active_waist


class HardwareTelemetry:
    def __init__(self, path, client):
        self.client = client
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x', encoding='utf-8')
        self.release = {}
        self.previous_q = None
        self.summary = dict(MAX_ABS_ERROR_DURING_ACQUIRE=0., MAX_ABS_DQ_DURING_ACQUIRE=0.,
                            MAX_COMMAND_STEP=0., STATE_AGE_MAX=0.)
        self.dts = []
        self.origin = None
        self.jump = 0.
        self.write(dict(event='configuration', backend='hardware' if client.real else 'simulation',
                        config=client.config, weight_target=client.weight_target,
                        mode='PHASE0_OFFICIAL_STYLE_HOLD', limits_mode='CURRENT_POSE_ONLY',
                        ACQUIRE_STRATEGY='FULL_WEIGHT_CURRENT_POSE', ownership_weight=client.weight_target,
                        control_gain=client.config['phase0_hold']))

    def write(self, row):
        self.stream.write(json.dumps(row, allow_nan=False) + '\n')
        self.stream.flush()

    def cycle(self, state, command, weight, phase, dt):
        if state is None or state.dq is None:
            raise ValueError('telemetry requires measured state and dq')
        now = time.monotonic()
        error = command.q - state.q
        step = 0. if self.previous_q is None else float(np.max(np.abs(command.q - self.previous_q)))
        self.previous_q = command.q.copy()
        if self.origin is None:
            self.origin = self.client.acquire_q.copy()
        if phase == 'acquire':
            self.summary['MAX_ABS_ERROR_DURING_ACQUIRE'] = max(self.summary['MAX_ABS_ERROR_DURING_ACQUIRE'], float(np.max(np.abs(error))))
            self.summary['MAX_ABS_DQ_DURING_ACQUIRE'] = max(self.summary['MAX_ABS_DQ_DURING_ACQUIRE'], float(np.max(np.abs(state.dq))))
            self.jump = max(self.jump, float(np.max(np.abs(state.q - self.origin))))
        age = now - state.timestamp
        self.summary['MAX_COMMAND_STEP'] = max(self.summary['MAX_COMMAND_STEP'], step)
        self.summary['STATE_AGE_MAX'] = max(self.summary['STATE_AGE_MAX'], age)
        if dt is not None:
            self.dts.append(dt)
        waist = [float(self.client.acquire_waist_q[i-12]) if i in active_waist(self.client.config) else None for i in (12, 13, 14)]
        row = dict(timestamp=now, phase=phase, arm_sdk_weight=weight, ownership_weight=weight,
                   q_measured=state.q.tolist(), q_command=command.q.tolist(), dq_measured=state.dq.tolist(),
                   error=error.tolist(), waist_q_measured=state.waist_q.tolist(), waist_dq_measured=state.waist_dq.tolist(),
                   waist_delta_q=(state.waist_q - self.client.acquire_waist_q).tolist(),
                   waist_q_command=[None, None, None],
                   state_age=age, loop_dt=dt, acquire_q=self.origin.tolist(),
                   WAIST_COMMAND_SENT=[],
                   WAIST_COMMAND_REQUIRED='UNKNOWN', WAIST_HOLD_BEHAVIOR='UNKNOWN')
        self.write(row)

    def before_release(self, state, weight):
        self.release.update(WEIGHT_BEFORE_RELEASE=weight, Q_BEFORE_RELEASE=state.q.tolist())
        self.write(dict(event='before_release', timestamp=time.monotonic(), **self.release))

    def after_release(self, state, weight):
        self.summary['STATE_AGE_MAX'] = max(self.summary['STATE_AGE_MAX'], time.monotonic() - state.timestamp)
        peak = max(self.release.get('DQ_PEAK_AFTER_RELEASE', 0.), float(np.max(np.abs(state.dq))))
        self.release.update(WEIGHT_AFTER_RELEASE=weight, Q_AFTER_RELEASE=state.q.tolist(), DQ_PEAK_AFTER_RELEASE=peak)
        self.write(dict(event='post_release', timestamp=time.monotonic(), dq_measured=state.dq.tolist(),
                        waist_q_measured=state.waist_q.tolist(), state_age=time.monotonic()-state.timestamp,
                        **self.release))

    def close(self):
        self.summary.update(CONTROL_LOOP_DT_MEAN=float(np.mean(self.dts)) if self.dts else None,
                            CONTROL_LOOP_DT_MAX=max(self.dts) if self.dts else None)
        real = self.client.real
        report = dict(REPORT='PHASE_0_HARDWARE_VALIDATION_REPORT',
                      LOWSTATE_REAL_RECEIVED='YES' if real else 'NO',
                      RT_ARM_SDK_REAL_PUBLISHER_CREATED='YES' if real else 'NO',
                      ROBOT_MODEL_CONFIRMED='YES' if self.client.config['robot']['model_confirmed'] else 'NO',
                      ROBOT_DOF=self.client.config['robot']['dof'],
                      WAIST_CONFIGURATION=self.client.config['waist']['configuration'],
                      ARM_JOINT_MAPPING_VERIFIED='YES' if self.client.config['robot']['arm_joint_mapping_verified'] else 'NO',
                      ARM_SDK_WEIGHT_BEHAVIOR_VERIFIED='NO', WEIGHT_LEVEL_TESTED=self.client.weight_target,
                      ACQUIRE_CURRENT_POSE_ONLY='YES', ACQUIRE_POSITION_JUMP_MAX_RAD=self.jump if self.origin is not None else None,
                      ACQUIRE_VELOCITY_PEAK_RAD_S=self.summary['MAX_ABS_DQ_DURING_ACQUIRE'] if self.origin is not None else None,
                      HOLD_STABLE='UNKNOWN', RELEASE_STABLE='UNKNOWN', RELEASE_CAUSES_LARGE_TRANSIENT='UNKNOWN',
                      WAIST_COMMAND_REQUIRED='UNKNOWN', WAIST_HOLD_BEHAVIOR='UNKNOWN', TAU_FF=0,
                      REAL_TARGET_MOTION_EXECUTED='NO', WAYPOINT_EXECUTED='NO', READY_FOR_PHASE1_REAL_JOINT_MOTION='NO',
                      FIRST_WRITE_DIAGNOSTICS=self.client.first_write_diagnostics,
                      ACQUIRE_STRATEGY='FULL_WEIGHT_CURRENT_POSE',
                      ABORT_REASON=self.client._fault, BACKEND='hardware' if real else 'simulation',
                      **self.summary, **self.release)
        try:
            self.write(dict(event='summary', **report))
            self.path.with_suffix('.report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        finally:
            self.stream.close()
