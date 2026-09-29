"""Observation-only Golden safety evidence and actual Write-call timing."""
import logging
import numpy as np

JOINT_NAMES = ('waist_yaw', 'waist_roll', 'waist_pitch') + tuple(
    side + joint for side in ('left_', 'right_') for joint in
    ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw'))


class SafetyAbort(RuntimeError):
    """Existing abort decision with structured evidence recorded separately."""


class GoldenDiagnostics:
    def __init__(self, config, clock, emit, real_dds=False):
        self.config, self.clock, self.emit = config, clock, emit
        self.real_dds = real_dds
        self.started = clock()
        self.capture_time = self.capture_state_time = None
        self.acquire_start = self.release_start = None
        self.first_trigger = None
        self.violation_samples = 0
        self.writes = []
        self.warnings = {}
        self.warning_count = 0
        self.q_min = np.full(17, np.inf)
        self.q_max = np.full(17, -np.inf)
        self.max_delta = np.zeros(17)
        self.max_dq = np.zeros(17)
        self.samples = 0
        self.waist_warning_count = 0
        self.waist_warning_first = None
        self.waist_max_abs_delta = 0.
        self.waist_max_abs_dq = 0.

    def inspect(self, state, q_hold, phase, weight, first):
        """Collect all conditions; only ARM_DELTA/VELOCITY use WARNING severity."""
        now = self.clock()
        age = None if state is None else now - state.timestamp
        available = state is not None and all(getattr(state, f) is not None for f in ('q', 'dq', 'waist_q', 'waist_dq'))
        q = np.concatenate((state.waist_q, state.q)) if available else None
        dq = np.concatenate((state.waist_dq, state.dq)) if available else None
        delta = (np.zeros(17) if q_hold is None else q-q_hold) if available else None
        violations = []
        waist = self.config['phase0_hold']['waist_watchdog']

        def add(reason, code, i=None, delta_limit=None, velocity_limit=None, **extra):
            violations.append(dict(phase=phase, reason=reason, abort_code=code,
                severity='WARNING' if code in ('ARM_DIAGNOSTIC_WARNING',
                                               'WAIST_DIAGNOSTIC_WARNING') else 'HARD',
                timestamp=now,
                motor_id=None if i is None else i+12,
                joint_name=None if i is None else JOINT_NAMES[i],
                q_acquire=None if i is None or q_hold is None else float(q_hold[i]),
                q_current=None if i is None else float(q[i]),
                delta_q=None if i is None or q_hold is None else float(delta[i]),
                dq=None if i is None else float(dq[i]),
                threshold_delta_q=delta_limit, threshold_dq=velocity_limit,
                state_age_ms=None if age is None else age*1000,
                weight=float(weight), elapsed_time=now-self.started,
                capture_elapsed_time=None if self.capture_time is None else now-self.capture_time,
                **extra))

        if age is None or not 0 <= age <= self.config['safety']['state_timeout_s']:
            add('STATE_STALE', 'LOWSTATE_STALE', threshold_state_age_ms=self.config['safety']['state_timeout_s']*1000)
        if not available:
            if state is not None:
                add('OTHER', 'missing upper-body state', detail='missing q/dq or waist_q/waist_dq')
            return violations, q, dq, delta, age
        if not np.isfinite(q).all() or not np.isfinite(dq).all():
            raise ValueError('upper-body state contains NaN/Inf')
        if q_hold is not None:
            self.samples += 1
            self.q_min = np.minimum(self.q_min, q)
            self.q_max = np.maximum(self.q_max, q)
            self.max_delta = np.maximum(self.max_delta, np.abs(delta))
            self.max_dq = np.maximum(self.max_dq, np.abs(dq))
        for i in range(17):
            dl = waist['max_abs_delta_rad'] if i < 3 else self.config['safety']['max_joint_step_rad']
            vl = waist['max_abs_velocity_rad_s'] if i < 3 else self.config['limits']['max_velocity_rad_s']
            code = 'WAIST_SAFETY_ABORT' if i < 3 else 'ARM_DIAGNOSTIC_WARNING'
            if i < 3:
                self.waist_max_abs_delta = max(self.waist_max_abs_delta, abs(float(delta[i])))
                self.waist_max_abs_dq = max(self.waist_max_abs_dq, abs(float(dq[i])))
                if abs(delta[i]) > waist['warning_abs_delta_rad']:
                    add('WAIST_DELTA', 'WAIST_DIAGNOSTIC_WARNING', i, waist['warning_abs_delta_rad'], vl)
                if abs(delta[i]) > dl:
                    add('WAIST_DELTA', code, i, dl, vl)
            elif not (phase == 'TEACH' and i >= 3):
                if abs(delta[i]) > dl:
                    add('ARM_DELTA', code, i, dl, vl)
            if abs(dq[i]) > vl:
                add('VELOCITY', code, i, dl, vl)
        if first:
            dl = self.config['phase0_hold']['pre_acquire_motion_threshold']
            vl = self.config['control']['stable_velocity_rad_s']
            for i in range(17):
                if abs(delta[i]) >= dl:
                    add('POSITION_DELTA', 'PRE_ACQUIRE_MOTION', i, dl, vl)
                if abs(dq[i]) > vl:
                    add('VELOCITY', 'PRE_ACQUIRE_MOTION', i, dl, vl)
        return violations, q, dq, delta, age

    def warn(self, conditions):
        """Count every joint/reason sample, print and emit only the first per pair."""
        for condition in conditions:
            if condition.get('abort_code') == 'WAIST_DIAGNOSTIC_WARNING':
                self.waist_warning_count += 1
                if self.waist_warning_first is None:
                    self.waist_warning_first = dict(event='WAIST_WARNING_FIRST_TRIGGER', **condition)
            self.warning_count += 1
            key = (condition['motor_id'], condition['reason'])
            if key not in self.warnings:
                self.warnings[key] = dict(first=condition.copy(), warning_count=0,
                                         max_abs_dq=0., max_abs_delta_q=0.)
                logging.warning('ARM_DIAGNOSTIC_WARNING: %s', condition)
                self.emit(dict(event='ARM_DIAGNOSTIC_WARNING', **condition))
            record = self.warnings[key]
            record['warning_count'] += 1
            record['max_abs_dq'] = max(record['max_abs_dq'], abs(condition['dq']))
            if condition['delta_q'] is not None:
                record['max_abs_delta_q'] = max(record['max_abs_delta_q'], abs(condition['delta_q']))

    def warning_summary(self):
        records = list(self.warnings.values())
        waist = [r for r in records if r['first'].get('abort_code') == 'WAIST_DIAGNOSTIC_WARNING']
        first = records[0]['first'] if records else None
        return dict(warning_count=self.warning_count, unique_warning_count=len(records),
                    affected_motors=sorted({key[0] for key in self.warnings}),
                    max_abs_dq=max((r['max_abs_dq'] for r in records), default=0.),
                    max_abs_delta_q=max((r['max_abs_delta_q'] for r in records), default=0.),
                    first_warning_phase=None if first is None else first['phase'],
                    first_warning_weight=None if first is None else first['weight'],
                    first_warning_time=None if first is None else first['timestamp'],
                    first_warning_elapsed_time=None if first is None else first['elapsed_time'],
                    waist_warning_count=sum(r['warning_count'] for r in waist),
                    waist_warning_first=None if not waist else waist[0]['first'])

    def latch(self, violations):
        if not violations:
            return
        self.violation_samples += 1
        if self.first_trigger is None:
            self.first_trigger = dict(event='SAFETY_ABORT_FIRST_TRIGGER', latch_state='FIRST_TRIGGER',
                                      timestamp=self.clock(), violations=violations)
            logging.error('%s triggered: SAFETY_ABORT_FIRST_TRIGGER phase=%s motor=%s delta=%s',
                          violations[0]['abort_code'], violations[0].get('phase'),
                          violations[0].get('motor_id'), violations[0].get('delta_q'))
            self.emit(self.first_trigger)

    def record_other(self, exc, phase, weight):
        self.latch([dict(phase=phase, reason='OTHER', abort_code=type(exc).__name__,
                        motor_id=None, joint_name=None, q_acquire=None, q_current=None,
                        delta_q=None, dq=None, threshold_delta_q=None, threshold_dq=None,
                        state_age_ms=None, weight=weight, elapsed_time=self.clock()-self.started,
                        detail=str(exc))])

    @staticmethod
    def _frequency(writes):
        dt = np.diff([w['write_start'] for w in writes])
        return dict(write_count=len(writes), interval_count=len(dt),
                    mean_dt=float(np.mean(dt)) if len(dt) else None,
                    min_dt=float(np.min(dt)) if len(dt) else None,
                    max_dt=float(np.max(dt)) if len(dt) else None,
                    p95_dt=float(np.percentile(dt, 95)) if len(dt) else None,
                    actual_command_hz=float(1/np.mean(dt)) if len(dt) and np.mean(dt)>0 else None)

    def summary(self, q_hold):
        successful = [w for w in self.writes if w['outcome']=='SUCCESS']
        actual = [w for w in successful if w['real_dds']]
        # Simulation timing is deliberately excluded from actual DDS metrics.
        acquisition = [w for w in actual if w['phase'] in ('ACQUIRE', 'ACQUIRE_COMPLETE')]
        capture = {}
        for name, predicate in (
            ('FIRST_WRITE', lambda w: True), ('FIRST_NONZERO', lambda w: w['weight']>0),
            ('WEIGHT_0P1', lambda w: w['weight']>=.1), ('WEIGHT_0P5', lambda w: w['weight']>=.5),
            ('WEIGHT_1P0', lambda w: w['weight']>=1.)):
            hit = next((w for w in acquisition if predicate(w)), None)
            capture[f'CAPTURE_TO_{name}_MS'] = None if hit is None or self.capture_time is None else (hit['write_start']-self.capture_time)*1000
        weights = [w['weight'] for w in acquisition]
        completed = next((w for w in acquisition if w['phase']=='ACQUIRE_COMPLETE' and w['weight']==1.), None)
        elapsed = None if completed is None or self.acquire_start is None else completed['write_start']-self.acquire_start
        known = bool(acquisition)
        ramp = dict(WEIGHT_MONOTONIC=('YES' if all(a<=b for a,b in zip(weights,weights[1:])) else 'NO') if known else 'UNKNOWN',
                    WEIGHT_START_NEAR_ZERO=('YES' if weights[0]<=.001 else 'NO') if known else 'UNKNOWN',
                    WEIGHT_END_1=('YES' if completed else 'NO') if known else 'UNKNOWN',
                    ACQUIRE_DURATION_APPROX_2S=('YES' if abs(elapsed-2.)<=.1 else 'NO') if elapsed is not None else 'NOT_COMPLETED',
                    acquire_duration_s=elapsed, duration_tolerance_s=.1,
                    max_acquire_weight=max(weights) if weights else None,
                    acquire_time_weight=[dict(time=w['write_start'], weight=w['weight']) for w in acquisition])
        waist = {}
        for i in range(3):
            waist[f'motor{i+12}'] = dict(joint_name=JOINT_NAMES[i],
                acquire_q=None if q_hold is None else float(q_hold[i]),
                min_q=float(self.q_min[i]) if self.samples else None,
                max_q=float(self.q_max[i]) if self.samples else None,
                max_abs_delta_q=float(self.max_delta[i]) if self.samples else None,
                max_abs_dq=float(self.max_dq[i]) if self.samples else None)
        return dict(event='GOLDEN_DIAGNOSTICS_SUMMARY', backend='DDS' if self.real_dds else 'SIMULATION',
            capture_time=self.capture_time, capture_state_time=self.capture_state_time,
            capture_state_age_ms=None if self.capture_time is None else (self.capture_time-self.capture_state_time)*1000,
            acquire_start=self.acquire_start, release_start=self.release_start,
            first_trigger=self.first_trigger, abort_latched=self.first_trigger is not None,
            violation_samples=self.violation_samples, **capture,
            ARM_WARNING_SUMMARY=self.warning_summary(),
            WAIST_WARNING_COUNT=self.waist_warning_count,
            WAIST_WARNING_FIRST_TRIGGER=self.waist_warning_first,
            WAIST_MAX_ABS_DELTA=self.waist_max_abs_delta,
            WAIST_MAX_ABS_DQ=self.waist_max_abs_dq,
            DDS_COMMAND_TIMING=self._frequency(actual),
            DDS_COMMAND_TIMING_BY_PHASE={phase:self._frequency([w for w in actual if w['phase']==phase]) for phase in ('ACQUIRE','HOLD','RELEASE')},
            DDS_WRITE_FAILURES=sum(w['outcome']!='SUCCESS' and w['real_dds'] for w in self.writes),
            OWNERSHIP_RAMP_AUDIT=ramp, WAIST_REAL_TEST_SUMMARY=waist,
            ARM_MAX_POSITION_DELTA=float(max(self.max_delta[3:])) if self.samples else None,
            ARM_MAX_VELOCITY=float(max(self.max_dq[3:])) if self.samples else None,
            ARM_MAX_DELTA_MOTOR=int(np.argmax(self.max_delta[3:]))+15 if self.samples else None,
            ARM_MAX_DQ_MOTOR=int(np.argmax(self.max_dq[3:]))+15 if self.samples else None)
