"""Buffered monotonic profiling; never sleeps or changes command scheduling."""
from contextlib import contextmanager
from functools import wraps
import time
import numpy as np

GROUPS = dict(prepare='prepare_us', wire_guard='prepare_us', cdr='prepare_us', serialization='prepare_us', deserialization='prepare_us', allocation='prepare_us', state_check='state_check_us', state_lock='state_check_us',
              diagnostics='diagnostics_us', crc='crc_us', dds_write='dds_write_us',
              logging='logging_us', json='logging_us', file_write='logging_us',
              file_flush='logging_us', console='logging_us', sleep='sleep_us')


def profiled(stage):
    def decorate(method):
        @wraps(method)
        def call(self, *args, **kwargs):
            with self.profiler.measure(stage):
                return method(self, *args, **kwargs)
        return call
    return decorate


class CycleProfiler:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.segments = []
        self.stack = []
        self.cursor = None
        self.enabled = True

    def _close_leaf(self, now):
        if self.stack and self.cursor is not None:
            self.segments.append((self.cursor, now, self.stack[-1]))
        self.cursor = now

    @contextmanager
    def measure(self, stage):
        if not self.enabled:
            yield
            return
        self._close_leaf(self.clock())
        self.stack.append(stage)
        try:
            yield
        finally:
            self._close_leaf(self.clock())
            self.stack.pop()

    def reports(self, writes):
        """Exclusive leaf attribution, clipped to consecutive successful Write starts."""
        self.enabled = False  # Final report serialization is outside the command loop.
        writes = [w for w in writes if w['outcome'] == 'SUCCESS']
        records = []
        for left, right in zip(writes, writes[1:]):
            start, end = left['write_start'], right['write_start']
            detail = {}
            for a, b, stage in self.segments:
                overlap = max(0., min(b, end)-max(a, start))
                if overlap:
                    detail[stage] = detail.get(stage, 0.)+overlap*1e6
            fields = {group:0. for group in GROUPS.values()}
            for stage, value in detail.items():
                fields[GROUPS[stage]] += value
            total = (end-start)*1e6
            residual = max(0., total-sum(fields.values()))
            candidates = {k:v for k,v in detail.items() if k!='sleep'}
            candidates['unattributed'] = residual
            record = dict(event='PER_CYCLE_PROFILE', from_sequence=left['sequence'],
                          to_sequence=right['sequence'], from_phase=left['phase'], to_phase=right['phase'],
                          start=start, end=end, real_dds=left['real_dds'] and right['real_dds'],
                          **fields, total_us=total, unattributed_us=residual, detail_us=detail,
                          dt_gt_30ms=total>30000., dt_gt_50ms=total>50000., dt_gt_100ms=total>100000.,
                          largest_non_sleep_stage=max(candidates, key=candidates.get),
                          largest_stage=max({**detail,'unattributed':residual}, key={**detail,'unattributed':residual}.get))
            records.append(record)
        return records


    @staticmethod
    def summary(records):
        fields = ('prepare_us', 'state_check_us', 'diagnostics_us', 'crc_us',
                  'dds_write_us', 'logging_us', 'sleep_us', 'total_us')
        means = {key:float(np.mean([r[key] for r in records])) if records else None for key in fields}
        means['guard_us'] = float(np.mean([r['detail_us'].get('wire_guard', 0.) for r in records])) if records else None
        return dict(event='CONTROL_LOOP_PROFILE_SUMMARY', interval_count=len(records),
                    mean_us=means, non_sleep_mean_us=float(np.mean([r['total_us']-r['sleep_us'] for r in records])) if records else None,
                    long_interval_counts={str(n):sum(r[f'dt_gt_{n}ms'] for r in records) for n in (30,50,100)})


class Motor14Diagnostics:
    def __init__(self):
        self.capture_q = None
        self.samples = []

    def capture(self, q):
        self.capture_q = float(q)

    def observe(self, phase, q, dq, timestamp):
        if self.capture_q is not None:
            self.samples.append(dict(phase=phase, q=float(q), dq=float(dq),
                                     delta=float(q)-self.capture_q, timestamp=timestamp))

    def summary(self):
        phases = {}
        for phase in ('ACQUIRE', 'HOLD', 'RELEASE'):
            rows = [s for s in self.samples if s['phase']==phase]
            delta = [s['delta'] for s in rows]
            phases[phase] = dict(sample_count=len(rows),
                mean_delta=float(np.mean(delta)) if rows else None,
                min_delta=min(delta) if rows else None, max_delta=max(delta) if rows else None,
                max_abs_delta=max(map(abs,delta)) if rows else None,
                mean_dq=float(np.mean([s['dq'] for s in rows])) if rows else None)
        return dict(event='MOTOR14_HOLD_COMPARISON', capture_q=self.capture_q,
                    units='rad, rad/s; sample-weighted means', phases=phases)
