#!/usr/bin/env python3
"""Offline BODY attitude / DDS command audit; never imports the robot SDK."""
import argparse
from bisect import bisect_right
import json
from pathlib import Path


def analyze(path):
    with Path(path).open() as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    states = [r for r in rows if r.get('event') == 'STATE' and r['phase'] == 'TEACH']
    times = [r['timestamp'] for r in states]
    writes = [r.copy() for r in rows if r.get('event') == 'DDS_WRITE_RESULT' and r['phase'] == 'TEACH']
    legacy_matches = 0
    for write in writes:
        if 'motor14_command_ref' not in write:
            # Old STATE logs are emitted after command preparation, before Write.
            index = bisect_right(times, write['write_start']) - 1
            if index >= 0 and 0 <= write['write_start'] - times[index] < .01:
                write['motor14_command_ref'] = states[index]['motor14_command_ref']
                legacy_matches += 1
    rates = []
    for a, b in zip(writes, writes[1:]):
        dt = b['write_start'] - a['write_start']
        if dt <= 0 or b['sequence'] != a['sequence'] + 1:
            continue
        for motor in (13, 14):
            key = f'motor{motor}_command_ref'
            if key in a and key in b:
                delta = b[key] - a[key]
                rates.append(dict(motor_id=motor, sequence_from=a['sequence'],
                                  sequence_to=b['sequence'], delta_q_rad=delta,
                                  write_dt_s=dt, abs_write_rate_rad_s=abs(delta/dt),
                                  minimum_limiter_dt_s=abs(delta)/.1))
    evidence = [w for w in writes if 'body_attitude' in w]
    violations = []
    for previous, current in zip(evidence, evidence[1:]):
        a, b = previous['body_attitude'], current['body_attitude']
        if current['sequence'] != previous['sequence'] + 1:
            continue
        dt = b['state_timestamp'] - a['state_timestamp']
        if abs(dt-b['limiter_dt_s']) > 1e-9:
            violations.append(dict(sequence=current['sequence'], reason='limiter_dt_mismatch'))
        for motor, axis in ((13, 'roll'), (14, 'pitch')):
            candidate = b[f'motor{motor}_candidate']
            target = max(-b['clamp_rad'], min(b['clamp_rad'], b[f'kp_{axis}']*b[f'{axis}_error']))
            step = b['rate_limit_rad_s'] * b['limiter_dt_s']
            expected = a[f'motor{motor}_candidate'] + max(-step, min(step, target-a[f'motor{motor}_candidate']))
            if abs(candidate-expected) > 1e-12 or abs(candidate) > b['clamp_rad']+1e-12:
                violations.append(dict(sequence=current['sequence'], motor_id=motor, reason='candidate_mismatch'))
            applied = candidate if b['closed_loop'] else 0.
            if abs(current[f'motor{motor}_correction']-applied) > 1e-12:
                violations.append(dict(sequence=current['sequence'], motor_id=motor, reason='applied_correction_mismatch'))
    shadow = [r for r in rows if r.get('event') == 'BODY_ATTITUDE_SHADOW_V1']
    return dict(file=str(path), teach_write_count=len(writes),
                shadow_sample_count=len(shadow), legacy_motor14_state_matches=legacy_matches,
                per_write_attitude_evidence_count=len(evidence),
                limiter_audit='AVAILABLE' if len(evidence) > 1 else 'UNAVAILABLE_IN_LEGACY_LOG',
                limiter_violations=violations,
                max_abs_candidate_rad={str(m): max((abs(r[f'motor{m}_candidate']) for r in shadow), default=None) for m in (13, 14)},
                max_write_rate={str(m): max((r for r in rates if r['motor_id'] == m), key=lambda r:r['abs_write_rate_rad_s'], default=None) for m in (13, 14)},
                sequence_3616_3617=[r for r in rates if r['sequence_from'] == 3616],
                teach_write_failures=sum(w['outcome'] != 'SUCCESS' for w in writes),
                waypoint_count=sum(r.get('event') == 'JOINT_WAYPOINT_MARK' for r in rows))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    for path in parser.parse_args().logs:
        print(json.dumps(analyze(path), ensure_ascii=False, allow_nan=False, indent=2))
