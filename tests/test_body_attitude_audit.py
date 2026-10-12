import importlib.util
import json
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('body_audit',
    Path(__file__).resolve().parents[1] / 'scripts/analyze_body_attitude_closed_loop.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_legacy_alignment_uses_pre_write_state_not_array_offset(tmp_path):
    rows = [dict(event='STATE', phase='TEACH', timestamp=t, motor14_command_ref=q)
            for t, q in ((.1, .5), (1.001, .01), (1.021, .0079))]
    rows += [dict(event='DDS_WRITE_RESULT', phase='TEACH', sequence=n,
                  write_start=t, outcome='SUCCESS')
             for n, t in ((3616, 1.002), (3617, 1.022))]
    path = tmp_path / 'legacy.jsonl'
    path.write_text('\n'.join(json.dumps(r) for r in rows))
    result = audit.analyze(path)
    assert result['limiter_audit'] == 'UNAVAILABLE_IN_LEGACY_LOG'
    pair = result['sequence_3616_3617'][0]
    assert pair['delta_q_rad'] == pytest.approx(-.0021)
    assert pair['abs_write_rate_rad_s'] == pytest.approx(.105)


@pytest.mark.parametrize('bad_dt', [False, True])
def test_new_evidence_distinguishes_source_dt_from_write_dt(tmp_path, bad_dt):
    rows = []
    for n, (source_time, write_time, candidate) in enumerate(((1., 2., 0.), (1.022, 2.02, .0022))):
        sample = dict(state_timestamp=source_time, limiter_dt_s=.02 if bad_dt else .022,
                      kp_roll=.5, kp_pitch=.5, roll_error=.1, pitch_error=.1,
                      clamp_rad=.05, rate_limit_rad_s=.1, closed_loop=True,
                      motor13_candidate=candidate, motor14_candidate=candidate)
        rows.append(dict(event='DDS_WRITE_RESULT', phase='TEACH', sequence=n,
                         write_start=write_time, outcome='SUCCESS', body_attitude=sample,
                         motor13_command_ref=-.12+candidate, motor14_command_ref=.14+candidate,
                         motor13_correction=candidate, motor14_correction=candidate))
    path = tmp_path / 'new.jsonl'
    path.write_text('\n'.join(json.dumps(r) for r in rows))
    result = audit.analyze(path)
    assert result['max_write_rate']['14']['abs_write_rate_rad_s'] == pytest.approx(.11)
    assert bool(result['limiter_violations']) == bad_dt
