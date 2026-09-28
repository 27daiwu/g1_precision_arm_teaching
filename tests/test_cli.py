from pathlib import Path
import subprocess
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('script', ['phase0_arm_sdk_probe', 'phase0_acquire_hold_test', 'phase1_joint_hold', 'phase1_waypoint_demo'])
def test_offline_scripts(script, tmp_path):
    args = [sys.executable, str(ROOT / 'scripts' / f'{script}.py'), '--duration', '0', '--log', str(tmp_path / 'response.jsonl')]
    if script == 'phase1_waypoint_demo':
        from g1_dual_arm_teaching.waypoint import JointWaypoint, WaypointStore
        import numpy as np
        for point in ('a', 'b'):
            WaypointStore.save(tmp_path / f'{point}.yaml', JointWaypoint(point, np.full(14, .001), .04))
            args += [f'--waypoint-{point}', str(tmp_path / f'{point}.yaml')]
    result = subprocess.run(args, cwd=ROOT, text=True, capture_output=True, timeout=15)
    assert result.returncode == 0, result.stderr
    if script == 'phase0_arm_sdk_probe':
        assert 'UNKNOWN' in result.stdout
    else:
        assert (tmp_path / 'response.jsonl').stat().st_size > 0
