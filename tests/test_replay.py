import json
from types import SimpleNamespace
import numpy as np
import pytest
from g1_dual_arm_teaching.trajectory.replay import ReplayLimits, ReplayPlan, ReplayGains, MockReplayExecutor
from g1_dual_arm_teaching.trajectory.replay_c2 import C2ReplayPlan, RelativeC2ReplayPlan, RelativeReplaySpec
from g1_dual_arm_teaching.trajectory.replay_permission import ReplayPermission


def limits():
    return ReplayLimits((-2.,)*14,(2.,)*14,1.,10.,.05,.1,.01,.1,.1,.1,10.,.02)


def fixture(tmp_path):
    t=np.array([10.,10.04,10.09])
    q=np.array([np.zeros(14),np.full(14,.001),np.full(14,.002)])
    arrays=dict(timestamp=t,cycle_timestamp=t+.001,q_measured=q,dq_measured=np.zeros((3,14)),q_ref=np.ones((3,14)),waist_q=np.ones((3,3)),torso_roll_pitch=np.zeros((3,2)),arm_motor_ids=np.arange(15,29),waist_motor_ids=np.arange(12,15))
    meta=dict(schema_version=2,sample_count=3,duration_s=.09,data_integrity='PASS',start_q=q[0].tolist(),end_q=q[-1].tolist())
    path=tmp_path/'trajectory.npz'
    np.savez(path,**arrays)
    path.with_suffix('.json').write_text(json.dumps(meta))
    return path,meta,arrays


def test_actual_times_and_analytic_limits(tmp_path):
    path,meta,a=fixture(tmp_path)
    plan=ReplayPlan.load(path,limits(),.5)
    assert plan.duration==pytest.approx(.18)
    q,v,acc=plan.sample(.04)
    np.testing.assert_allclose(q,.0005)
    assert plan.peak_velocity==pytest.approx(.01875)
    assert plan.peak_acceleration==pytest.approx(.9375)
    np.testing.assert_array_equal(plan.sample(9)[0],a['q_measured'][-1])
    for t in np.linspace(0,plan.duration,1000):
        q,v,acc=plan.sample(t)
        assert np.max(np.abs(v))<=plan.peak_velocity+1e-12
        assert np.max(np.abs(acc))<=plan.peak_acceleration+1e-12


@pytest.mark.parametrize('fault',['duplicate','reverse','gap','nan','shape','mapping','count','integrity','position','velocity','acceleration','step','endpoint','cycle','duration','missing'])
def test_reject_invalid_data(tmp_path,fault):
    _,m,a=fixture(tmp_path)
    if fault=='duplicate':a['timestamp'][1]=a['timestamp'][0]
    if fault=='reverse':a['timestamp'][1]=9
    if fault=='gap':a['timestamp'][2]=11
    if fault=='nan':a['q_measured'][1,0]=np.nan
    if fault=='shape':a['dq_measured']=np.zeros((3,13))
    if fault=='mapping':a['arm_motor_ids'][0]=14
    if fault=='count':m['sample_count']=4
    if fault=='integrity':m['data_integrity']='WARN'
    if fault=='position':a['q_measured'][1,0]=3
    if fault=='velocity':a['dq_measured'][1,0]=2
    if fault=='acceleration':a['dq_measured'][1,0]=.8
    if fault=='step':a['q_measured'][1,0]=.06
    if fault=='endpoint':m['end_q']=[1.]*14
    if fault=='cycle':a['cycle_timestamp'][1]=9
    if fault=='duration':m['duration_s']=5
    if fault=='missing':del a['waist_q']
    with pytest.raises(ValueError):ReplayPlan(m,a,limits())


def test_speed_scale_cannot_bypass_planned_constraints(tmp_path):
    _,m,a=fixture(tmp_path)
    with pytest.raises(ValueError):ReplayPlan(m,a,limits(),5)
    for s in [0,-1,np.nan]:
        with pytest.raises(ValueError):ReplayPlan(m,a,limits(),s)


def executor(path):
    e=MockReplayExecutor(limits(),ReplayGains((12.,)*14,(1.5,)*14,(5.,)*14,(20.,)*14,(.5,)*14,(3.,)*14))
    e.load(path)
    return e


def state(t,q=None,dq=None):
    return SimpleNamespace(timestamp=t,q=np.zeros(14) if q is None else q,dq=np.zeros(14) if dq is None else dq)


def test_mock_state_machine_end_hold_release(tmp_path):
    p,_,_=fixture(tmp_path);e=executor(p)
    for t in [0.,.02,.04,.06,.08,.10,.12,.14]:
        q=e.plan.sample(t)[0]
        e.tick(t,state(t,q),hold_duration=.04)
    assert e.transitions==['LOAD','VALIDATE','ACQUIRE','START_CHECK','EXECUTE','HOLD_END','RELEASE']
    assert e.owner.released
    np.testing.assert_array_equal(e.owner.references[-1],e.plan.q[-1])


@pytest.mark.parametrize('fault',['start','stale','tracking','velocity','clock','gap','missing'])
def test_mock_faults_release_and_do_not_command_on_failure(tmp_path,fault):
    p,_,_=fixture(tmp_path);e=executor(p)
    if fault=='start':
        s=state(0,np.ones(14));now=0
    else:
        e.tick(0,state(0));now=.02;s=state(now)
        if fault=='stale':s.timestamp=-1
        if fault=='tracking':s.q[:]=.2
        if fault=='velocity':s.dq[:]=2
        if fault=='clock':now=0
        if fault=='gap':now=.3;s.timestamp=now
        if fault=='missing':s=None
    n=len(e.owner.references)
    with pytest.raises((ValueError,AttributeError)):e.tick(now,s)
    assert e.owner.released and len(e.owner.references)==n


def test_replay_gains_require_independent_ranges():
    with pytest.raises(ValueError):
        ReplayGains((40.,)*14,(1.5,)*14,(5.,)*14,(20.,)*14,(.5,)*14,(3.,)*14)


def test_loader_is_bounded_and_rejects_incomplete_pairs(tmp_path):
    p,m,a=fixture(tmp_path)
    m['sample_count']=30001
    p.with_suffix('.json').write_text(json.dumps(m))
    with pytest.raises(ValueError):ReplayPlan.load(p,limits())
    m['sample_count']=3
    p.with_suffix('.json').write_text(json.dumps(m))
    del a['q_measured']
    np.savez(p,**a)
    with pytest.raises(ValueError):ReplayPlan.load(p,limits())


def test_legacy_schema_supported_and_q_ref_is_not_replayed(tmp_path):
    p,m,a=fixture(tmp_path)
    m['schema_version']=1
    m.pop('start_q');m.pop('end_q')
    p.with_suffix('.json').write_text(json.dumps(m))
    plan=ReplayPlan.load(p,limits())
    np.testing.assert_array_equal(plan.q,a['q_measured'])
    assert not np.array_equal(plan.q,a['q_ref'])


def test_invalid_plan_releases_without_acquiring_or_commanding(tmp_path):
    p,m,a=fixture(tmp_path);m['data_integrity']='WARN'
    p.with_suffix('.json').write_text(json.dumps(m))
    e=MockReplayExecutor(limits(),ReplayGains((12.,)*14,(1.5,)*14,(5.,)*14,(20.,)*14,(.5,)*14,(3.,)*14))
    with pytest.raises(ValueError):e.load(p)
    assert not e.owner.acquired and not e.owner.references and e.owner.released


def test_first_target_step_is_checked_without_start_alignment(tmp_path):
    p,_,_=fixture(tmp_path);e=executor(p)
    e.tick(0,state(0,np.full(14,.009)))
    np.testing.assert_array_equal(e.owner.references[0],np.zeros(14))
    e.close()
    assert e.owner.released


def test_relative_plan_rebases_exactly_scales_motion_and_preserves_absolute_plan(tmp_path):
    p,meta,a=fixture(tmp_path)
    before=np.load(p,allow_pickle=False)['q_measured'].copy()
    absolute=C2ReplayPlan.load(p,limits(),.5,jerk_limit=1e6,jerk_jump_limit=1e6)
    acquire=np.linspace(-.2,.2,14)
    relative=RelativeC2ReplayPlan.load(p,limits(),.5,jerk_limit=1e6,
        jerk_jump_limit=1e6,acquire_q=acquire,motion_scale=.25)
    np.testing.assert_array_equal(relative.sample(0.)[0],acquire)
    np.testing.assert_allclose(relative.q[-1]-acquire,.0005,atol=1e-12)
    np.testing.assert_array_equal(absolute.q,a['q_measured'])
    np.testing.assert_array_equal(np.load(p,allow_pickle=False)['q_measured'],before)
    assert relative.duration==pytest.approx(absolute.duration)
    assert relative.peak_velocity == pytest.approx(absolute.peak_velocity*.25,rel=1e-6)
    assert relative.peak_acceleration == pytest.approx(absolute.peak_acceleration*.25,rel=1e-6)


def test_relative_spec_binds_only_to_acquire_snapshot_and_selects_segment(tmp_path):
    p,_,_=fixture(tmp_path)
    spec=RelativeReplaySpec.load(p,limits(),1.,jerk_limit=1e6,jerk_jump_limit=1e6,
        motion_scale=.5,start_index=0,end_index=3)
    acquire=np.full(14,.25)
    plan=spec.bind(acquire)
    np.testing.assert_array_equal(plan.q[0],acquire)
    assert plan.start_index==0 and plan.end_index==3
    np.testing.assert_allclose(plan.q[-1]-acquire,.001,atol=1e-12)


def test_relative_plan_rejects_out_of_range_rebased_target(tmp_path):
    p,_,_=fixture(tmp_path)
    with pytest.raises(ValueError):
        RelativeC2ReplayPlan.load(p,limits(),1.,jerk_limit=1e6,
            jerk_jump_limit=1e6,acquire_q=np.full(14,1.999),motion_scale=2.)


def test_relative_plan_requires_permission_bound_to_mode_scale_and_segment(tmp_path):
    p,_,_=fixture(tmp_path)
    plan=RelativeC2ReplayPlan.load(p,limits(),1.,jerk_limit=1e6,
        jerk_jump_limit=1e6,acquire_q=np.zeros(14),motion_scale=.5,
        start_index=0,end_index=3)
    permission=object.__new__(ReplayPermission)
    permission.replay_mode='absolute'
    permission.manifest={'relative_motion_scale':.5,'relative_start_index':0,
        'relative_end_index':3,'max_first_run_duration_s':10.,
        'max_first_run_span_rad':1.}
    permission.stop=1000.
    permission.limits=ReplayLimits((-2.,)*14,(2.,)*14,1e9,1e9,1.,.1,1.,1.,1.,1.,10.,1.)
    permission.jerk=1e9
    with pytest.raises(ValueError,match='replay_mode'):
        permission.check_plan(plan)
    permission.replay_mode='relative'
    permission.manifest['relative_motion_scale']=.25
    with pytest.raises(ValueError,match='not bound'):
        permission.check_plan(plan)
    permission.manifest['relative_motion_scale']=.5
    permission.check_plan(plan)
