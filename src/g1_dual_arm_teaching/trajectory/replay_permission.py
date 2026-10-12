"""Fail-closed Replay hardware permission manifest. No production defaults."""
import hashlib
from datetime import datetime
import json
from pathlib import Path
import numpy as np
from .replay import ReplayLimits,ReplayGains


def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(65536),b''):h.update(chunk)
    return h.hexdigest()


class ReplayPermission:
    def __init__(self,path,trajectory,*,execute=False):
        self.manifest=json.loads(Path(path).read_text(encoding='utf-8'))
        m=self.manifest
        if not isinstance(m,dict):raise ValueError('permission must be JSON object')
        self.start_transition=m.get('start_transition',{'enabled':False})
        if not isinstance(self.start_transition,dict):raise ValueError('invalid start transition permission')
        self.start_transition_enabled=self.start_transition.get('enabled') is True
        if self.start_transition_enabled:
            transition_approval=m.get('approvals',{}).get('start_transition',{})
            space_approval=m.get('approvals',{}).get('mechanical_space',{})
            for approval in (transition_approval,space_approval):
                if (approval.get('verified') is not True or not all(isinstance(approval.get(k),str) and approval[k].strip()
                    for k in ('reviewer','verified_at','evidence'))):
                    raise ValueError('start transition requires separate mechanical-space approval')
                verified_at=datetime.fromisoformat(approval['verified_at'])
                if verified_at.utcoffset() is None:raise ValueError('approval date must include timezone')
            for key in ('duration_s','max_abs_start_delta_rad'):
                value=self.start_transition.get(key)
                if not isinstance(value,(int,float)) or not np.isfinite(value) or value<=0:
                    raise ValueError(f'explicit start transition {key} required')
        self.limits=ReplayLimits(**m['limits']);self.gains=ReplayGains(**m['gains'])
        self.speed=float(m['speed_scale']);self.jerk=float(m['jerk_limit']);self.jerk_jump=float(m['jerk_jump_limit'])
        self.hold=float(m['hold_end_s']);self.stop=float(m['stop_duration_s'])
        self.period=float(m['period_s']);self.gain_ramp=float(m['gain_ramp_s'])
        for x in (self.speed,self.jerk,self.jerk_jump,self.stop,self.period,self.gain_ramp):
            if not np.isfinite(x) or x<=0:raise ValueError('invalid timing/constraint')
        if not np.isfinite(self.hold) or self.hold<0 or self.period>self.limits.cycle_timeout:
            raise ValueError('invalid hold/period')
        for key in ('max_first_run_duration_s','max_first_run_span_rad'):
            if not np.isfinite(m[key]) or m[key]<=0:raise ValueError('invalid first-run envelope')
        if m.get('trajectory_sha256')!=sha256(trajectory) or m.get('metadata_sha256')!=sha256(Path(trajectory).with_suffix('.json')):
            raise ValueError('permission not bound to exact trajectory/metadata')
        self.allowed=False
        if execute:
            if m.get('hardware_execute_enabled') is not True:raise ValueError('hardware execute disabled')
            required_approvals=['joint_limits','replay_gains','gain_transition','mechanical_space','speed_jerk',
                        'fault_release_policy','transport_write_bound','body_closed_loop']
            if self.start_transition_enabled:required_approvals.append('start_transition')
            for key in required_approvals:
                approval=m.get('approvals',{}).get(key,{})
                if (approval.get('verified') is not True or not all(isinstance(approval.get(f),str) and approval[f].strip()
                    for f in ('reviewer','verified_at','evidence'))):
                    raise ValueError(f'unverified hardware prerequisite: {key}')
                verified_at=datetime.fromisoformat(approval['verified_at'])
                if verified_at.utcoffset() is None:raise ValueError('approval date must include timezone')
            if m.get('arm_motor_ids')!=list(range(15,29)):
                raise ValueError('hardware motor mapping not approved')
            if len(m.get('joint_limit_sources',[]))!=14 or not all(isinstance(s,str) and s.strip() for s in m['joint_limit_sources']):
                raise ValueError('14 actual joint-limit evidence sources required')
            self.allowed=True
        policies=m.get('fault_actions',{})
        required=('preflight','feedback','tracking','motion','timing','write','operator')
        if any(policies.get(k) not in ('freeze_then_bounded_release','external_stop_required_then_best_effort_release') for k in required):
            raise ValueError('explicit per-fault release decisions required')
        self.fault_actions=policies
        if not np.array_equal(self.gains.kd,np.full(14,1.5)):
            raise ValueError('Kd must match audited Golden Kd1.5')
        if any(lo>40 or hi<40 for lo,hi in zip(self.gains.kp_min,self.gains.kp_max)):
            raise ValueError('gain-transition ranges must include acquisition Kp40')

    def check_plan(self,plan):
        if (plan.duration>self.manifest['max_first_run_duration_s'] or
                np.max(np.ptp(plan.q,axis=0))>self.manifest['max_first_run_span_rad']):
            raise ValueError('first-run short/small mechanical envelope exceeded')
        if (plan.peak_acceleration+1.875*plan.peak_velocity/self.stop>self.limits.acceleration or
            plan.peak_jerk+5.625*plan.peak_acceleration/self.stop+6*plan.peak_velocity/self.stop**2>self.jerk):
            raise ValueError('approved controlled-stop constraints cannot be met')

    def make_start_transition(self,start_q,plan):
        if not self.start_transition_enabled:
            return None
        from .start_transition import SepticStartTransition
        start=np.asarray(start_q,dtype=float)
        if start.shape!=(14,) or not np.isfinite(start).all():raise ValueError('invalid captured transition start')
        if np.max(np.abs(start-plan.q[0]))>self.start_transition['max_abs_start_delta_rad']:
            raise ValueError('captured pose exceeds separately approved transition envelope')
        transition=SepticStartTransition(start,plan.q[0],self.start_transition['duration_s'],self.limits,self.jerk,
            end_velocity=np.zeros(14),end_acceleration=np.zeros(14),end_jerk=plan.start_jerk,
            separately_approved=True,space_checked=True)
        vmax=max(plan.peak_velocity,transition.peak_velocity)
        amax=max(plan.peak_acceleration,transition.peak_acceleration)
        jmax=max(plan.peak_jerk,transition.peak_jerk)
        if (amax+1.875*vmax/self.stop>self.limits.acceleration or
                jmax+5.625*amax/self.stop+6*vmax/self.stop**2>self.jerk):
            raise ValueError('connector plus C2 path cannot meet controlled-stop envelope')
        return transition
