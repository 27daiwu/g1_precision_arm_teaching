"""Golden integration for offline wire tests ONLY; no UnitreeTransport support."""
import copy
from types import SimpleNamespace
import numpy as np
from .arm_sdk_types import JointState
from .golden_controller import ArmSdkGoldenController
from ..control.body_attitude_shadow import BodyAttitudeShadow
from ..trajectory.replay_c2 import C2ReplayPlan


class FakeReplayTransport:
    def __init__(self,clock,q=None):
        self.clock=clock
        self.q=np.zeros(14) if q is None else np.asarray(q,dtype=float).copy()
        self.dq=np.zeros(14);self.waist=np.zeros(3)
        self.timestamp=None
        self.frames=[];self.fail=False
        self._crc=SimpleNamespace(Crc=lambda cmd:0)
        self.publisher=SimpleNamespace(Write=self._write)
    def _message(self):
        return SimpleNamespace(mode_pr=0,mode_machine=0,reserve=[],crc=0,motor_cmd=[SimpleNamespace(q=0.,dq=0.,kp=0.,kd=0.,tau=0.,mode=0,reserve=0) for _ in range(30)])
    def state(self):
        return JointState(self.q,self.clock() if self.timestamp is None else self.timestamp,self.dq,self.waist,np.zeros(3),imu_quaternion_wxyz=np.array([1.,0.,0.,0.]))
    def _write(self,cmd):
        if self.fail:raise RuntimeError('injected fake write failure')
        self.frames.append(copy.deepcopy(cmd))
        self.q=np.array([m.q for m in cmd.motor_cmd[15:29]])
        return True


class ReplayFakeController(ArmSdkGoldenController):
    """Runs existing CRC/wire/release pipeline; final guard on fake Write."""
    def __init__(self,transport,config,limits,gains,*,clock,sleep):
        if type(transport) is not FakeReplayTransport:
            raise ValueError('fake transport required; real DDS prohibited')
        self._initialize_replay(transport,config,limits,gains,clock=clock,sleep=sleep)

    def _initialize_replay(self,transport,config,limits,gains,*,clock,sleep,real_dds=False,emit=None):
        if not np.array_equal(gains.kd,np.full(14,1.5)):
            raise ValueError('Golden wire requires Kd1.5; independent Replay audit needed for other Kd')
        super().__init__(transport,config,clock=clock,sleep=sleep,real_dds=real_dds,emit=emit,
                         teach_waist=True,motor14_kp=60.,waist_kp_by_axis=(80.,80.,120.),waist_pitch_hold_bias=.010)
        self.limits,self.gains=limits,gains
        self.shadow=BodyAttitudeShadow()
        self.replay_active=False;self.last_reference=None;self.last_publish=None
        self.deadline=None;self.transitions=['LOAD','VALIDATE']
        self.stop_started=None;self.last_feedback=None
        # Single existing publisher's Write is decorated, no new publisher is created.
        self.original_write=transport.publisher.Write
        transport.publisher.Write=self._protected_write
    def acquire(self,plan,hold_end_s):
        if type(plan) is not C2ReplayPlan or plan.limits != self.limits:
            raise ValueError('validated C2 plan with matching limits required')
        if not np.isfinite(hold_end_s) or hold_end_s<0:raise ValueError('invalid hold duration')
        self.plan=plan;self.hold_end_s=hold_end_s
        self.transitions.append('ACQUIRE')
        try:
            self.capture_upper_body_pose()
            self.acquire_current_pose()
            self._frame(1.,'HOLD')
            self.transitions.append('START_CHECK')
            state=self._state()
            if np.max(np.abs(state.q-plan.q[0]))>self.limits.start_error or np.max(np.abs(state.dq))>self.limits.start_velocity:
                raise ValueError('start mismatch; no alignment motion')
            self.shadow.update(state)
            self.last_reference=state.q.copy()
            self.last_publish=self.clock()
            self.last_feedback=state
            self.deadline=self.clock()+plan.duration+hold_end_s+self.limits.cycle_timeout
            self.started=self.clock();self.replay_active=True
            self.transitions.append('EXECUTE')
        except BaseException:
            self.close();raise
    def _protected_write(self,cmd):
        if self.replay_active and self.phase!='RELEASE':
            state=self._state();now=self.clock()
            q=np.array([m.q for m in cmd.motor_cmd[15:29]])
            dq=np.asarray(state.dq)
            if (not np.isfinite([state.q,dq,q]).all() or not 0<=now-state.timestamp<=self.limits.state_timeout
                or now>self.deadline or not 0<=now-self.last_publish<=self.limits.cycle_timeout
                or np.any(state.q<self.limits.q_min) or np.any(state.q>self.limits.q_max)
                or np.any(q<self.limits.q_min) or np.any(q>self.limits.q_max)
                or np.max(np.abs(state.q-q))>self.limits.tracking_error
                or np.max(np.abs(dq))>self.limits.velocity
                or np.max(np.abs(q-self.last_reference))>self.limits.step):
                raise ValueError('REPLAY_FINAL_WRITE_GUARD')
            dt=state.timestamp-self.last_feedback.timestamp
            if dt<0 or (dt>0 and np.max(np.abs((dq-self.last_feedback.dq)/dt))>self.limits.acceleration):
                raise ValueError('REPLAY_FEEDBACK_ACCELERATION_OR_TIME')
        result=self.original_write(cmd)
        if self.replay_active and self.phase!='RELEASE':
            self.last_reference=q.copy();self.last_publish=now
            self.last_feedback=state
        return result
    def tick(self):
        try:
            if not self.replay_active:raise ValueError('Replay is not active')
            elapsed=self.clock()-self.started
            stop_complete=False
            if self.stop_started is not None:
                u=min(1.,max(0.,(self.clock()-self.stop_started)/self.stop_duration))
                elapsed=self.stop_elapsed+self.stop_duration*(u-2.5*u**4+3*u**5-u**6)
                stop_complete=u>=1.
            q,_,_=self.plan.sample(elapsed)
            self.teach_q=q.copy();self.teach_kp=np.asarray(self.gains.kp).copy()
            self.runtime_guard.teach_q=self.teach_q;self.runtime_guard.teach_kp=self.teach_kp
            state=self._state();sample=self.shadow.update(state)
            self.set_body_attitude_correction(sample.motor13_candidate,sample.motor14_candidate)
            self._frame(1.,'TEACH')
            if stop_complete:self.close();return
            motion_duration=getattr(self,'motion_duration',self.plan.duration)
            if elapsed>=motion_duration and self.transitions[-1]!='HOLD_END':self.transitions.append('HOLD_END')
            if elapsed>=motion_duration+self.hold_end_s:self.close()
        except BaseException:
            self.close();raise
    def request_stop(self,duration):
        if not self.replay_active or self.stop_started is not None:raise ValueError('stop not available')
        p=self.plan
        vmax=max(p.peak_velocity,getattr(getattr(self,'start_transition',None),'peak_velocity',0.))
        amax=max(p.peak_acceleration,getattr(getattr(self,'start_transition',None),'peak_acceleration',0.))
        jmax=max(p.peak_jerk,getattr(getattr(self,'start_transition',None),'peak_jerk',0.))
        if (not np.isfinite(duration) or duration<=0
            or amax+1.875*vmax/duration>self.limits.acceleration
            or jmax+5.625*amax/duration+6*vmax/duration**2>p.jerk_limit):
            raise ValueError('controlled stop bounds exceed approved constraints')
        self.stop_started=self.clock();self.stop_elapsed=self.clock()-self.started
        self.stop_duration=duration
        self.deadline=self.clock()+duration+self.limits.cycle_timeout
        self.transitions.append('CONTROLLED_STOP')
    def close(self):
        # Freeze last successfully published reference, then use existing bounded release.
        if self.last_reference is not None and self.replay_active and self.teach_kp is not None:
            self.teach_q=self.last_reference.copy();self.runtime_guard.teach_q=self.teach_q
        self.replay_active=False
        if self.transitions[-1]!='RELEASE':self.transitions.append('RELEASE')
        self.release()
