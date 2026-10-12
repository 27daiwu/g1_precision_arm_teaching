"""Replay 1.1: one Golden owner/publisher; explicit hardware permission required."""
import time
import numpy as np
from .transport import UnitreeTransport
from .replay_fake_adapter import ReplayFakeController,FakeReplayTransport
from .golden_reference import GOLDEN_RAMP_S
from ..trajectory.replay_c2 import C2ReplayPlan


class ReplayController(ReplayFakeController):
    def __init__(self,transport,config,permission,*,clock=time.monotonic,sleep=time.sleep,emit=None):
        real=type(transport) is UnitreeTransport
        if not real and type(transport) is not FakeReplayTransport:raise ValueError('unsupported transport')
        if real and not permission.allowed:raise ValueError('hardware execution permission denied')
        self.permission=permission
        self.abort_reason=None;self.fault_kind='preflight';self.release_failures=0
        self.release_attempted=False
        self.max_tracking=0.;self.publish_intervals=[];self.last_body=None
        self.last_attempted_reference=None
        self._initialize_replay(transport,config,permission.limits,permission.gains,
                               clock=clock,sleep=sleep,real_dds=real,emit=emit)
        self.period_s=permission.period

    def acquire(self,plan,hold_end_s=None):
        if type(plan) is not C2ReplayPlan or plan.limits!=self.limits:raise ValueError('validated matching C2 plan required')
        self.permission.check_plan(plan)
        self.plan=plan;self.hold_end_s=self.permission.hold
        self.start_transition=None
        self.transitions.append('ACQUIRE')
        try:
            self.capture_upper_body_pose()
            self.start_transition=self.permission.make_start_transition(self.q_hold[3:],plan)
            if self.start_transition is not None:
                times=np.linspace(0.,self.start_transition.duration,max(2,int(np.ceil(self.start_transition.duration/self.period_s))+1))
                predicted=np.asarray([self.start_transition.sample(t)[0] for t in times])
                if np.max(np.abs(np.diff(predicted,axis=0)))>self.limits.step:
                    raise ValueError('startup transition predicted Write-step budget exceeded')
            self.acquire_current_pose()
            self.phase='HOLD'
            self.transitions.append('PREFLIGHT')
            self._preflight()
            self.shadow.update(self._state())
            # Gain transition is part of preflight, at captured current command only.
            self.teach_q=self.q_hold[3:].copy()
            start=self.clock()
            while True:
                elapsed=self.clock()-start
                u=min(1.,elapsed/self.permission.gain_ramp)
                self.teach_kp=40.+(np.asarray(self.gains.kp)-40.)*(u*u*(3-2*u))
                self.runtime_guard.teach_q=self.teach_q;self.runtime_guard.teach_kp=self.teach_kp
                self._body_update();self._frame(1.,'TEACH')
                if u>=1.:break
                self._profiled_sleep(self.period_s)
            self._preflight()
            self.last_reference=np.array([m.q for m in self.cmd.motor_cmd[15:29]])
            first_ref=self.start_transition.sample(0.)[0] if self.start_transition is not None else plan.q[0]
            if np.max(np.abs(first_ref-self.last_reference))>min(self.limits.step,self.limits.velocity*self.period_s):
                raise ValueError('first target transfer exceeds step/rate budget')
            if np.max(np.abs(first_ref-self.last_reference))>1e-6:
                raise ValueError('C2 start reference must match captured command within wire tolerance; no alignment motion')
            self.last_publish=self.clock();self.last_feedback=self._state()
            self.started=self.clock();self.motion_duration=plan.duration+(self.start_transition.duration if self.start_transition else 0.)
            self.deadline=self.started+self.motion_duration+self.hold_end_s+self.limits.cycle_timeout
            self.replay_active=True;self.fault_kind='motion'
            self.transitions.append('EXECUTE')
            self._profiled_sleep(self.period_s)
        except BaseException as exc:
            self.abort_reason=str(exc);self.close();raise

    def _preflight(self):
        state=self._state();now=self.clock()
        if self.stopped:raise InterruptedError('operator stop during preflight')
        if not 0<=now-state.timestamp<=self.limits.state_timeout:raise ValueError('preflight stale feedback')
        expected=self.q_hold[3:] if self.start_transition is not None else self.plan.q[0]
        if (np.max(np.abs(state.q-expected))>self.limits.start_error or
                np.max(np.abs(state.dq))>self.limits.start_velocity):
            raise ValueError('preflight start mismatch/not stationary; no automatic alignment')

    def _sample_motion(self,elapsed):
        if self.start_transition is not None and elapsed<self.start_transition.duration:
            q,v,a,_=self.start_transition.sample(elapsed)
            return q,v,a
        offset=self.start_transition.duration if self.start_transition is not None else 0.
        return self.plan.sample(max(0.,elapsed-offset))

    def _body_update(self):
        try:
            state=self._state();sample=self.shadow.update(state)
        except Exception as exc:
            self._reject('feedback',str(exc))
        self.last_body=dict(state_timestamp=state.timestamp,torso_roll=sample.torso_roll,torso_pitch=sample.torso_pitch,
                            roll_error=sample.roll_error,pitch_error=sample.pitch_error,
                            motor13_correction=sample.motor13_candidate,motor14_correction=sample.motor14_candidate)
        self.set_body_attitude_correction(sample.motor13_candidate,sample.motor14_candidate)

    def _reject(self,kind,reason):
        self.fault_kind=kind;self.abort_reason=reason
        self.emit(dict(event='REPLAY_GUARD_REJECT',kind=kind,reason=reason,
                       timestamp=self.clock(),sequence=self.sequence,
                       q_ref=[m.q for m in self.cmd.motor_cmd[15:29]]))
        raise ValueError('REPLAY_WRITE_GUARD: '+reason)

    def _protected_write(self,cmd):
        now=self.clock();q=np.array([m.q for m in cmd.motor_cmd[15:29]])
        state=None
        if self.phase=='RELEASE':
            try:state=self._state()
            except Exception:pass
        if self.phase!='RELEASE':
            try:state=self._state()
            except Exception as exc:self._reject('feedback',str(exc))
            if state is None or state.dq is None or not np.isfinite([state.q,state.dq,q]).all():self._reject('feedback','invalid feedback/command')
            if not np.isfinite(now) or not 0<=now-state.timestamp<=self.limits.state_timeout:self._reject('feedback','stale/future feedback')
            if np.any(state.q<self.limits.q_min) or np.any(state.q>self.limits.q_max) or np.any(q<self.limits.q_min) or np.any(q>self.limits.q_max):self._reject('motion','joint limit')
            if np.max(np.abs(state.dq))>self.limits.velocity:self._reject('motion','measured velocity')
            error=float(np.max(np.abs(state.q-q)))
            self.max_tracking=max(self.max_tracking,error)
            if error>self.limits.tracking_error:self._reject('tracking','tracking error')
            if self.replay_active:
                dt=now-self.last_publish;delta=float(np.max(np.abs(q-self.last_reference)))
                if not 0<dt<=self.limits.cycle_timeout or now>self.deadline:self._reject('timing','publish interval/deadline')
                if delta>self.limits.step or delta>self.limits.velocity*dt+1e-9:self._reject('motion','actual reference step/rate')
                feedback_dt=state.timestamp-self.last_feedback.timestamp
                if feedback_dt<0:self._reject('feedback','feedback clock backwards')
                if feedback_dt>0 and np.max(np.abs((state.dq-self.last_feedback.dq)/feedback_dt))>self.limits.acceleration:self._reject('motion','measured acceleration')
        self.last_attempted_reference=q.copy()
        try:result=self.original_write(cmd)
        except BaseException as exc:
            self.fault_kind='write';self.abort_reason=str(exc)
            self.emit(dict(event='REPLAY_WRITE_FAILED',timestamp=now,sequence=self.sequence,
                           q_ref=q.tolist(),q_measured=None if state is None else state.q.tolist(),
                           dq_measured=None if state is None else state.dq.tolist(),error=str(exc)))
            raise
        if result is False:self._reject('write','publisher returned False')
        if self.replay_active and self.phase!='RELEASE':
            self.publish_intervals.append(now-self.last_publish)
            self.last_reference=q.copy();self.last_publish=now;self.last_feedback=state
        self.emit(dict(event='REPLAY_WIRE_STATE',sequence=self.sequence,phase=self.phase,timestamp=now,
                       q_ref=q.tolist(),q_measured=None if state is None else state.q.tolist(),
                       dq_measured=None if state is None else state.dq.tolist(),
                       state_timestamp=None if state is None else state.timestamp,
                       body=self.last_body,weight=cmd.motor_cmd[29].q,outcome='SUCCESS'))
        return result

    def tick(self):
        try:
            if not self.replay_active:raise ValueError('Replay not active')
            if self.stopped:self.fault_kind='operator';raise InterruptedError('operator hard abort')
            elapsed=self.clock()-self.started;complete=False
            if self.stop_started is not None:
                u=min(1.,max(0.,(self.clock()-self.stop_started)/self.stop_duration))
                elapsed=self.stop_elapsed+self.stop_duration*(u-2.5*u**4+3*u**5-u**6);complete=u>=1.
            self.teach_q=self._sample_motion(elapsed)[0];self.runtime_guard.teach_q=self.teach_q
            self._body_update();self._frame(1.,'TEACH')
            if complete:self.close();return
            if elapsed>=self.motion_duration and self.transitions[-1]!='HOLD_END':self.transitions.append('HOLD_END')
            if elapsed>=self.motion_duration+self.hold_end_s:self.close()
        except BaseException as exc:
            self.abort_reason=self.abort_reason or str(exc);self.close();raise

    def close(self):
        if self.abort_reason:
            action=self.permission.fault_actions[self.fault_kind]
            self.emit(dict(event='REPLAY_ABORT',reason=self.abort_reason,kind=self.fault_kind,action=action))
            if action.startswith('external_stop_required'):
                print('REPLAY: EXTERNAL STOP REQUIRED; attempting approved best-effort release',flush=True)
        if self.last_reference is not None and self.teach_kp is not None:
            reference=(self.last_attempted_reference if self.fault_kind=='write' and self.last_attempted_reference is not None else self.last_reference)
            self.teach_q=reference.copy();self.runtime_guard.teach_q=self.teach_q
        self.replay_active=False
        if self.transitions[-1]!='RELEASE':self.transitions.append('RELEASE')
        self.release()

    def release(self):
        if not self.attempted_write or self.release_attempted:return
        self.release_attempted=True
        start=self.clock();weight=self.weight;self.diagnostics.release_start=start
        try:
            while self.clock()-start<GOLDEN_RAMP_S:
                w=weight*(1.-self.smoothstep((self.clock()-start)/GOLDEN_RAMP_S))
                try:self._frame(w,'RELEASE')
                except Exception as exc:
                    self.release_failures+=1
                    self.emit(dict(event='REPLAY_RELEASE_WRITE_FAILED',error=str(exc)))
                self._profiled_sleep(self.period_s)
        finally:
            try:
                self._frame(0.,'RELEASE',final=True)
                self.released=True
            except Exception as exc:
                self.release_failures+=1
                self.emit(dict(event='REPLAY_RELEASE_UNCONFIRMED',error=str(exc)))
            self.emit(dict(event='REPLAY_EXIT',released=self.released,release_failures=self.release_failures,
                           abort_reason=self.abort_reason,transitions=self.transitions,
                           max_tracking_error_rad=self.max_tracking,
                           frequency=dict(interval_count=len(self.publish_intervals),mean_dt_s=float(np.mean(self.publish_intervals)) if self.publish_intervals else None,min_dt_s=min(self.publish_intervals,default=None),max_dt_s=max(self.publish_intervals,default=None),p95_dt_s=float(np.percentile(self.publish_intervals,95)) if self.publish_intervals else None,actual_hz=1./float(np.mean(self.publish_intervals)) if self.publish_intervals else None)))
