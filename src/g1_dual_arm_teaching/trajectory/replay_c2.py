"""Auditable C2 quintic Hermite interpolation on unmodified recorded times."""
import copy
import numpy as np
from .replay import ReplayPlan, load_replay_trajectory


def extrema(coeff, order=0):
    """Exact polynomial extrema candidates on normalized interval [0,1]."""
    p=np.polynomial.polynomial.polyder(coeff, order)
    roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(p))
    u=[0.,1.]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<1]
    values=np.polynomial.polynomial.polyval(u,p)
    return float(np.min(values)),float(np.max(values))


class C2Curve:
    """Unvalidated geometry for diagnostics; never accepted by the wire adapter."""
    def __init__(self,time,q):
        self.time=np.asarray(time,dtype=float).copy()
        self.q=np.asarray(q,dtype=float).copy()
        if (self.time.ndim!=1 or len(self.time)<2 or self.q.shape!=(len(self.time),14)
                or not np.isfinite(self.time).all() or not np.isfinite(self.q).all()
                or np.any(np.diff(self.time)<=0)):
            raise ValueError('invalid curve input')
        self.duration=float(self.time[-1])
        t=self.time
        edge=2 if len(t)>2 else 1
        v=np.gradient(self.q,t,axis=0,edge_order=edge)
        a=np.gradient(v,t,axis=0,edge_order=edge)
        v[[0,-1]]=0.;a[[0,-1]]=0.
        self.coefficients=np.zeros((len(t)-1,14,8))
        matrix=np.array([[1,1,1],[3,4,5],[6,12,20]],dtype=float)
        self.peak_velocity=self.peak_acceleration=self.peak_jerk=0.
        self.max_linear_deviation=0.
        self.position_min=np.full(14,np.inf);self.position_max=np.full(14,-np.inf)
        for i,h in enumerate(np.diff(t)):
            c=self.coefficients[i]
            c[:,0]=self.q[i];c[:,1]=h*v[i];c[:,2]=.5*h*h*a[i]
            rhs=np.array([self.q[i+1]-c[:,0]-c[:,1]-c[:,2],h*v[i+1]-c[:,1]-2*c[:,2],h*h*a[i+1]-2*c[:,2]])
            c[:,3:6]=np.linalg.solve(matrix,rhs).T
        # Match the incoming jerk at the final knot and reach HOLD with zero jerk.
        if len(t)>2:
            prev=self.coefficients[-2];hp=t[-2]-t[-3]
            start_j=np.array([np.polynomial.polynomial.polyval(1.,np.polynomial.polynomial.polyder(p,3))/hp**3 for p in prev])
        else:
            start_j=np.zeros(14)
        from .start_transition import septic_hermite
        self.coefficients[-1]=septic_hermite(self.q[-2],v[-2],a[-2],start_j,
            self.q[-1],np.zeros(14),np.zeros(14),np.zeros(14),t[-1]-t[-2])
        self.peak_velocity=self.peak_acceleration=self.peak_jerk=0.
        self.max_linear_deviation=0.
        self.position_min=np.full(14,np.inf);self.position_max=np.full(14,-np.inf)
        for i,h in enumerate(np.diff(t)):
            for j,p in enumerate(self.coefficients[i]):
                lo,hi=extrema(p)
                self.position_min[j]=min(self.position_min[j],lo)
                self.position_max[j]=max(self.position_max[j],hi)
                for order,name in [(1,'peak_velocity'),(2,'peak_acceleration'),(3,'peak_jerk')]:
                    lo,hi=extrema(p,order)
                    setattr(self,name,max(getattr(self,name),abs(lo)/h**order,abs(hi)/h**order))
                diff=p.copy();diff[0]-=self.q[i,j];diff[1]-=self.q[i+1,j]-self.q[i,j]
                lo,hi=extrema(diff)
                self.max_linear_deviation=max(self.max_linear_deviation,abs(lo),abs(hi))
        h0=t[1]-t[0]
        self.start_jerk=np.array([np.polynomial.polynomial.polyval(0.,np.polynomial.polynomial.polyder(p,3))/h0**3 for p in self.coefficients[0]])
        self.end_jerk=np.zeros(14)
        jumps=[float(np.max(np.abs(self.start_jerk)))]
        for i in range(1,len(self.coefficients)):
            hl=t[i]-t[i-1];hr=t[i+1]-t[i]
            jl=np.array([np.polynomial.polynomial.polyval(1.,np.polynomial.polynomial.polyder(p,3))/hl**3 for p in self.coefficients[i-1]])
            jr=np.array([np.polynomial.polynomial.polyval(0.,np.polynomial.polynomial.polyder(p,3))/hr**3 for p in self.coefficients[i]])
            jumps.append(float(np.max(np.abs(jr-jl))))
        jumps.append(float(np.max(np.abs(self.end_jerk))))
        self.peak_jerk_jump=max(jumps)
        self.coefficients.setflags(write=False)
        self.time.setflags(write=False);self.q.setflags(write=False)

    def sample(self,elapsed):
        if not np.isfinite(elapsed) or elapsed<0:raise ValueError('invalid elapsed time')
        if elapsed>=self.duration:return self.q[-1].copy(),np.zeros(14),np.zeros(14)
        i=max(0,min(len(self.time)-2,int(np.searchsorted(self.time,elapsed,side='right')-1)))
        h=self.time[i+1]-self.time[i];u=(elapsed-self.time[i])/h
        return tuple(np.array([np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(p,k)) for p in self.coefficients[i]])/h**k for k in range(3))


class C2ReplayPlan(C2Curve):
    def __init__(self, metadata, arrays, limits, speed_scale, *, jerk_limit, jerk_jump_limit):
        if not np.isfinite(jerk_limit) or jerk_limit<=0 or not np.isfinite(jerk_jump_limit) or jerk_jump_limit<=0:
            raise ValueError('explicit jerk and jerk-jump limits required')
        raw=ReplayPlan(metadata,arrays,limits,speed_scale,_raw_only=True)
        super().__init__(raw.time,raw.q)
        self.limits,self.speed_scale=limits,speed_scale
        self.jerk_limit,self.jerk_jump_limit=jerk_limit,jerk_jump_limit
        if (np.any(self.position_min<limits.q_min) or np.any(self.position_max>limits.q_max)
                or self.peak_velocity>limits.velocity or self.peak_acceleration>limits.acceleration
                or self.peak_jerk>jerk_limit or self.peak_jerk_jump>jerk_jump_limit or self.duration>limits.max_duration):
            raise ValueError('C2 planned position/velocity/acceleration/jerk/duration limit')
        # Any two publisher ticks within allowed gap have bounded target distance.
        if self.peak_velocity*limits.cycle_timeout>limits.step:
            raise ValueError('C2 maximum tick interval exceeds target step budget')
        self.coefficients.setflags(write=False)

    @classmethod
    def load(cls,path,limits,speed_scale,*,jerk_limit,jerk_jump_limit):
        m,a=load_replay_trajectory(path,limits.max_samples)
        return cls(m,a,limits,speed_scale,jerk_limit=jerk_limit,
                   jerk_jump_limit=jerk_jump_limit)


class RelativeC2ReplayPlan(C2ReplayPlan):
    """C2 plan whose recorded displacement is rebased onto one acquire snapshot.

    This object is only an offline plan. Hardware execution still requires the
    existing permission manifest and its independent approvals.
    """
    def __init__(self, metadata, arrays, limits, speed_scale, *, jerk_limit,
                 jerk_jump_limit, acquire_q, motion_scale=1.0,
                 start_index=0, end_index=None):
        q_source = np.asarray(arrays.get("q_measured"), dtype=float)
        time_source = np.asarray(arrays.get("timestamp"), dtype=float)
        if q_source.ndim != 2 or q_source.shape[1:] != (14,):
            raise ValueError("invalid recorded arm positions")
        n = len(q_source)
        end_index = n if end_index is None else end_index
        if (type(start_index) is not int or type(end_index) is not int or
                not 0 <= start_index < end_index <= n or end_index-start_index < 3):
            raise ValueError("relative replay segment must contain at least three samples")
        acquire = np.asarray(acquire_q, dtype=float)
        if acquire.shape != (14,) or not np.isfinite(acquire).all():
            raise ValueError("acquire_q must be one finite 14-axis snapshot")
        if not np.isfinite(motion_scale) or motion_scale <= 0:
            raise ValueError("motion_scale must be finite and positive")
        if time_source.shape != (n,) or not np.isfinite(time_source).all():
            raise ValueError("invalid recorded timestamps")
        if metadata.get("sample_count") != n or metadata.get("data_integrity") != "PASS":
            raise ValueError("source metadata integrity is not PASS")
        source_shapes = {"timestamp": (n,), "cycle_timestamp": (n,),
                         "q_measured": (n, 14), "dq_measured": (n, 14),
                         "q_ref": (n, 14), "waist_q": (n, 3),
                         "torso_roll_pitch": (n, 2)}
        for key, shape in source_shapes.items():
            value = np.asarray(arrays.get(key), dtype=float)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f"invalid source field {key}")
        if (not np.array_equal(arrays.get("arm_motor_ids"), np.arange(15, 29)) or
                not np.array_equal(arrays.get("waist_motor_ids"), np.arange(12, 15))):
            raise ValueError("source motor mapping must be 15..28 and 12..14")
        source_dt = np.diff(time_source)
        cycle_time = np.asarray(arrays["cycle_timestamp"], dtype=float)
        if (np.any(source_dt <= 0) or np.any(source_dt > limits.gap) or
                np.any(np.diff(cycle_time) <= 0) or np.any(cycle_time < time_source)):
            raise ValueError("source time is discontinuous or invalid")
        if not np.isclose(metadata.get("duration_s", -1),
                          time_source[-1]-time_source[0], atol=1e-8, rtol=1e-8):
            raise ValueError("source metadata duration mismatch")
        for key, expected in (("start_q", q_source[0]), ("end_q", q_source[-1])):
            if key in metadata and not np.allclose(metadata[key], expected, atol=1e-10, rtol=0):
                raise ValueError(f"source metadata {key} mismatch")
        selected = {key: np.asarray(value)[start_index:end_index].copy()
                    for key, value in arrays.items()
                    if key in ("timestamp", "cycle_timestamp", "q_measured", "dq_measured",
                               "q_ref", "waist_q", "torso_roll_pitch")}
        selected["arm_motor_ids"] = np.asarray(arrays.get("arm_motor_ids")).copy()
        selected["waist_motor_ids"] = np.asarray(arrays.get("waist_motor_ids")).copy()
        recorded_start = selected["q_measured"][0].copy()
        source_q = selected["q_measured"].copy()
        source_time = selected["timestamp"].copy()
        source_waist = selected["waist_q"].copy()
        selected["q_measured"] = acquire + float(motion_scale) * (selected["q_measured"]-recorded_start)
        selected["dq_measured"] = float(motion_scale) * selected["dq_measured"]
        selected["q_ref"] = acquire + float(motion_scale) * (selected["q_ref"]-recorded_start)
        selected_metadata = dict(metadata)
        selected_metadata.update(sample_count=end_index-start_index,
                                 duration_s=float(selected["timestamp"][-1]-selected["timestamp"][0]),
                                 start_q=selected["q_measured"][0].tolist(),
                                 end_q=selected["q_measured"][-1].tolist(),
                                 data_integrity="PASS")
        super().__init__(selected_metadata, selected, limits, speed_scale,
                         jerk_limit=jerk_limit, jerk_jump_limit=jerk_jump_limit)
        self.motion_scale = float(motion_scale)
        self.start_index, self.end_index = start_index, end_index
        self.recorded_start_q = recorded_start
        self.acquire_q = acquire.copy()
        self.source_q = source_q
        self.source_time = source_time
        self.source_waist_q = source_waist
        if not np.array_equal(self.q[0], acquire):
            raise ValueError("relative first reference must exactly equal acquire snapshot")
        self.acquire_q.setflags(write=False)
        self.recorded_start_q.setflags(write=False)
        self.source_q.setflags(write=False)
        self.source_time.setflags(write=False)
        self.source_waist_q.setflags(write=False)
        self.source_metadata = metadata

    @classmethod
    def load(cls, path, limits, speed_scale, *, jerk_limit, jerk_jump_limit,
             acquire_q, motion_scale=1.0, start_index=0, end_index=None):
        metadata, arrays = load_replay_trajectory(path, limits.max_samples)
        return cls(metadata, arrays, limits, speed_scale, jerk_limit=jerk_limit,
                   jerk_jump_limit=jerk_jump_limit, acquire_q=acquire_q,
                   motion_scale=motion_scale, start_index=start_index,
                   end_index=end_index)


class RelativeReplaySpec:
    """Validated source selection; bind only after capturing the acquire state."""
    def __init__(self, metadata, arrays, limits, speed_scale, *, jerk_limit,
                 jerk_jump_limit, motion_scale=1.0, start_index=0,
                 end_index=None):
        self.metadata = copy.deepcopy(metadata)
        self.arrays = {key: np.asarray(value).copy() for key, value in arrays.items()}
        for value in self.arrays.values():
            value.setflags(write=False)
        self.limits, self.speed_scale = limits, speed_scale
        self.jerk_limit, self.jerk_jump_limit = jerk_limit, jerk_jump_limit
        self.motion_scale = motion_scale
        self.start_index, self.end_index = start_index, end_index

    @classmethod
    def load(cls, path, limits, speed_scale, *, jerk_limit, jerk_jump_limit,
             motion_scale=1.0, start_index=0, end_index=None):
        metadata, arrays = load_replay_trajectory(path, limits.max_samples)
        return cls(metadata, arrays, limits, speed_scale,
                   jerk_limit=jerk_limit, jerk_jump_limit=jerk_jump_limit,
                   motion_scale=motion_scale, start_index=start_index,
                   end_index=end_index)

    def bind(self, acquire_q):
        return RelativeC2ReplayPlan(
            self.metadata, self.arrays, self.limits, self.speed_scale,
            jerk_limit=self.jerk_limit, jerk_jump_limit=self.jerk_jump_limit,
            acquire_q=acquire_q, motion_scale=self.motion_scale,
            start_index=self.start_index, end_index=self.end_index)
