"""Auditable C2 quintic Hermite interpolation on unmodified recorded times."""
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
        return cls(m,a,limits,speed_scale,jerk_limit=jerk_limit,jerk_jump_limit=jerk_jump_limit)
