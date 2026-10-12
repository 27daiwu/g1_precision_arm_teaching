"""Separately approved septic startup connector used only by Replay permission."""
import numpy as np
from .replay_c2 import extrema


def septic_hermite(q0, v0, a0, j0, q1, v1, a1, j1, duration):
    """Return normalized-time degree-seven coefficients matching endpoint jets."""
    h=float(duration)
    values=[np.asarray(x,dtype=float) for x in (q0,v0,a0,j0,q1,v1,a1,j1)]
    if h<=0 or not np.isfinite(h) or any(x.shape!=(14,) or not np.isfinite(x).all() for x in values):
        raise ValueError('invalid septic endpoint data')
    q0,v0,a0,j0,q1,v1,a1,j1=values
    c=np.zeros((14,8));c[:,0]=q0;c[:,1]=h*v0;c[:,2]=.5*h*h*a0;c[:,3]=h**3*j0/6.
    matrix=np.array([[1,1,1,1],[4,5,6,7],[12,20,30,42],[24,60,120,210]],dtype=float)
    rhs=np.stack((q1-c[:,:4].sum(axis=1),h*v1-c[:,1]-2*c[:,2]-3*c[:,3],
                  h*h*a1-2*c[:,2]-6*c[:,3],h**3*j1-6*c[:,3]),axis=1)
    c[:,4:]=np.linalg.solve(matrix,rhs.T).T
    return c


class SepticStartTransition:
    def __init__(self,start,end,duration,limits,jerk_limit,*,end_velocity=None,end_acceleration=None,
                 end_jerk=None,separately_approved=False,space_checked=False):
        if not separately_approved or not space_checked:
            raise ValueError('separate transition and mechanical-space approval required')
        self.duration=float(duration)
        start,end=np.asarray(start),np.asarray(end)
        if start.shape!=(14,) or end.shape!=(14,) or not np.isfinite([start,end]).all() or not np.isfinite(duration) or duration<=0:
            raise ValueError('invalid transition')
        zero=np.zeros(14)
        self.coefficients=septic_hermite(start,zero,zero,zero,end,
            zero if end_velocity is None else end_velocity,
            zero if end_acceleration is None else end_acceleration,
            zero if end_jerk is None else end_jerk,self.duration)
        self.peak_velocity=self.peak_acceleration=self.peak_jerk=0.
        for j,p in enumerate(self.coefficients):
            lo,hi=extrema(p)
            if lo<limits.q_min[j] or hi>limits.q_max[j]:raise ValueError('position limit')
            for order,bound in [(1,limits.velocity),(2,limits.acceleration),(3,jerk_limit)]:
                lo,hi=extrema(p,order)
                peak=max(abs(lo),abs(hi))/duration**order
                if peak>bound or (order==1 and peak*limits.cycle_timeout>limits.step):raise ValueError('transition dynamic/step limit')
                if order==1:self.peak_velocity=max(self.peak_velocity,peak)
                elif order==2:self.peak_acceleration=max(self.peak_acceleration,peak)
                else:self.peak_jerk=max(self.peak_jerk,peak)
    def sample(self,t):
        if not np.isfinite(t) or not 0<=t<=self.duration:raise ValueError('transition time')
        u=t/self.duration
        return tuple(np.array([np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(p,k)) for p in self.coefficients])/self.duration**k for k in range(4))
