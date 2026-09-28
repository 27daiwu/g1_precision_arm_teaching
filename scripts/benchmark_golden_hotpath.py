#!/usr/bin/env python3
"""Offline only: no DDS initialization/publisher; fake backend is labeled."""
import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import statistics
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import numpy as np
from g1_dual_arm_teaching.sdk.golden_controller import ArmSdkGoldenController
from g1_dual_arm_teaching.sdk.arm_sdk_types import JointState
from g1_dual_arm_teaching.utils.config import load_config
from g1_dual_arm_teaching.golden_cli import BufferedDiagnostics


def measure(call, n):
    times=[]
    for _ in range(n):
        start=time.perf_counter_ns();call();times.append((time.perf_counter_ns()-start)/1e6)
    return dict(mean_ms=statistics.mean(times),p95_ms=float(np.percentile(times,95)),max_ms=max(times))


def legacy_breakdown(c, n):
    totals=dict(field_iteration_us=0.,expected_command_build_us=0.,deep_compare_us=0.,other_us=0.)
    for _ in range(n):
        start=time.perf_counter_ns()
        for i,motor in enumerate(c.cmd.motor_cmd):
            for name in ('q','dq','kp','kd','tau','mode','reserve'):
                t=time.perf_counter_ns()
                default=c.defaults.motor_cmd[i]
                if not hasattr(default,name):continue
                actual=getattr(motor,name);expected=getattr(default,name)
                totals['field_iteration_us']+=(time.perf_counter_ns()-t)/1000
                t=time.perf_counter_ns()
                if 12<=i<=28 and name in ('q','dq','kp','kd','tau'):
                    expected=dict(q=c.q_hold[i-12],dq=0.,kp=40.,kd=1.5,tau=0.)[name]
                elif i==29 and name=='q':expected=.5
                totals['expected_command_build_us']+=(time.perf_counter_ns()-t)/1000
                t=time.perf_counter_ns()
                assert np.allclose(actual,expected,rtol=0,atol=1e-6)
                totals['deep_compare_us']+=(time.perf_counter_ns()-t)/1000
        # Instrumented decomposition includes timer overhead, unlike the baseline call benchmark.
    return {key:value/n for key,value in totals.items() if key!='other_us'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations',type=int,default=500)
    args=parser.parse_args()
    if args.iterations<1:parser.error('iterations must be positive')
    try:
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        factory,crc,backend=unitree_hg_msg_dds__LowCmd_,CRC(),'SDK_SERIALIZATION_WITH_FAKE_PUBLISHER'
    except ImportError:
        factory=lambda:SimpleNamespace(mode_pr=0,mode_machine=0,reserve=[0]*4,crc=0,
            motor_cmd=[SimpleNamespace(q=0.,dq=0.,kp=0.,kd=0.,tau=0.,mode=0,reserve=0) for _ in range(35)])
        crc,backend=SimpleNamespace(Crc=lambda msg:0),'FAKE_MESSAGE_FAKE_CRC_NO_CDR_NO_DDS'
    transport=SimpleNamespace(_message=factory,_crc=crc,publisher=SimpleNamespace(Write=lambda message:True),
        state=lambda:JointState(np.arange(15,29)/100,time.monotonic(),np.zeros(14),np.arange(12,15)/100,np.zeros(3)))
    buffer=BufferedDiagnostics()
    c=ArmSdkGoldenController(transport,load_config('configs'),emit=buffer.emit)
    c.capture_upper_body_pose();c._frame(0.,'ACQUIRE');c._frame(.5,'ACQUIRE')
    # No robot, no real waiting: benchmark steady current-pose frames directly.
    full=measure(lambda:c._full_guard(c.cmd,.5),min(args.iterations,100))
    substeps=legacy_breakdown(c,min(args.iterations,40))
    guard=measure(lambda:c._guard(c.cmd,.5),args.iterations)
    logging_cost=measure(lambda:c.emit(dict(event='BENCHMARK',q=[.1]*17)),args.iterations)
    c._frame(.5,'HOLD')  # Exclude the one representative HOLD copy from steady-state timing.
    hotpath=measure(lambda:c._frame(.5,'HOLD'),args.iterations)
    codec={}
    if hasattr(c.cmd,'serialize'):
        data=c.cmd.serialize()
        codec['serialization']=measure(c.cmd.serialize,min(args.iterations,100))
        codec['deserialization']=measure(lambda:type(c.cmd).deserialize(data),min(args.iterations,100))
    result=dict(event='GOLDEN_HOT_PATH_OFFLINE_BENCHMARK',backend=backend,iterations=args.iterations,
        real_robot_started=False,sleep_included=False,
        legacy_full_guard_single_pass=full, legacy_frame_guard_passes=2,
        WIRE_GUARD_PROFILE=dict(substeps,allocation_us=None,
             allocation_note='included in field/expected/NumPy comparison; not independently isolated',
             codec=codec or 'UNAVAILABLE_NO_SDK'),
        runtime_guard=guard,buffered_logging=logging_cost,steady_frame_non_sleep=hotpath,
        targets_met=dict(guard=guard['mean_ms']<.5,logging=logging_cost['mean_ms']<.2,non_sleep=hotpath['mean_ms']<2.))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
