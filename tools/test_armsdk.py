#!/usr/bin/env python3

import argparse
import math
import signal
import sys
import time

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


# ============================================================
# G1 29DoF joint map
# ============================================================

JOINTS = {
    # Waist
    "waist_yaw": 12,
    "waist_roll": 13,
    "waist_pitch": 14,

    # Left arm
    "left_shoulder_pitch": 15,
    "left_shoulder_roll": 16,
    "left_shoulder_yaw": 17,
    "left_elbow": 18,
    "left_wrist_roll": 19,
    "left_wrist_pitch": 20,
    "left_wrist_yaw": 21,

    # Right arm
    "right_shoulder_pitch": 22,
    "right_shoulder_roll": 23,
    "right_shoulder_yaw": 24,
    "right_elbow": 25,
    "right_wrist_roll": 26,
    "right_wrist_pitch": 27,
    "right_wrist_yaw": 28,
}

UPPER_BODY_JOINTS = list(range(12, 29))

ARM_SDK_WEIGHT_INDEX = 29


class ArmSdkJointTester:
    def __init__(
        self,
        interface,
        kp=40.0,
        kd=1.5,
        dt=0.02,
    ):
        self.kp = kp
        self.kd = kd
        self.dt = dt

        self.running = True
        self.low_state = None
        self.weight = 0.0

        ChannelFactoryInitialize(0, interface)

        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()

        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._lowstate_callback, 10)

    def _lowstate_callback(self, msg):
        self.low_state = msg

    def _wait_for_lowstate(self, timeout=5.0):
        print("Waiting for rt/lowstate...")

        start = time.monotonic()

        while self.low_state is None:
            if time.monotonic() - start > timeout:
                raise RuntimeError(
                    "No rt/lowstate received. "
                    "Check DDS interface / robot connection."
                )
            time.sleep(0.01)

        print("rt/lowstate received.")

    def _write(self):
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.publisher.Write(self.cmd)

    def _set_weight(self, weight):
        self.weight = max(0.0, min(1.0, weight))
        self.cmd.motor_cmd[ARM_SDK_WEIGHT_INDEX].q = self.weight

    def _set_joint(self, joint_id, q):
        motor = self.cmd.motor_cmd[joint_id]

        motor.q = q
        motor.dq = 0.0
        motor.kp = self.kp
        motor.kd = self.kd
        motor.tau = 0.0

    def _capture_pose(self):
        """
        Capture current waist + both arms.
        """
        return {
            joint: float(self.low_state.motor_state[joint].q)
            for joint in UPPER_BODY_JOINTS
        }

    def _fill_hold_command(self, q_hold):
        """
        Important:
        Always provide commands for joints 12~28 while Arm SDK owns
        the upper body.
        """
        for joint in UPPER_BODY_JOINTS:
            self._set_joint(joint, q_hold[joint])

    @staticmethod
    def _smoothstep(x):
        x = max(0.0, min(1.0, x))
        return x * x * (3.0 - 2.0 * x)

    def acquire(self, q_hold, duration=2.0):
        """
        Smoothly increase Arm SDK weight 0 -> 1.
        """
        print("Acquiring Arm SDK...")

        start = time.monotonic()

        while self.running:
            t = time.monotonic() - start

            if t >= duration:
                break

            ratio = self._smoothstep(t / duration)

            self._fill_hold_command(q_hold)
            self._set_weight(ratio)

            self._write()
            time.sleep(self.dt)

        self._fill_hold_command(q_hold)
        self._set_weight(1.0)
        self._write()

        print("Arm SDK acquired.")

    def release(self, q_hold, duration=2.0):
        """
        Smoothly release Arm SDK and then stop publishing.
        """
        print("\nReleasing Arm SDK...")

        start_weight = self.weight
        start = time.monotonic()

        while True:
            t = time.monotonic() - start

            if t >= duration:
                break

            ratio = self._smoothstep(t / duration)
            weight = start_weight * (1.0 - ratio)

            self._fill_hold_command(q_hold)
            self._set_weight(weight)

            self._write()
            time.sleep(self.dt)

        # Final explicit release frame.
        self._fill_hold_command(q_hold)
        self._set_weight(0.0)
        self._write()

        # Only allow DDS a short time to send the final frame.
        time.sleep(0.05)

        print("Arm SDK released.")

    def move_joint_relative(
        self,
        joint_name,
        offset,
        move_time=2.0,
        hold_time=2.0,
        return_to_start=True,
    ):
        """
        Move one upper-body joint relative to its current position.

        Example:
            left_wrist_roll +0.10 rad
        """

        if joint_name not in JOINTS:
            raise ValueError(f"Unknown joint: {joint_name}")

        joint_id = JOINTS[joint_name]

        self._wait_for_lowstate()

        q_hold = self._capture_pose()

        q_start = q_hold[joint_id]
        q_target = q_start + offset

        print("")
        print("=======================================")
        print(f"joint       : {joint_name}")
        print(f"joint id    : {joint_id}")
        print(f"start q     : {q_start:+.4f} rad")
        print(f"offset      : {offset:+.4f} rad")
        print(f"target q    : {q_target:+.4f} rad")
        print(f"move time   : {move_time:.2f} s")
        print(f"hold time   : {hold_time:.2f} s")
        print("=======================================")
        print("")

        try:
            # --------------------------------------------------
            # 1. Acquire upper body at current pose
            # --------------------------------------------------
            self.acquire(q_hold)

            if not self.running:
                return

            # --------------------------------------------------
            # 2. Move selected joint
            # --------------------------------------------------
            if getattr(self, 'diagnostics', None) is not None:
                self.diagnostics.phase = 'MOVE'
            print("Moving joint...")

            start = time.monotonic()

            while self.running:
                t = time.monotonic() - start

                if t >= move_time:
                    break

                ratio = self._smoothstep(t / move_time)

                q_cmd = (
                    q_start
                    + ratio * (q_target - q_start)
                )

                # First hold all 12~28.
                self._fill_hold_command(q_hold)

                # Then override selected joint.
                self._set_joint(joint_id, q_cmd)

                self._set_weight(1.0)
                self._write()

                time.sleep(self.dt)

            # --------------------------------------------------
            # 3. Hold target
            # --------------------------------------------------
            if self.running and hold_time > 0:
                if getattr(self, 'diagnostics', None) is not None:
                    self.diagnostics.phase = 'HOLD'
                print("Holding target...")

                start = time.monotonic()

                while self.running:
                    if time.monotonic() - start >= hold_time:
                        break

                    self._fill_hold_command(q_hold)
                    self._set_joint(joint_id, q_target)

                    self._set_weight(1.0)
                    self._write()

                    time.sleep(self.dt)

            # --------------------------------------------------
            # 4. Return
            # --------------------------------------------------
            if self.running and return_to_start:
                if getattr(self, 'diagnostics', None) is not None:
                    self.diagnostics.phase = 'RETURN'
                print("Returning to start pose...")

                # Actual position at start of return.
                q_return_start = float(
                    self.low_state.motor_state[joint_id].q
                )

                start = time.monotonic()

                while self.running:
                    t = time.monotonic() - start

                    if t >= move_time:
                        break

                    ratio = self._smoothstep(t / move_time)

                    q_cmd = (
                        q_return_start
                        + ratio * (q_start - q_return_start)
                    )

                    self._fill_hold_command(q_hold)
                    self._set_joint(joint_id, q_cmd)

                    self._set_weight(1.0)
                    self._write()

                    time.sleep(self.dt)

        finally:
            # Ctrl+C / exception / normal completion all come here.
            self.release(q_hold)

    def stop(self):
        self.running = False


def print_joint_list():
    print("Available joints:")
    for name, index in JOINTS.items():
        print(f"  {index:2d}  {name}")


def main():
    parser = argparse.ArgumentParser(
        description="Unitree G1 29DoF Arm SDK single-joint test tool"
    )

    parser.add_argument(
        "interface",
        help="DDS network interface, e.g. eth0 / enp130s0",
    )

    parser.add_argument(
        "--joint",
        type=str,
        help="Joint name to control",
    )

    parser.add_argument(
        "--offset",
        type=float,
        default=0.10,
        help="Relative target offset in radians. Default: 0.10",
    )

    parser.add_argument(
        "--move-time",
        type=float,
        default=2.0,
        help="Movement duration in seconds. Default: 2.0",
    )

    parser.add_argument(
        "--hold-time",
        type=float,
        default=2.0,
        help="Target hold duration in seconds. Default: 2.0",
    )

    parser.add_argument(
        "--kp",
        type=float,
        default=40.0,
        help="Position gain. Default: 40",
    )

    parser.add_argument(
        "--kd",
        type=float,
        default=1.5,
        help="Damping gain. Default: 1.5",
    )

    parser.add_argument(
        "--no-return",
        action="store_true",
        help="Do not return the joint to its initial position",
    )

    parser.add_argument(
        "--list-joints",
        action="store_true",
        help="Show supported joints",
    )

    parser.add_argument('--diagnostics-log', help='new JSONL file; buffered motor14/Write diagnostics only')
    args = parser.parse_args()

    if args.list_joints:
        print_joint_list()
        return

    if args.joint is None:
        parser.error("--joint is required")

    if args.joint not in JOINTS:
        print(f"Unknown joint: {args.joint}\n")
        print_joint_list()
        sys.exit(1)

    diagnostics_stream = None
    if args.diagnostics_log:
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
        from g1_dual_arm_teaching.sdk.reference_diagnostics import ReferenceDiagnostics
        path = Path(args.diagnostics_log)
        path.parent.mkdir(parents=True, exist_ok=True)
        diagnostics_stream = path.open('x', encoding='utf-8')

    tester = ArmSdkJointTester(
        interface=args.interface,
        kp=args.kp,
        kd=args.kd,
    )

    observer = ReferenceDiagnostics(tester) if diagnostics_stream is not None else None
    if observer is not None:
        observer.parameters.update(joint=args.joint, offset=args.offset, move_time=args.move_time,
                                   hold_time=args.hold_time, return_to_start=not args.no_return)

    def signal_handler(signum, frame):
        print("\nStop requested.")
        tester.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("")
    print("WARNING")
    print("Make sure the robot is stable and the arm workspace is clear.")
    print("Ctrl+C will trigger Arm SDK release instead of killing immediately.")
    print("")

    input("Press Enter to start...")

    try:
        tester.move_joint_relative(
            joint_name=args.joint,
            offset=args.offset,
            move_time=args.move_time,
            hold_time=args.hold_time,
            return_to_start=not args.no_return,
        )
    finally:
        if observer is not None:
            try:
                observer.finish(diagnostics_stream)
            finally:
                diagnostics_stream.close()



if __name__ == "__main__":
    main()
