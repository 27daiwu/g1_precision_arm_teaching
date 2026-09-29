"""Cached scalar invariants; no CDR, JSON or NumPy scalar comparisons per frame."""
import math


class RuntimeWireGuard:
    def __init__(self, defaults, reference, motor14_kp=40., teach_waist=False,
                 static_hold_waist_kp=None, static_hold_arm_kp=None, waist_kp_by_axis=None):
        if motor14_kp not in (40., 50., 60.):
            raise ValueError('GOLDEN_WIRE_GUARD invalid motor14 Kp')
        self.motor14_kp = float(motor14_kp)
        self.teach_waist = bool(teach_waist)
        self.static_hold_waist_kp = static_hold_waist_kp
        self.static_hold_arm_kp = static_hold_arm_kp
        self.waist_kp_by_axis = waist_kp_by_axis
        self.teach_q = None
        self.teach_kp = None
        self.reference = tuple(float(q) for q in reference)
        if len(self.reference) != 17 or not all(map(math.isfinite, self.reference)):
            raise ValueError('GOLDEN_WIRE_GUARD invalid captured reference')
        self.expected = []
        for i, motor in enumerate(defaults.motor_cmd):
            fields = []
            for name in ('q', 'dq', 'kp', 'kd', 'tau', 'mode', 'reserve'):
                if not hasattr(motor, name):
                    continue
                value = getattr(motor, name)
                if 12 <= i <= 28:
                    default_kp = (waist_kp_by_axis[i-12] if waist_kp_by_axis is not None and 12 <= i <= 14
                                  else static_hold_waist_kp if static_hold_waist_kp is not None and i in (12, 13, 14)
                                  else static_hold_arm_kp if static_hold_arm_kp is not None and 15 <= i <= 28
                                  else 60. if self.teach_waist and i in (12,13)
                                  else self.motor14_kp if i == 14 else 40.)
                    value = dict(q=self.reference[i-12], dq=0., kp=default_kp, kd=1.5, tau=0.).get(name, value)
                fields.append((name, float(value)))
            self.expected.append(tuple(fields))
        self.top = tuple((name, tuple(getattr(defaults, name)) if name=='reserve' else getattr(defaults, name))
                         for name in ('mode_pr', 'mode_machine', 'reserve') if hasattr(defaults,name))

    def check(self, message, weight):
        if not math.isfinite(weight) or not 0 <= weight <= 1:
            raise ValueError('GOLDEN_WIRE_GUARD invalid weight')
        motors = message.motor_cmd
        if len(motors) != len(self.expected) or len({id(m) for m in motors}) != len(motors):
            raise ValueError('GOLDEN_WIRE_GUARD wrong controlled set/aliased slots')
        for i, fields in enumerate(self.expected):
            motor = motors[i]
            for name, expected in fields:
                if self.teach_q is not None and 15 <= i <= 28 and name == 'q':
                    expected = self.teach_q[i-15]
                if self.teach_kp is not None and 15 <= i <= 28 and name == 'kp':
                    expected = self.teach_kp[i-15]
                if i == 29 and name == 'q':
                    expected = weight
                elif i == 19 and name == 'q' and self.selected_q is not None:
                    expected = self.selected_q
                actual = getattr(motor, name)
                if i == 29 and name == 'q' and (not math.isfinite(actual) or not 0 <= actual <= 1):
                    raise ValueError('GOLDEN_WIRE_GUARD motor29.q range')
                # q permits float32 wire representation. Other invariants are exact.
                valid = (math.isfinite(actual) and
                         (abs(actual-expected) <= 1e-6 if name=='q' else actual==expected))
                if not valid:
                    raise ValueError(f'GOLDEN_WIRE_GUARD motor{i}.{name}')
        for name, expected in self.top:
            actual = tuple(getattr(message, name)) if name=='reserve' else getattr(message, name)
            if actual != expected:
                raise ValueError(f'GOLDEN_WIRE_GUARD {name}')

    selected_q = None
