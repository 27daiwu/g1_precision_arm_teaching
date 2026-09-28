"""One monotonic scheduler; late cycles never trigger catch-up command bursts."""
import math
import time


class ControlLoop:
    def __init__(self, frequency_hz):
        if not math.isfinite(frequency_hz) or frequency_hz <= 0:
            raise ValueError('frequency must be finite and positive')
        self.period = 1.0 / frequency_hz
        self.next = time.monotonic()

    def wait(self):
        self.next += self.period
        now = time.monotonic()
        if self.next < now:
            self.next = now
        time.sleep(max(0.0, self.next - now))
