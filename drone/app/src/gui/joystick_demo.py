"""Временный источник данных для проверки индикаторов без джойстика."""
import math
import time


class JoystickDemo:
    def __init__(self, panel):
        self.panel = panel
        self._timer = None
        self._started_at = None

    def start(self):
        if self._started_at is not None:
            return
        self._started_at = time.monotonic()
        self._tick()

    def _tick(self):
        elapsed = time.monotonic() - self._started_at
        values = {
            "roll": round(1000 * math.sin(elapsed * 1.4)),
            "pitch": round(1000 * math.sin(elapsed * 0.9)),
            "yaw": round(1000 * math.sin(elapsed * 1.9)),
            "throttle": round(500 + 500 * math.sin(elapsed * 0.7)),
        }
        for name, value in values.items():
            self.panel.axis_values[name].set(value)
        sensitivity = (10, 25, 50, 75, 100)[int(elapsed / 3) % 5]
        self.panel.sensitivity_text.set(f"{sensitivity}%")
        self._timer = self.panel.after(33, self._tick)

    def stop(self):
        if self._timer is not None:
            self.panel.after_cancel(self._timer)
            self._timer = None
        self._started_at = None
