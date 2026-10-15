import time

import numpy as np

from mission.mission_block.mission_block import MissionBlock
from safety_commander.commander import CommandResult


class ReachCourse(MissionBlock):
    def __init__(
        self, 
        mission_env,
        yaw : float, # rad
        eps : float = 0.15, # rad
        pos : tuple[float, float, float] | None = None,
        max_yaw_rate : float = 0.3 # rad/s, target slew rate
    ):
        super().__init__("ReachCourse", mission_env)
        self.yaw = yaw
        self.pos = pos
        self.eps = eps
        if not np.isfinite(max_yaw_rate) or max_yaw_rate <= 0:
            raise ValueError("max_yaw_rate must be positive and finite")
        self.max_yaw_rate = max_yaw_rate
        self._command_yaw = None
        self._last_tick_at = None

    def start(self):
        self._command_yaw = None
        self._last_tick_at = None
        super().start()

    def tick(self):
        if self._can_tick():
            _, _, _, q = self.menv.commander.get_drone_position()
            if q is None:
                self._fail(f"unknown drone orientation: {q}")
                return self._state
            w, x, y, z = q
            yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            delta = self.yaw - yaw
            error = np.arctan2(np.sin(delta), np.cos(delta))
            now = time.monotonic()
            if self._command_yaw is None:
                self._command_yaw = float(yaw)
            else:
                delta = self.yaw - self._command_yaw
                remaining = np.arctan2(np.sin(delta), np.cos(delta))
                step = self.max_yaw_rate * (now - self._last_tick_at)
                next_yaw = self._command_yaw + np.clip(remaining, -step, step)
                self._command_yaw = float(np.arctan2(np.sin(next_yaw), np.cos(next_yaw)))
            self._last_tick_at = now
            res = self.menv.commander.set_offboard_target(pos=self.pos, yaw=self._command_yaw)
            if res != CommandResult.SUCCESS:
                self._fail(res.name)
            else:
                remaining = self.yaw - self._command_yaw
                target_error = np.arctan2(np.sin(remaining), np.cos(remaining))
                if abs(target_error) < 1e-6 and abs(error) <= self.eps:
                    self._complete()
        return self._state

