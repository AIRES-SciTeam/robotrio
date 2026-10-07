import time

import numpy as np

from mission.mission_block.mission_block import MissionBlock
from safety_commander.commander import CommandResult


class MoveTo(MissionBlock):
    def __init__(
        self,
        mission_env,
        pos : tuple[float, float, float], # m, NED
        eps : float = 0.05, # m
        speed_eps : float = 0.05, # m/s
        settle_time : float = 2.0, # s
        yaw : float | None = None, # rad, NED
        max_yaw_rate : float = 0.3, # rad/s, target slew rate
        yaw_eps : float = 0.15 # rad
    ):
        super().__init__("MoveTo", mission_env)
        self.pos = pos
        self.yaw = yaw
        self.max_yaw_rate = max_yaw_rate
        self.yaw_eps = yaw_eps
        if not np.isfinite(max_yaw_rate) or max_yaw_rate <= 0:
            raise ValueError("max_yaw_rate must be positive and finite")
        self._command_yaw = None
        self._last_tick_at = None
        self.eps = eps
        self.speed_eps = speed_eps
        self.settle_time = settle_time
        self._settled_since = None

    def start(self):
        self._command_yaw = None
        self._last_tick_at = None
        self._settled_since = None
        super().start()

    def tick(self):
        if self._can_tick():
            x, y, z, q = self.menv.commander.get_drone_position()
            self.menv.logger.debug("Position = %s, destination = %s", (x, y, z), self.pos)
            if any(value is None for value in (x, y, z)):
                self._fail(f"unknown drone position: {(x, y, z)}")
                return self._state
            now = time.monotonic()
            yaw_reached = True
            if self.yaw is not None:
                if q is None:
                    self._fail("Unknown drone orientation")
                    return self._state
                w, qx, qy, qz = q
                current_yaw = np.arctan2(2*(w*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
                if self._command_yaw is None:
                    self._command_yaw = float(current_yaw)
                else:
                    delta = self.yaw - self._command_yaw
                    remaining = np.arctan2(np.sin(delta), np.cos(delta))
                    step = self.max_yaw_rate * (now - self._last_tick_at)
                    angle = self._command_yaw + np.clip(remaining, -step, step)
                    self._command_yaw = float(np.arctan2(np.sin(angle), np.cos(angle)))
                self._last_tick_at = now
                error = self.yaw - current_yaw
                remaining = self.yaw - self._command_yaw
                yaw_reached = (
                    abs(np.arctan2(np.sin(error), np.cos(error))) <= self.yaw_eps
                    and abs(np.arctan2(np.sin(remaining), np.cos(remaining))) < 1e-6
                )
            res = self.menv.commander.set_offboard_target(pos=self.pos, yaw=self._command_yaw)
            if res != CommandResult.SUCCESS:
                self._fail(f"{res}")
                return self._state
            velocity = self.menv.commander.get_drone_velocity()
            distance = np.linalg.norm(np.array((x, y, z)) - np.array(self.pos))
            speed = None if any(v is None for v in velocity) else np.linalg.norm(velocity)
            if distance <= self.eps and speed is not None and speed <= self.speed_eps and yaw_reached:
                now = time.monotonic()
                if self._settled_since is None:
                    self._settled_since = now
                    self.menv.logger.debug("MoveTo settling: distance=%.3f, speed=%.3f", distance, speed)
                if now - self._settled_since >= self.settle_time:
                    self._complete()
            else:
                self._settled_since = None
        return self._state
