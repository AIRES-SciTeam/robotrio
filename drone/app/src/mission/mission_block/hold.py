import time

from mission.mission_block.mission_block import MissionBlock
from safety_commander.commander import CommandResult


class Hold(MissionBlock):
    def __init__(
        self,
        mission_env,
        timeout : float | None = None, # s
        pos : tuple[float, float, float] | None = None
    ):
        super().__init__("Hold", mission_env)
        self.timeout = timeout
        self.pos = pos
        self.start_time = None

    def start(self):
        self.start_time = None
        super().start()

    def tick(self):
        if self._can_tick():
            if self.timeout is None:
                res = self.menv.commander.set_offboard_target(pos=self.pos)
                if res != CommandResult.SUCCESS:
                    self._fail(f"{res.name}")
                return self._state
            if self.start_time is None:
                self.start_time = time.monotonic()
            if time.monotonic() - self.start_time <= self.timeout:
                res = self.menv.commander.set_offboard_target(pos=self.pos)
                if res != CommandResult.SUCCESS:
                    self._fail(f"{res.name}")
                return self._state
            self._complete()
        return self._state

