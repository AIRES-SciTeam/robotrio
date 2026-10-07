import time

from mission.mission_block.mission_block import MissionBlock
from safety_commander.commander import CommandResult


class Land(MissionBlock):
    def __init__(
        self,
        mission_env,
        timeout : float = 60 # s
    ):
        super().__init__("Land", mission_env)
        self.timeout = timeout
        self.start_time = None

    def start(self):
        self.start_time = None
        super().start()

    def tick(self):
        if self._can_tick():
            if self.start_time is None:
                self.start_time = time.monotonic()
                res = self.menv.commander.land()
                if res != CommandResult.SUCCESS:
                    self._fail(f"{res.name}")
                    return self._state
            if self.menv.commander.is_landed() is True:
                self._complete()
            elif time.monotonic() - self.start_time > self.timeout:
                self._fail("landing timeout")
        return self._state

