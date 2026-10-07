from mission.mission_block.mission_block import MissionBlock
from safety_commander.commander import CommandResult


class FlightMode(MissionBlock):
    def __init__(
        self,
        mission_env,
        mode : str,
        timeout : float = 60 # s
    ):
        super().__init__(f"Flight mode {mode}", mission_env)
        self.timeout = timeout
        self.mode = mode

    def tick(self):
        if self._can_tick():
            res = self.menv.commander.set_flight_mode(self.mode, self.timeout)
            if res == CommandResult.SUCCESS:
                self._complete()
            else:
                self._fail(res.name)
        return self._state

