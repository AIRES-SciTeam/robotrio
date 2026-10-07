from mission.mission_block.mission_block import MissionBlock
from safety_commander.commander import CommandResult


class Arm(MissionBlock):
    def __init__(
        self, 
        mission_env
    ):
        super().__init__("Arm", mission_env)

    def tick(self):
        if self._can_tick():
            if self.menv.commander.armed():
                self._complete()
                return self._state
            res = self.menv.commander.arm()
            if res == CommandResult.SUCCESS:
                self._complete()
            else:
                self.menv.logger.warning("Arm: %s; retrying next tick", res.name)
        return self._state



class Disarm(MissionBlock):
    def __init__(
        self, 
        mission_env
    ):
        super().__init__("Disarm", mission_env)

    def tick(self):
        if self._can_tick():
            if self.menv.commander.armed() is False:
                self._complete()
                return self._state
            res = self.menv.commander.disarm()
            if res == CommandResult.SUCCESS:
                self._complete()
            else:
                self._fail(f"{res}")
        return self._state

