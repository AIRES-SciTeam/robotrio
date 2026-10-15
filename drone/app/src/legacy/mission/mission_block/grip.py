from mission.mission_block.mission_block import MissionBlock
from safety_commander.commander import CommandResult


class Grip(MissionBlock):
    def __init__(self, mission_env, tag: str):
        super().__init__("Grip", mission_env)
        self.tag = tag
        self._sent = False
        self._hold_pos = None

    def start(self):
        self._sent = False
        self._hold_pos = None
        super().start()

    def _request(self):
        return self.menv.grip_ctrl.grip(self.tag, confirm=False)

    def _confirmed(self):
        return self.menv.grip_ctrl.is_attached(self.tag)

    def tick(self):
        if not self._can_tick():
            return self._state
        if self.menv.grip_ctrl is None:
            self._fail("GripController unavailable")
            return self._state
        if self._hold_pos is None:
            x, y, z, _ = self.menv.commander.get_drone_position()
            if any(v is None for v in (x, y, z)):
                self._fail("Unknown drone position")
                return self._state
            self._hold_pos = (x, y, z)
        res = self.menv.commander.set_offboard_target(pos=self._hold_pos)
        if res != CommandResult.SUCCESS:
            self._fail(res.name)
            return self._state
        if not self._sent:
            if not self._request():
                self._fail("Gripper command rejected")
                return self._state
            self._sent = True
        if self._confirmed():
            self._complete()
        return self._state



class Release(Grip):
    def __init__(self, mission_env):
        super().__init__(mission_env, tag=None)
        self.name = "Release"

    def _request(self):
        return self.menv.grip_ctrl.release(confirm=False)

    def _confirmed(self):
        return not self.menv.grip_ctrl.is_busy()

