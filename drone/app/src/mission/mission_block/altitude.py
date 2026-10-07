from mission.mission_block.mission_block import MissionBlock
from safety_commander.commander import CommandResult


class ReachAltitude(MissionBlock):
    def __init__(
        self, 
        mission_env,
        altitude : float, # m
        eps : float = 0.15 # m
    ):
        super().__init__("ReachAltitude", mission_env)
        self.altitude = altitude
        self.eps = eps
        self.hold_xy = None

    def start(self):
        self.hold_xy = None
        super().start()

    def tick(self):
        if self._can_tick():
            x, y, z, _ = self.menv.commander.get_drone_position()
            if any(value is None for value in (x, y, z)):
                self._fail(f"Unknown drone position: {(x, y, z)}")
                return self._state
            if self.hold_xy is None:
                self.hold_xy = (x, y)
            if abs(z + self.altitude) <= self.eps:
                self._complete()
            else:
                res = self.menv.commander.set_offboard_target(pos=(*self.hold_xy, -self.altitude))
                if res != CommandResult.SUCCESS:
                    self._fail(f"{res.name}")
        return self._state

    

class Takeoff(ReachAltitude):
    def __init__(self, mission_env):
        super().__init__(
            mission_env=mission_env,
            altitude=1
        )
        self.name = "Takeoff"

