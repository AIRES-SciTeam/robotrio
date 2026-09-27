from gz.transport import Node
from dataclasses import dataclass
from math import sqrt
from dataclasses import dataclass


from Modules.Logger.LoggerFabric import LoggerFabric
from Modules.GripController.Gripper import Gripper, GZConfig
from Modules.GripController.PoseTracker import PoseTracker


@dataclass
class GripConfig:
    gripping_point : tuple[float, float, float]
    gripping_eps : tuple[float, float, float]

class GripController:
    def __init__(
        self, 
        gz_config: GZConfig,
        grip_config: GripConfig, 
        logger_fabric: LoggerFabric
    ):
        self._logger = logger_fabric.get_logger("GripController")
        self._node = Node()
        self.gz_config = gz_config
        self.grip_config = grip_config 
        
        self._gripper = Gripper(self._node, gz_config, logger_fabric)
        self._pose_tracker = PoseTracker(self._node, gz_config, logger_fabric)

        self._logger.info("Initialized")

    def _is_attachable(self, tag) -> tuple[bool, float]:
        drone_pose = self._pose_tracker.get_drone_pose().position
        cargo_pose = self._pose_tracker.get_cargo_pose(tag).position
        dist = sqrt(sum((d - c) * (d - c) for d, c in zip(drone_pose, cargo_pose, strict=True)))
        self._logger.debug("Attachability checking: drone=(%s, %s, %s), tag=(%s, %s, %s)", *drone_pose, *cargo_pose)
        if all(abs(c) <= e for c, e in zip(self.grip_config.gripping_point, self.grip_config.gripping_eps, strict=True)):
            self._logger.debug("Res = True, dist = %s", dist)
            return True, dist
        self._logger.debug("Res = False, dist = %s", dist)
        return False, dist

    def toggle(self):
        if not self._gripper.is_busy():
            min_dist = 1e+8
            nearest_tag = None
            for tag in self.gz_config.tags:
                attachable, dist = self._is_attachable(tag)
                if attachable and dist < min_dist:
                    min_dist = dist
                    nearest_tag = tag
            if nearest_tag is None:
                self._logger.warning("No attachable tags")
                return False
            self._logger.info("Attach requested for tag %s", nearest_tag)
            return self._gripper.grip(nearest_tag)
        else:
            self._logger.info("Release reauested")
            return self._gripper.release()

    def is_busy(self):
        return self._gripper.is_busy()

    def pose_snapshot(self):
        return self._pose_tracker.snapshot()

    def grip(self, tag, confirm=True):
        return self._gripper.grip(tag, confirm=confirm)

    def release(self, confirm=True):
        return self._gripper.release(confirm=confirm)

    def is_attached(self, tag):
        return self._gripper.is_attached(tag)
