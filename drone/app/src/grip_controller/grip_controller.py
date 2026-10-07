from gz.transport import Node
from math import sqrt
from dataclasses import dataclass

from logger.logger_fabric import LoggerFabric
from grip_controller.gripper import GZConfig, Gripper
from grip_controller.pose_tracker import PoseTracker


MODULE_NAME = "grip_controller"

@dataclass
class GripConfig:
    gripping_point : tuple[float, float, float]
    gripping_eps : tuple[float, float, float]

    def __post_init__(self):
        from math import isfinite
        if len(self.gripping_point) != 3 or len(self.gripping_eps) != 3:
            raise ValueError("Grip coordinates and tolerances must have three components")
        if not all(isfinite(v) for v in (*self.gripping_point, *self.gripping_eps)):
            raise ValueError("Grip coordinates and tolerances must be finite")
        if any(v < 0 for v in self.gripping_eps):
            raise ValueError("Grip tolerances must be nonnegative")

class GripController:
    def __init__(
        self, 
        gz_config: GZConfig,
        grip_config: GripConfig, 
        logger_fabric: LoggerFabric
    ):
        if not logger_fabric.is_alive():
            raise RuntimeError("LoggerFabric is stopped")
        self._logger = logger_fabric.get_logger(f"{type(self).__name__}", module_name=MODULE_NAME)
        self._logger.info("Initializing")
        self._node = Node()
        self.gz_config = gz_config
        self.grip_config = grip_config 
        
        self._gripper = Gripper(self._node, gz_config, logger_fabric)
        self._pose_tracker = PoseTracker(self._node, gz_config, logger_fabric)

        self._logger.info("Initialized")

    def _is_attachable(self, tag, snapshot=None) -> tuple[bool, float]:
        drone, cargos = self._pose_tracker.snapshot() if snapshot is None else snapshot
        cargo = cargos.get(tag)
        if drone is None or cargo is None:
            return False, float("inf")
        local = drone.inverse().compose(cargo).position
        error = tuple(c - p for c, p in zip(local, self.grip_config.gripping_point))
        return (
            all(abs(c) <= e for c, e in zip(error, self.grip_config.gripping_eps)),
            sqrt(sum(c * c for c in error)),
        )

    def toggle(self):
        if not self._gripper.is_busy():
            min_dist = float("inf")
            nearest_tag = None
            snapshot = self._pose_tracker.snapshot()
            for tag in self.gz_config.tags:
                attachable, dist = self._is_attachable(tag, snapshot)
                if attachable and dist < min_dist:
                    min_dist = dist
                    nearest_tag = tag
            if nearest_tag is None:
                self._logger.warning("No attachable tags")
                return False
            self._logger.info("Attach requested for tag %s", nearest_tag)
            return self._gripper.grip(nearest_tag)
        else:
            self._logger.info("Release requested")
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
