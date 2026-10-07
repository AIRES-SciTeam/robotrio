from __future__ import annotations

from dataclasses import dataclass
from logging import Logger
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np
    from safety_commander.commander import Commander
    from grip_controller.grip_controller import GripController
    from mission.image_handlers import ImageReceiver, ImagePublisher
    from mission.tag_detector import TagDetector

@dataclass
class MissionEnv:
    commander : Commander
    logger : Logger
    camera_rotation : np.ndarray # optical camera frame -> FRD
    camera_position : tuple[float, float, float] # m, FRD
    image_receiver : ImageReceiver
    tag_detector : TagDetector | None
    image_publisher : ImagePublisher | None = None
    grip_ctrl : GripController | None = None
