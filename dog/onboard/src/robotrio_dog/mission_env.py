from __future__ import annotations
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, Any

from .commander import Commander
from .logger_fabric import Logger
from .image_handlers import ImageReceiver, TagDetector
if TYPE_CHECKING:
    from .mission_executor import MissionExecutor

@dataclass
class MissionEnv:
    commander : Commander
    logger : Logger
    image_receiver : ImageReceiver
    tag_detectors : Dict[str, TagDetector]
    variables : Dict[str, Any] = field(default_factory=dict)
