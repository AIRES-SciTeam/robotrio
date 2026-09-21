import logging
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Callable

from MissionBlocks import *
from Commander.MAVLinkCommander import DRONE_MAVLinkCommander
from Utils.Configs import DRONE_ConnConfig


class FailureAction(Enum):
    SKIP = auto()
    RETRY = auto()
    STOP = auto()

@dataclass
class DRONE_MissionStep:
    block : DRONE_MissionBlock
    on_failure : FailureAction = FailureAction.STOP
    max_retries : int = 0
    retry_delay : float = 0.5 # s
    timeout : float | None = None # s

class DRONE_MissionController:
    def __init__(
        self,
        plan : List[DRONE_MissionStep],
        commander : DRONE_MAVLinkCommander,
        logger : logging.Logger,
        conn_config : DRONE_ConnConfig,
        tag_family : str,
        camera_rotation : np.ndarray,
        camera_position : np.ndarray,
        tick_period : float, # s
        manual_interrupt_action : Callable[[DRONE_MissionEnv], DRONE_MissionBlock],
        end_action : Callable[[DRONE_MissionEnv], DRONE_MissionBlock],
        fail_action : Callable[[DRONE_MissionEnv], DRONE_MissionBlock]
    ):
        pass