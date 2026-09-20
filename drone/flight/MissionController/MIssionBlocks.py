from abc import ABC, abstractmethod
import numpy as np

from TagDetector import DRONE_TagDetector
from ImageReciever import DRONE_ImageReciever
from Commander.MAVLinkCommander import DRONE_MAVLinkCommander


class DRONE_MissionBlock(ABC):
    def __init__(
        self,
        default : DRONE_MissionBlock | None = None
    ):
        self.started = False
        self.ended = False

        self.default = default

    def start(self):
        self.started = True

    @abstractmethod
    def tick(self) -> bool:
        pass

    def end(self):
        self.ended = True

    def is_ended(self):
        return self.ended

class Arm(DRONE_MissionBlock):
    def __init__(
        self,
        commander : DRONE_MAVLinkCommander,
        max_tries : int = 3
    ):
        super().__init__()
        self.commander = commander
        self.max_tries = max_tries
        self.try_num = 0
    
    def tick(self):
        success = self.commander.arm()
        if not success:
            self.try_num += 1
            if self.try_num < self.max_tries:
                return self.tick()
            else:
                return False
        self.try_num = 0
        self.end()
        return True

class Disarm(DRONE_MissionBlock):
    def __init__(
        self,
        commander : DRONE_MAVLinkCommander,
        max_tries : int = 3
    ):
        super().__init__()
        self.commander = commander
        self.max_tries = max_tries
        self.try_num = 0
    
    def tick(self):
        success = self.commander.disarm()
        if not success:
            self.try_num += 1
            if self.try_num < self.max_tries:
                return self.tick()
            else:
                return False
        self.try_num = 0
        self.end()
        return True

class TakePositionByTag(DRONE_MissionBlock):
    def __init__(
        self, 
        tag_id : int,
        tag_size : float,
        offset : np.ndarray,
        orientation : np.ndarray,
        image_reciever : DRONE_ImageReciever,
        tag_detector : DRONE_TagDetector
    ):
        self.tag_id = tag_id
        self.tag_size = tag_size

        self.offset = offset
        self.orientation = orientation

        self.image_reciever = image_reciever
        self.tag_detector = tag_detector

    def tick(self):
        frame = self.image_reciever.get_image()
        camera_params, dist_coeffs = self.image_reciever.get_camerainfo()

        pos, rot = self.tag_detector.get_tag_position(
            frame=frame,
            tag_id=self.tag_id,
            camera_params=camera_params,
            dist_coeffs=dist_coeffs,
            tag_size=self.tag_size
        )


        
