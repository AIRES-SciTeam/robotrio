import time

from logger.logger_fabric import LoggerFabric
from mission.mission_env import MissionEnv
from mission.tag_detector import draw_tag_border
from mission.mission_block.mission_block import MissionBlock


class CameraTest(MissionBlock):
    def __init__(
        self,
        mission_env: MissionEnv,
        *,
        draw_tags: bool = True,
    ):
        super().__init__(name="CameraTest", mission_env=mission_env)
        self.draw_tags = draw_tags
        self._last_frame = None

    def tick(self):
        if self._can_tick():
            start = time.monotonic()
            img, header = self.menv.image_receiver.get_image(with_header=True)
            if img is None or img is self._last_frame:
                return self._state
            received_at = time.monotonic()
            detections = self.menv.tag_detector.detect(img) if self.draw_tags else []
            detected_at = time.monotonic()
            annotated = img.copy() if self.draw_tags else img
            for det in detections:
                draw_tag_border(annotated, det)
            drawned_at = time.monotonic()
            self.menv.image_publisher.pub(annotated, header)
            self._last_frame = img
            published_at = time.monotonic()
            self.menv.logger.info(
                "Receiving_rate: %.2f, detection_rate: %.2f, pubilishing_rate: %.2f", 
                1/r if (r := (received_at - start)) != 0 else -1,
                1/r if (r := (detected_at - received_at)) != 0 else -1,
                1/r if (r := (published_at - drawned_at)) != 0 else -1
            )
        return self._state
