import time

import numpy as np

from mission.mission_block.move_to import MoveTo
from safety_commander.commander import CommandResult


class MoveAboveTag(MoveTo):
    """Approach a tag in local NED, retaining the world target on visual loss.

    height is the vertical distance above the tag, not altitude above the origin.
    Camera extrinsics map optical coordinates to body FRD; telemetry quaternion
    is (w, x, y, z), rotating FRD into NED. Uses the latest telemetry pose.
    """

    def __init__(
        self,
        mission_env,
        tag_id: int,
        tag_size: float,
        height: float,
        eps: float = 0.1,
        speed_eps: float = 0.1,
        settle_time: float = 1.0,
        search_yaw_rate: float = 0.3,  # rad/s, positive clockwise in NED
        detection_timeout: float = 0.5,
    ):
        if not isinstance(tag_id, int) or isinstance(tag_id, bool) or tag_id < 0:
            raise ValueError("tag_id must be a nonnegative integer")
        for name, value in (("tag_size", tag_size), ("height", height),
                            ("search_yaw_rate", search_yaw_rate), ("detection_timeout", detection_timeout)):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        for name, value in (("eps", eps), ("speed_eps", speed_eps), ("settle_time", settle_time)):
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be nonnegative and finite")
        if mission_env.tag_detector is None or mission_env.image_receiver is None:
            raise ValueError("MoveAboveTag requires a camera receiver and tag detector")
        self._camera_rotation = np.array(mission_env.camera_rotation, dtype=float, copy=True)
        self._camera_position = np.array(mission_env.camera_position, dtype=float, copy=True)
        if (
            self._camera_rotation.shape != (3, 3)
            or not np.isfinite(self._camera_rotation).all()
            or not np.allclose(self._camera_rotation.T @ self._camera_rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(self._camera_rotation), 1, atol=1e-5)
            or self._camera_position.shape != (3,)
            or not np.isfinite(self._camera_position).all()
        ):
            raise ValueError("Invalid camera optical-to-FRD extrinsics")
        super().__init__(mission_env, pos=None, eps=eps, speed_eps=speed_eps, settle_time=settle_time)
        self.name = "MoveAboveTag"
        self.tag_id = tag_id
        self.tag_size = tag_size
        self.height = height
        self.search_yaw_rate = search_yaw_rate
        self.detection_timeout = detection_timeout
        self._last_frame = None
        self._target_acquired = False
        self._tag_visible = False
        self._last_seen_at = None
        self._searching = False

    def start(self):
        self.pos = None
        self._last_frame = None
        self._target_acquired = False
        self._tag_visible = False
        self._last_seen_at = None
        self._searching = False
        self.yaw = None
        super().start()

    @staticmethod
    def _body_to_ned(q):
        q = np.asarray(q, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
            raise ValueError("Invalid drone attitude quaternion")
        w, x, y, z = q / np.linalg.norm(q)
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
        ])

    def tick(self):
        if not self._can_tick():
            return self._state
        x, y, z, q = self.menv.commander.get_drone_position()
        if any(v is None for v in (x, y, z)) or not np.isfinite((x, y, z)).all():
            self._fail("Unknown drone position")
            return self._state
        if self.pos is None:
            self.pos = (x, y, z)

        frame = self.menv.image_receiver.get_image()
        camera_params, dist_coeffs = self.menv.image_receiver.get_camerainfo()
        now = time.monotonic()
        if frame is None or camera_params is None or q is None:
            self._tag_visible = False
        if frame is not None and frame is not self._last_frame and camera_params is not None and q is not None:
            body_to_ned = self._body_to_ned(q)
            self._last_frame = frame
            self._tag_visible = False
            estimate = self.menv.tag_detector.get_tag_position(
                frame, self.tag_id, camera_params, dist_coeffs, self.tag_size,
            )
            if estimate is not None:
                tag_in_camera = np.asarray(estimate[0], dtype=float)
                if tag_in_camera.shape == (3,) and np.isfinite(tag_in_camera).all() and tag_in_camera[2] > 0:
                    tag_in_body = self._camera_position + self._camera_rotation @ tag_in_camera
                    tag_in_ned = np.array((x, y, z)) + body_to_ned @ tag_in_body
                    self.pos = tuple(float(v) for v in tag_in_ned - (0, 0, self.height))
                    if not self._target_acquired:
                        self.menv.logger.info("MoveAboveTag: acquired tag %s, target=%s", self.tag_id, self.pos)
                    self._target_acquired = True
                    self._tag_visible = True
                    self._last_seen_at = now

        # Keep the last NED position target while searching; never complete
        # based on a stale detection, even if that position has been reached.
        if not self._tag_visible or self._last_seen_at is None or now - self._last_seen_at > self.detection_timeout:
            if not self._searching:
                self.menv.logger.info("MoveAboveTag: searching for tag %s by yaw rotation", self.tag_id)
            self._searching = True
            self._settled_since = None
            self._command_yaw = None
            self._last_tick_at = None
            self.yaw = None
            result = self.menv.commander.set_offboard_target(pos=self.pos, yaw_rate=self.search_yaw_rate)
            if result != CommandResult.SUCCESS:
                self._fail(result.name)
            return self._state
        if self._searching or self.yaw is None:
            # Hold the current heading explicitly to stop the search rotation.
            rotation = self._body_to_ned(q)
            self.yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
            if self._searching:
                self.menv.logger.info("MoveAboveTag: found tag %s, stopping search rotation", self.tag_id)
            self._searching = False
        return super().tick()
