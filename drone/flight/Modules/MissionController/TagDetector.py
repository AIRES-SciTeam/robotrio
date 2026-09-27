import cv2
from pupil_apriltags import Detector
import numpy as np

from Modules.Logger.LoggerFabric import LoggerFabric


class TagDetector:
    def __init__(
        self, 
        tag_family : str,
        logger_fabric : LoggerFabric
    ):
        self._logger = logger_fabric.get_logger("TagDetector")
        
        self._detector = Detector(families = tag_family)

        self._logger.info("Initialized: tag_family=%s", tag_family)

    def detect(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self._detector.detect(
            gray_frame,
            estimate_tag_pose = False
        )
        self._logger.log(
            5,
            "Detected %d tags: %s",
            len(detections),
            [
                {"id": detection.tag_id, "corners": detection.corners.tolist()}
                for detection in detections
            ],
        )
        return detections

    def estimate_pose(self, detection, camera_params, dist_coeffs, tag_size):
        self._logger.log(
            5,
            "Estimating pose: id=%s, tag_size=%s, camera_params=%s, dist_coeffs=%s",
            detection.tag_id, tag_size, camera_params, dist_coeffs,
        )
        corners = detection.corners

        fx, fy, cx, cy = camera_params
        camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float64)

        half = tag_size / 2
        object_points = np.array([
            [-half, half, 0],
            [half, half, 0],
            [half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float64)

        image_points = np.ascontiguousarray(corners, dtype=np.float64).reshape(4, 2)

        success, rvec, tvec = cv2.solvePnP(
            objectPoints=object_points,
            imagePoints=image_points,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success:
            self._logger.debug("Pose rejected: id=%s, solvePnP failed", detection.tag_id)
            return None
        if not np.isfinite(tvec).all() or not np.isfinite(rvec).all():
            self._logger.debug(
                "Pose rejected: id=%s, non-finite tvec=%s, rvec=%s",
                detection.tag_id, tvec, rvec,
            )
            return None
        if tvec[2, 0] <= 0:
            self._logger.debug(
                "Pose rejected: id=%s, non-positive depth=%s",
                detection.tag_id, tvec[2, 0],
            )
            return None

        position = tvec.reshape(3)
        rotation, _ = cv2.Rodrigues(rvec)

        self._logger.log(
            5,
            "Pose estimated: id=%s, camera_position=%s, rotation=%s",
            detection.tag_id, position.tolist(), rotation.tolist(),
        )
        return position, rotation

    def get_tag_position(self, frame, tag_id, camera_params, dist_coeffs, tag_size):
        self._logger.log(5, "Searching for tag: id=%s", tag_id)
        detections = self.detect(frame)

        det_idx : int | None = None
        for idx, detection in enumerate(detections):
            if detection.tag_id == tag_id:
                det_idx = idx
                break
        else:
            self._logger.log(5, "Tag not found: id=%s", tag_id)
            return None

        self._logger.log(5, "Tag found: id=%s, estimating pose", tag_id)
        return self.estimate_pose(detections[det_idx], camera_params, dist_coeffs, tag_size)
