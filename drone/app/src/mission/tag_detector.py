import cv2
from pupil_apriltags import Detector, Detection
import numpy as np
from typing import List

from logger.logger_fabric import LoggerFabric


MODULE_NAME="mission"

class TagDetector:
    def __init__(
        self,
        tag_family : str,
        logger_fabric : LoggerFabric
    ):
        if not logger_fabric.is_alive():
            raise RuntimeError("LoggerFabric is stopped")
        self._logger = logger_fabric.get_logger(f"{__name__}", module_name=MODULE_NAME)

        self._detector = Detector(
            families=tag_family,
            nthreads=2,
            quad_decimate=1.0
        )
        self._logger.info("Initialized: tag_family=%s", tag_family)

    def detect(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections : List[Detection] = self._detector.detect(
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

def draw_tag_border(
    frame: np.ndarray,
    detection: Detection,
    color: tuple[int, int, int] = (0, 255, 0),
    alpha: float = 0.35,
) -> np.ndarray:
    """Fill the tag polygon in place; alpha=0 is transparent, alpha=1 opaque."""
    if not 0 <= alpha <= 1:
        raise ValueError("alpha must be between 0 and 1")
    corners = np.rint(detection.corners).astype(np.int32).reshape(4, 1, 2)
    overlay = frame.copy()
    cv2.fillConvexPoly(overlay, corners, color, lineType=cv2.LINE_AA)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, dst=frame)
    return frame
