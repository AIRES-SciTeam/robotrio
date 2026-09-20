import cv2
from pupil_apriltags import Detector
import numpy as np
import logging


class DRONE_TagDetector:
    def __init__(
        self, 
        tag_family : str,
        logger : logging.Logger
    ):
        self.logger = logger
        self.detector = Detector(families = tag_family)

    def detect(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray_frame,
            estimate_tag_pose = False
        )
        return detections

    def estimate_pose(self, detection, camera_params, dist_coeffs, tag_size):
        _, corners = detection

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

        if not success or not np.isfinite(tvec).all():
            return None
        if tvec[2, 0] <= 0:
            return None

        position = tvec.reshape(3)
        rotation, _ = cv2.Rodrigues(rvec)

        return position, rotation

    def get_tag_position(self, frame, tag_id, camera_params, dist_coeffs, tag_size):
        detections = self.detect(frame)

        det_idx : int | None = None
        for idx, detection in enumerate(detections):
            if detection.tag_id == tag_id:
                det_idx = idx
                break
        else:
            return None

        pos, rot = self.estimate_pose(det_idx, camera_params, dist_coeffs, tag_size)

        return pos, rot
