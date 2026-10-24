from picamera2 import Picamera2 # type: ignore
from typing import Dict, Any, List
import threading
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection

from .logger_fabric import LoggerFabric, MAIN_LOGGER_NAME


# region ImageReceiver

class ImageReceiver:    
    def __init__(
        self,
        config : Dict[str, Any],
        logger_fabric : LoggerFabric
    ):
        self._logger = logger_fabric.get_logger(
            name=f"{MAIN_LOGGER_NAME}.{type(self).__name__}",
            level=config.get("log_level", "info")
        )
        self._logger.info("Initializing")

        try:
            self._camera_params = (config["fx"], config["fy"], config["cx"], config["cy"])
            self._dist_coeffs = np.array(config["dist_coeffs"], dtype=np.float64)
        except Exception as e:
            self._logger.critical("Initialization failed: %s", e)
            raise e

        self._frame_lock = threading.Lock()
        self._frame = None
        self._timestamp = None

        self._cam = None
        try:
            self._cam = Picamera2()
            self._cam.configure(
                self._cam.create_video_configuration(
                    main={"format": "RGB888"}
                )
            )
            self._cam.post_callback = self._image_callback
            self._cam.start(show_preview=False)
        except Exception as e:
            self._logger.critical("Initialization failed: %s", e)
            if self._cam is not None:
                self._cam.close()
            raise e

        self._logger.info("Initialized")

    def _image_callback(self, request):
        timestamp = request.get_metadata()["SensorTimestamp"]
        frame = request.make_array("main")
        with self._frame_lock:
            self._timestamp = timestamp
            self._frame = frame

    def close(self):
        if self._cam is not None:
            self._cam.stop()
            self._cam.close()
            self._cam = None

    def get_image(self):
        with self._frame_lock:
            return self._timestamp, self._frame

    def get_camerainfo(self):
        return self._camera_params, self._dist_coeffs

# endregion

# region TagDetector

class TagDetector:
    def __init__(
        self,
        config : Dict[str, Any],
        logger_fabric : LoggerFabric
    ):
        self._logger = logger_fabric.get_logger(
            name=f"{MAIN_LOGGER_NAME}.{type(self).__name__}",
            level=config.get("log_level", "info")
        )
        self._logger.info("Initializing")
        
        try:
            self._detector = Detector(
                families=config.get("tag_family", "tag36h11"),
                nthreads=2,
                quad_decimate=1.0
            )
        except Exception as e:
            self._logger.critical("Initialization failed: %s", e)
            raise e
        
        self._logger.info("Initialized")

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
    if not 0 <= alpha <= 1:
        raise ValueError("alpha must be between 0 and 1")
    corners = np.rint(detection.corners).astype(np.int32).reshape(4, 1, 2)
    overlay = frame.copy()
    cv2.fillConvexPoly(overlay, corners, color, lineType=cv2.LINE_AA)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, dst=frame)
    return frame
