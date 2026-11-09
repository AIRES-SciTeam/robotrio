import importlib
from picamera2 import Picamera2 # type: ignore
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import threading, time
import cv2
from pupil_apriltags import Detection, Detector
import numpy as np
import sys
from pathlib import Path


class Frames:
    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None
        self.sequence = 0
        self.closed = False

    def publish(self, jpeg):
        with self.condition:
            self.jpeg = jpeg
            self.sequence += 1
            self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


def handler_for(frames):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                body = (b'<!doctype html><title>Dog camera</title>'
                        b'<body style="background:#111;color:white;font-family:sans-serif">'
                        b'<h1>Dog camera / AprilTag</h1>'
                        b'<img src="/stream.mjpg" style="max-width:100%"></body>')
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return
            self.connection.settimeout(5)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            sequence = -1
            try:
                while True:
                    with frames.condition:
                        frames.condition.wait_for(
                            lambda: frames.closed or frames.sequence != sequence
                        )
                        if frames.closed:
                            return
                        sequence, jpeg = frames.sequence, frames.jpeg
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg + b"\r\n")
                    self.wfile.flush()
            except (OSError, TimeoutError):
                pass

    return Handler

class ImageReceiver:
    def __init__(self):
        self._dir = Path("/home/pi/robotrio_ws/val")
        self._dir.mkdir(exist_ok=True)

        self.body_pitch = 0
        self._timestamp = None
        self._frame = None
        self._lock = threading.Lock()
        self._detector_lock = threading.Lock()

        self._cam = Picamera2()
        self._cam.configure(
            self._cam.create_video_configuration(
                main={"format": "RGB888"}
            )
        )
        self._cam.post_callback = self._image_callback
        self._cam.start(show_preview=False)
        self._detector = Detector(families="tag36h11")
        self._pair_detector = Detector(families="tag25h9")
        self._camera_params = (794.46859366, 793.68863406, 631.52134703, 337.94348282)
        fx, fy, cx, cy = self._camera_params
        self._camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1],
        ], dtype=np.float64)
        self._tag_size = 0.010
        self._tag_sizes = {0: 0.027, 1: 0.010}
        self.start_stream()

    def _image_callback(self, request):
        self._timestamp = request.get_metadata()["SensorTimestamp"]
        frame = request.make_array("main")
        with self._lock:
            self._frame = frame

    def get_frame(self):
        with self._lock:
            return self._frame

    def get_poses(self, frame=None, *, family="tag36h11", tag_size=None):
        if family not in ("tag36h11", "tag25h9"):
            raise ValueError(f"Unsupported tag family: {family}")
        if family == "tag25h9" and tag_size is None:
            raise ValueError("tag25h9 requires its physical tag_size in metres")
        if tag_size is not None and (not np.isfinite(tag_size) or tag_size <= 0):
            raise ValueError("tag_size must be positive and finite")
        if frame is None:
            frame = self.get_frame()
        if frame is None:
            return []
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        with self._detector_lock:
            detector = self._detector if family == "tag36h11" else self._pair_detector
            detections = detector.detect(
                frame_gray, estimate_tag_pose=True,
                camera_params=self._camera_params,
                tag_size=self._tag_size if tag_size is None else tag_size,
            )
            if tag_size is not None:
                return detections
            # Translation scales linearly with the physical tag side length.
            for detection in detections:
                scale = self._tag_sizes.get(detection.tag_id, self._tag_size) / self._tag_size
                if detection.pose_t is not None:
                    detection.pose_t = detection.pose_t * scale
                if getattr(detection, "pose_err", None) is not None:
                    detection.pose_err *= scale ** 2
            return detections

    def start_stream(self, port=8080):
        self._stream_frames = Frames()
        self._stream_stop = threading.Event()
        self._server = ThreadingHTTPServer(("0.0.0.0", port), handler_for(self._stream_frames))
        self._server.daemon_threads = True
        self._http_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._video_thread = threading.Thread(target=self._publish_frames, daemon=True)
        self._http_thread.start()
        self._video_thread.start()
        print(f"Camera stream: http://<robot-IP>:{port}/", flush=True)

    def _publish_frames(self):
        try:
            while not self._stream_stop.is_set():
                frame = self.get_frame()
                if frame is not None:
                    # Run both detectors on the untouched image before drawing.
                    detections = [
                        (d, "tag36h11", self._tag_sizes.get(d.tag_id, self._tag_size))
                        for d in self.get_poses(frame)
                    ]
                    detections.extend(
                        (d, "tag25h9", 0.100)
                        for d in self.get_poses(frame, family="tag25h9", tag_size=0.100)
                    )
                    frame = frame.copy()
                    for detection, family, tag_size in detections:
                        corners = np.rint(detection.corners).astype(np.int32)
                        center = tuple(np.rint(detection.center).astype(int))
                        cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
                        cv2.circle(frame, center, 4, (0, 0, 255), -1)
                        cv2.putText(frame, f"{family}:{detection.tag_id}", center,
                                    cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
                        if detection.pose_R is not None and detection.pose_t is not None:
                            rvec, _ = cv2.Rodrigues(detection.pose_R)
                            cv2.drawFrameAxes(
                                frame, self._camera_matrix, None,
                                rvec, detection.pose_t,
                                tag_size / 2, 2,
                            )
                    ok, jpeg = cv2.imencode(".jpg", frame)
                    if ok:
                        self._stream_frames.publish(jpeg.tobytes())
                self._stream_stop.wait(0.1)
        except Exception as e:
            print(f"Camera stream failed: {e}", flush=True)
        finally:
            self._stream_frames.close()

    def close(self):
        if hasattr(self, "_stream_stop"):
            self._stream_stop.set()
            self._stream_frames.close()
            if hasattr(self, "_video_thread"):
                self._video_thread.join()
                self._server.shutdown()
                self._http_thread.join()
            if hasattr(self, "_server"):
                self._server.server_close()
        self._cam.close()

    def photo(self):
        with self._lock:
            cv2.imwrite(self._dir / f"{self._timestamp}.jpg", self._frame)

def run(img_recv : ImageReceiver):
    file_path = "/home/pi/robotrio_src/src/alg_test/inner.py"
    module_name = "inner"
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None:
        raise ImportError(f"Не удалось найти файл {file_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    res = module.run(img_recv)
    print(res)
    

def main():
    image_recv = ImageReceiver()
    try:
        print("Ready")
        while True:
            command = input("> ")
            if command == "stop":
                break
            elif command == "run":
                run(image_recv)
            elif command == "":
                image_recv.photo()
    except KeyboardInterrupt:
        pass
    finally:
        image_recv.close()
