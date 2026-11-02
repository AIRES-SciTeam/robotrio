from xgolib import XGO_DOG
from pupil_apriltags import Detection, Detector
from picamera2 import Picamera2 # type: ignore
import threading, time
import cv2
import numpy as np
import math
from http.server import ThreadingHTTPServer
from .camera_stream import Frames, handler_for


class ImageReceiver:
    def __init__(self):
        self.body_pitch = 0
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
        self._camera_params = (794.46859366, 793.68863406, 631.52134703, 337.94348282)

    def _image_callback(self, request):
        frame = request.make_array("main")
        with self._lock:
            self._frame = frame

    def get_frame(self):
        with self._lock:
            return self._frame

    def get_poses(self, frame=None):
        if frame is None:
            frame = self.get_frame()
        if frame is None:
            return []
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        with self._detector_lock:
            return self._detector.detect(
                frame_gray, estimate_tag_pose=True,
                camera_params=self._camera_params, tag_size=0.027
            )

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
                    frame = frame.copy()
                    for detection in self.get_poses(frame):
                        corners = np.rint(detection.corners).astype(np.int32)
                        center = tuple(np.rint(detection.center).astype(int))
                        cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
                        cv2.circle(frame, center, 4, (0, 0, 255), -1)
                        cv2.putText(frame, str(detection.tag_id), center,
                                    cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
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

TARGET_DISTANCE = 0.18
LOOK_DOWN_PITCH = 10
FORWARD_SPEED = 6
STEP_SECONDS = 2
TURN_SPEED = 6
CENTER_TOLERANCE = 0.10
LOW_TAG_POSITION = 0.70 

def observe_tag(img_rec, upper_only=False):
    previous = img_rec.get_frame()
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        frame = img_rec.get_frame()
        if frame is not None and frame is not previous:
            tags = [d for d in img_rec.get_poses(frame) if d.tag_id == 0]
            if upper_only:
                pitch = math.radians(img_rec.body_pitch)
                vertical = np.array([0, math.cos(pitch), math.sin(pitch)])
                tags = [d for d in tags if d.pose_R is not None
                        and abs(float(np.dot(d.pose_R[:, 2], vertical))) >= math.cos(math.radians(20))]
            if not tags:
                return None
            tag = max(tags, key=lambda d: d.center[1])
            return tag, frame.shape[1], frame.shape[0]
        time.sleep(0.02)
    raise TimeoutError("No fresh camera frame")


def find_tag_hor(dog: XGO_DOG, img_rec, sign=1):
    """Center the tag without changing the current body pitch."""
    turning = None
    deadline = time.monotonic() + 30
    try:
        while time.monotonic() < deadline:
            observation = observe_tag(img_rec)
            if observation is None:
                velocity = math.copysign(TURN_SPEED, sign)
            else:
                tag, width, _ = observation
                error = float(tag.center[0]) - width / 2
                if abs(error) <= width * CENTER_TOLERANCE:
                    return True
                # Positive SDK turn rotates left; image X increases right.
                velocity = -math.copysign(TURN_SPEED, error)
                sign = velocity
            if velocity != turning:
                dog.turn(velocity)
                turning = velocity
                print(f"Aligning in current pitch: turn={velocity}", flush=True)
        return False
    finally:
        dog.turn(0)


def reach_tag_x(dog: XGO_DOG, img_rec):
    pitch = 0
    missing = 0
    dog.stop()
    dog.attitude("p", pitch)
    img_rec.body_pitch = pitch
    try:
        while True:
            observation = observe_tag(img_rec)
            if observation is None:
                missing += 1
                print(f"Target missing while stopped: {missing}/5", flush=True)
                if missing >= 5:
                    return False
                time.sleep(0.1)
                continue
            missing = 0
            tag, width, height = observation
            distance = float(np.linalg.norm(tag.pose_t))
            if not math.isfinite(distance):
                raise ValueError("Invalid tag distance")
            error_x = float(tag.center[0]) - width / 2
            position_y = float(tag.center[1]) / height
            print(f"Target: distance={distance:.3f} m, y={position_y:.2f}, pitch={pitch}", flush=True)

            if distance > TARGET_DISTANCE and position_y >= LOW_TAG_POSITION and pitch == 0:
                pitch = LOOK_DOWN_PITCH
                dog.attitude("p", pitch)
                img_rec.body_pitch = pitch
                print(f"Looking down: pitch={pitch}", flush=True)
                time.sleep(0.3)
                continue  # Re-measure after tilting, before deciding where to move.

            if abs(error_x) > width * CENTER_TOLERANCE:
                if not find_tag_hor(dog, img_rec, -error_x):
                    return False
                continue  # Re-measure distance after alignment; preserve pitch.

            if distance <= TARGET_DISTANCE:
                print("Target reached", flush=True)
                return True

            if pitch != 0:
                dog.attitude("p", 0)
                pitch = 0
                img_rec.body_pitch = pitch
                time.sleep(0.3)
            print(f"Forward step: speed={FORWARD_SPEED}, duration={STEP_SECONDS}s", flush=True)
            dog.move_x(FORWARD_SPEED, STEP_SECONDS)
            dog.stop()
            # Next iteration observes a new frame while standing still.
    finally:
        dog.stop()


def center_upper_tag(dog, img_rec):
    """Center using body translation only; return the measured camera pose.

    Assumes the level camera mounting used by the approach code. Translation
    starts from main's x=0, y=0, z=60 pose. No walking or rotation is commanded.
    """
    dog.stop()
    body_y, body_z = 0.0, 60.0
    for _ in range(30):
        observation = observe_tag(img_rec, upper_only=True)
        if observation is None:
            raise RuntimeError("Upper tag not visible; cannot align for grasp")
        tag, width, height = observation
        error_x = (float(tag.center[0]) - width / 2) / width
        error_y = (float(tag.center[1]) - height / 2) / height
        position_mm = np.asarray(tag.pose_t).reshape(3) * 1000
        if not np.isfinite(position_mm).all():
            raise ValueError("Invalid upper tag pose")
        print(f"Upper tag: camera XYZ={position_mm.tolist()} mm, image error=({error_x:.3f}, {error_y:.3f})", flush=True)
        if abs(error_x) <= .05 and abs(error_y) <= .05:
            return position_mm, tag.pose_R
        if abs(error_x) > .05:
            new_y = max(-18, min(18, body_y - math.copysign(3, error_x)))
            if new_y == body_y:
                raise RuntimeError("Body Y limit reached before centering tag")
            dog.translation("y", new_y)
            body_y = new_y
        elif abs(error_y) > .05:
            new_z = max(60, min(110, body_z - math.copysign(3, error_y)))
            if new_z == body_z:
                raise RuntimeError("Body height limit reached before centering tag")
            dog.translation("z", new_z)
            body_z = new_z
        time.sleep(.3)
    raise TimeoutError("Body translation did not center upper tag")


def main():
    dog = XGO_DOG(port="/dev/ttyAMA0", baud=115200)
    img_rec = None
    try:
        img_rec = ImageReceiver()
        img_rec.start_stream()

        dog.reset()
        dog.pace("slow")
        dog.translation("z", 60)

        time.sleep(10)
        print("start")
        if find_tag_hor(dog, img_rec, -1):
            if reach_tag_x(dog, img_rec):
                position_mm, rotation = center_upper_tag(dog, img_rec)
                print(f"Ready for camera-to-arm conversion: {position_mm.tolist()} mm", flush=True)
                print("Grasp not commanded: camera-to-arm calibration is required", flush=True)
        else:
            print("Target alignment timed out", flush=True)

    except KeyboardInterrupt:
        return
    except Exception as e:
        raise e
    finally:
        try:
            dog.stop()
        finally:
            if img_rec is not None:
                img_rec.close()

        
