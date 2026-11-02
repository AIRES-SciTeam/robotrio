"""Camera/AprilTag smoke test: python -m robotrio_dog.camera_stream."""

import argparse
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LOG = logging.getLogger("camera_stream")


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
                pass  # Browser disconnected or stopped consuming the stream.

        def log_message(self, fmt, *args):
            LOG.debug("HTTP %s: " + fmt, self.client_address[0], *args)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--family", default="tag25h9", help="One AprilTag family")
    parser.add_argument("--tag-id", type=int, help="Only highlight this ID; default: all")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=10)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or min(args.width, args.height) <= 0 or not 0 < args.fps <= 60:
        parser.error("Use port 1..65535, positive dimensions and fps in (0, 60]")
    if len(args.family.split()) != 1:
        parser.error("Specify exactly one --family")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")

    import cv2
    import numpy as np
    from picamera2 import Picamera2
    from pupil_apriltags import Detector

    detector = Detector(families=args.family, nthreads=2)
    frames = Frames()
    server = ThreadingHTTPServer((args.host, args.port), handler_for(frames))
    server.daemon_threads = True
    camera = None
    thread = None
    try:
        camera = Picamera2()
        # Picamera2 RGB888 arrays use BGR byte order, matching OpenCV.
        camera.configure(camera.create_video_configuration(
            main={"size": (args.width, args.height), "format": "RGB888"},
            controls={"FrameRate": args.fps},
        ))
        camera.start(show_preview=False)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        LOG.info("Listening on %s:%d; open http://<robot-IP>:%d/", args.host, args.port, args.port)
        LOG.info("Family=%s, ID=%s; Ctrl+C to stop", args.family, args.tag_id)
        previous_ids = None
        while True:
            started = time.monotonic()
            frame = camera.capture_array("main")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)
            ids = []
            for detection in detections:
                if args.tag_id is not None and detection.tag_id != args.tag_id:
                    continue
                ids.append(detection.tag_id)
                corners = np.rint(detection.corners).astype(np.int32)
                center = tuple(np.rint(detection.center).astype(int))
                cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
                cv2.circle(frame, center, 4, (0, 0, 255), -1)
                cv2.putText(frame, f"{args.family}:{detection.tag_id}",
                            (max(0, center[0] - 50), max(20, center[1] - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 2)
            if sorted(ids) != previous_ids:
                LOG.info("Visible tag IDs: %s", ids)
                previous_ids = sorted(ids)
            cv2.putText(frame, f"Tags: {len(ids)}  {time.strftime('%H:%M:%S')}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            frames.publish(jpeg.tobytes())
            time.sleep(max(0, 1 / args.fps - (time.monotonic() - started)))
    except KeyboardInterrupt:
        LOG.info("Stopping camera stream")
    finally:
        frames.close()
        if thread is not None:
            server.shutdown()
            thread.join()
        server.server_close()
        if camera is not None:
            camera.close()


if __name__ == "__main__":
    main()
