#!/opt/python-venv/bin/python3
import argparse
import logging
from pathlib import Path
import sys
import time

# Allow running this file directly from any working directory.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

import rclpy
from rclpy.executors import ExternalShutdownException

from drone.flight.MissionController.ImageReciever import DRONE_ImageReciever
from drone.flight.Utils.Configs import DRONE_ConnConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/x500/down_cam/image")
    parser.add_argument("--camera-info-topic", default="/x500/down_cam/camera_info")
    args, ros_args = parser.parse_known_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(message)s")
    logger = logging.getLogger("image_receiver_check")
    config = DRONE_ConnConfig(
        control_conn="",
        image_topic=args.image_topic,
        camerainfo_topic=args.camera_info_topic,
    )

    rclpy.init(args=ros_args)
    receiver = None
    try:
        receiver = DRONE_ImageReciever(config, logger)
        logger.info("Receiving %s; FPS uses wall-clock time. Ctrl+C to stop.", args.image_topic)
        next_report = time.monotonic() + 1.0
        while rclpy.ok():
            rclpy.spin_once(receiver, timeout_sec=0.1)
            now = time.monotonic()
            if now < next_report:
                continue
            next_report = now + 1.0

            fps, recieving_lag = receiver.get_fps()
            if receiver.last_recieved_at is None:
                logger.info("Waiting for images on %s", args.image_topic)
                continue

            age = now - receiver.last_recieved_at
            _, frame = receiver.get_image()
            size = "none" if frame is None else f"{frame.shape[1]}x{frame.shape[0]}"
            lag_text = "n/a" if recieving_lag is None else f"{recieving_lag * 1000:.2f} ms"
            logger.info(
                "FPS: %.1f | frame: %s | last received: %.2f s ago | recieving_lag: %s%s",
                fps, size, age, lag_text,
                " | NO RECENT IMAGES" if age > 1.0 else "",
            )
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if receiver is not None:
            receiver.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
