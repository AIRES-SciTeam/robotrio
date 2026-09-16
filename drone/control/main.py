#!/opt/python-venv/bin/python3
import logging
from pathlib import Path
import rclpy # type: ignore
from rclpy.executors import ExternalShutdownException # type: ignore
from threading import Thread

from utils import DRONE_ModelConfig, DRONE_ConnConfig, DRONE_TagConfig
from Gamepad import DRONE_Gamepad
from Gripper import DRONE_Gripper
from MAVLinkConn import DRONE_MAVLinkConn
from TagDetector import DRONE_TagDetector


def spin_detector(node, logger):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except Exception:
        logger.exception("TagDetector: processing failed; detector stopped")


if __name__ == "__main__":
    logs_dir = Path(__file__).resolve().parents[2] / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(logs_dir / "drone_controller.log", encoding="utf-8"),
        ],
    )
    logger = logging.getLogger("drone.control")

    rclpy.init()

    tag_detector = None
    tag_detector_thread = None
    try:
        model_config = DRONE_ModelConfig(
            world = "scene",
            model = "x500",
            cargo = "goods"
        )
        conn_config = DRONE_ConnConfig(
            type = "udp",
            ip = "127.0.0.1",
            port = 18571
        )
        tag_config = DRONE_TagConfig(
            family = "tag36h11",
            list = ["00", "01", "02", "03", "04", "05"]
        )

        connection = DRONE_MAVLinkConn(
            conn_config = conn_config, 
            logger = logger
        )
        gripper = DRONE_Gripper(
            model_config = model_config,
            tag_config = tag_config,
            logger = logger,
            grip_distance = 0.6
        )
        commander = DRONE_Gamepad(
            conn = connection,
            gripper = gripper,
            logger = logger,
            deadzone = 0.1
        )
        tag_detector = DRONE_TagDetector(
            tag_config = tag_config,
            logger = logger
        )

        tag_detector_thread = Thread(
            target = spin_detector, args=(tag_detector, logger),
            name="tag-detector",
        )
        tag_detector_thread.start()

        commander.run()
    finally:
        rclpy.try_shutdown()
        if tag_detector_thread is not None and tag_detector_thread.ident is not None:
            tag_detector_thread.join()
        if tag_detector is not None:
            tag_detector.destroy_node()
