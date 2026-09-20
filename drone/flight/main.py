#!/opt/python-venv/bin/python3
import logging
from pathlib import Path

from Utils.Configs import DRONE_ModelConfig, DRONE_ConnConfig, DRONE_TagConfig
from ManualController.ManualController_Keyboard import DRONE_KeyboardController
from ManualController.ManualController_Gamepad import DRONE_GamepadController
from GripController.GripController import DRONE_GripController
from Commander.MAVLinkCommander import DRONE_MAVLinkCommander

if __name__ == "__main__":
    logs_dir = Path(__file__).resolve().parents[2] / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(logs_dir / "drone_controller.log", encoding="utf-8"),
        ],
    )
    logger = logging.getLogger("drone.control")

    model_config = DRONE_ModelConfig(
        world = "scene",
        model = "x500",
        cargo = "goods"
    )
    conn_config = DRONE_ConnConfig(
        control_conn="udpin:127.0.0.1:14541",
        image_topic="",
        camerainfo_topic=""
    )
    tag_config = DRONE_TagConfig(
        family = "tag36h11",
        list = ["00", "01", "02", "03", "04", "05"]
    )

    commander = DRONE_MAVLinkCommander(
        conn_config = conn_config, 
        logger = logger
    )
    gripper = DRONE_GripController(
        model_config = model_config,
        tag_config = tag_config,
        logger = logger,
        grip_distance = 0.6
    )
    controller = DRONE_KeyboardController(
        com = commander,
        gripper = gripper,
        logger = logger
    )
    # controller = DRONE_GamepadController(
    #     com = commander,
    #     gripper = gripper,
    #     logger = logger,
    #     deadzone = 0.1
    # )

    controller.run()
