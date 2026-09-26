#!/opt/python-venv/bin/python3
import logging
from pathlib import Path

import numpy as np

from Commander.MAVLinkCommander import DRONE_MAVLinkCommander
from GripController.GripController import DRONE_GripController
from ManualController.ManualController_Gamepad import DRONE_GamepadController
from MissionController.MissionBlocks import Arm, Disarm, Land, Manual, Takeoff
from MissionController.MissionController import (
    DRONE_MissionController,
    DRONE_MissionStep,
    FailureAction,
)
from Utils.Configs import DRONE_ConnConfig, DRONE_ModelConfig, DRONE_TagConfig


def configure_logger() -> logging.Logger:
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
    return logging.getLogger("drone.control")


def create_plan() -> list[DRONE_MissionStep]:
    return [
        DRONE_MissionStep(
            block_class=Arm,
            on_failure=FailureAction.FAIL,
        ),
        DRONE_MissionStep(
            block_class=Takeoff,
            on_failure=FailureAction.FAIL,
        ),
        DRONE_MissionStep(
            block_class=Land,
            block_params={"timeout": 60.0},
            on_failure=FailureAction.FAIL,
        ),
        DRONE_MissionStep(
            block_class=Disarm,
            on_failure=FailureAction.FAIL,
        ),
    ]


def main() -> None:
    logger = configure_logger()

    model_config = DRONE_ModelConfig(
        world="scene",
        model="x500",
        cargo="goods",
    )
    conn_config = DRONE_ConnConfig(
        control_conn="udpin:127.0.0.1:14541",
        image_topic="/x500/down_cam/image",
        camerainfo_topic="/x500/down_cam/camera_info",
    )
    tag_config = DRONE_TagConfig(
        family="tag36h11",
        list=["00", "01", "02", "03", "04", "05"],
    )

    commander = None
    mission_controller = None

    try:
        commander = DRONE_MAVLinkCommander(
            conn_address=conn_config.control_conn,
            logger=logger,
        )
        gripper = DRONE_GripController(
            model_config=model_config,
            tag_config=tag_config,
            logger=logger,
            grip_distance=0.6,
        )
        mission_controller = DRONE_MissionController(
            plan=create_plan(),
            commander=commander,
            logger=logger,
            conn_config=conn_config,
            tag_family=tag_config.family,
            camera_rotation=np.eye(3),
            camera_position=np.zeros(3),
            tick_period=0.05,
            pause_step=DRONE_MissionStep(block_class=Manual),
            end_step=DRONE_MissionStep(block_class=Manual),
            fail_step=DRONE_MissionStep(
                block_class=Land,
                block_params={"timeout": 60.0},
            ),
        )
        controller = DRONE_GamepadController(
            com=commander,
            gripper=gripper,
            mission_controller=mission_controller,
            logger=logger,
            rate=50,
            deadzone=0.1,
        )

        mission_controller.start()
        logger.info("DRONE_MissionController: Ready; press Y to start the mission")
        controller.run()

    except KeyboardInterrupt:
        logger.info("DRONE_Main: Interrupted")
    finally:
        if mission_controller is not None:
            mission_controller.stop()
        if commander is not None:
            commander.stop()


if __name__ == "__main__":
    main()
