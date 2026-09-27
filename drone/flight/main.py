#!/opt/python-venv/bin/python3
from pathlib import Path
import time

from Modules.ManualController.ManualController_Gamepad import GamepadController
from Modules.GripController.GripController import GripController, GZConfig, GripConfig
from Modules.Logger.LoggerFabric import LoggerFabric
from Modules.SimpleCommander.Commander import Commander, MAVLinkConnConfig, OffboardConfig


gz_config = GZConfig(
    tags=["00", "01", "02", "03", "04", "05"]
)
grip_config = GripConfig(
    gripping_point=(0, 0, 0),
    gripping_eps=(0.1, 0.1, 0.3)
)
conn_config = MAVLinkConnConfig()
offboard_config = OffboardConfig()

logger_fabric = LoggerFabric(
    basic_level="info",
    logs_dir=Path(__file__).parents[2] / "logs"
)

main_logger = logger_fabric.get_logger("Main", use_stderr=True)

commander = None
grip_ctrl = None
js_ctrl = None
try:
    commander = Commander(
        logger_fabric=logger_fabric,
        conn_config=conn_config,
        offboard_config=offboard_config
    )
    main_logger.info("Commander inited")
    grip_ctrl = GripController(
        gz_config=gz_config,
        grip_config=grip_config,
        logger_fabric=logger_fabric
    )
    main_logger.info("Grip_ctrl inited")
    js_ctrl = GamepadController(
        commander=commander,
        grip_ctrl=grip_ctrl,
        logger_fabric=logger_fabric
    )
    main_logger.info("JS_ctrl inited")
    while True:
        time.sleep(1)
except Exception:
    pass
finally:
    if commander is not None:
       commander.stop()
    if js_ctrl is not None:
        js_ctrl.stop()
    