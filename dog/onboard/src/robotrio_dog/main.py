import json, argparse
import time
from typing import List

import inspect
from xgolib import XGO_DOG

from .logger_fabric import LoggerFabric, MAIN_LOGGER_NAME
from .commander import Commander
from .image_handlers import ImageReceiver, TagDetector
from .mission_executor import MissionExecutor
from .mission_units import *


def conver2u8_my(data, limit, min_value=0):
    if data is None or limit is None:
        return min_value
    low, high = limit if isinstance(limit, list) else (-limit, limit)
    if data >= high:
        return 255
    if data <= low:
        return min_value
    return round(255 * (data - low) / (high - low))

def main():
    sdk = inspect.getmodule(XGO_DOG)
    orig_convert = sdk.conver2u8
    sdk.conver2u8 = conver2u8_my

    parser = argparse.ArgumentParser()
    parser.add_argument("config", metavar="conf", nargs="?")
    parser.add_argument("-c", "--conf", metavar="conf")
    args = parser.parse_args()

    config_file = args.config or args.conf

    if config_file is None:
        configs = {}
    else:
        with open(config_file, "r") as file:
            configs = json.load(file)

    logger_config = configs.get("logger", {})
    telemetry_config = configs.get("telemetry", {})
    commander_config = configs.get("commander", {})
    mission_executor_config = configs.get("mission_executor", {})
    image_receiver_config = configs.get("image_receiver", {})
    tag_detectors_config = configs.get("tag_detectors", {})

    logger_fabric = LoggerFabric(logger_config)
    main_logger = logger_fabric.get_logger(name=MAIN_LOGGER_NAME)

    commander : Commander | None = None
    image_receiver : ImageReceiver | None = None
    tag_detectors : List[TagDetector] | None = None
    mission_executor : MissionExecutor | None = None

    main_logger.info("Initializing system")
    success = True
    try:
        commander = Commander(
            config=commander_config,
            telemetry_config=telemetry_config,
            logger_fabric=logger_fabric
        )
        image_receiver = ImageReceiver(
            config=image_receiver_config,
            logger_fabric=logger_fabric
        )
        time.sleep(1)
        tag_log_level = tag_detectors_config.get("log_level", "info")
        tag_detectors = {
            family: TagDetector(
                logger_fabric=logger_fabric,
                log_level=tag_log_level,
                tag_family=family
            )
            for family in tag_detectors_config.get("families", ["tag36h11"])
        }
        mission_executor = MissionExecutor(
            config=mission_executor_config,
            logger_fabric=logger_fabric,
            commander=commander,
            image_receiver=image_receiver,
            tag_detectors=tag_detectors 
        )

        mission_executor.configure([
            SetPace("slow"),
            FindTag("tag25h9:0", 30)
        ])

        mission_executor.execute()

    except Exception as e:
        main_logger.critical("System initialization failed: %s", e)
        success = False
    finally:
        if commander is not None:
            commander.close()
        if mission_executor is not None:
            mission_executor.stop()
        main_logger.info("System stopped")

    sdk.conver2u8 = orig_convert

    if success:
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
