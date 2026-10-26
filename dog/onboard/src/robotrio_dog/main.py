import json, argparse
import time

from .logger_fabric import LoggerFabric, MAIN_LOGGER_NAME
from .commander import Commander


def main():
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

    logger_fabric = LoggerFabric(logger_config)
    main_logger = logger_fabric.get_logger(name=MAIN_LOGGER_NAME)

    commander : Commander | None = None

    main_logger.info("Initializing system")
    success = True
    try:
        commander = Commander(
            config=commander_config,
            telemetry_config=telemetry_config,
            logger_fabric=logger_fabric
        )
        # commander.set_velocity(lin_vel=(5, None, None), approx_time_limit=4)
        a = 0
        while a != "stop":
            a = input()
            if a == "stop":
                break
            a = int(a)
            commander.control_arm(claw=a)
        # time.sleep(5)
        # commander.stop_motion()
    except Exception as e:
        main_logger.critical("System initialization failed: %s", e)
        success = False
    finally:
        if commander is not None:
            commander.close()
        main_logger.info("System stopped")

    if success:
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
