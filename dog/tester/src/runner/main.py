import argparse, json, logging
from pathlib import Path

from .logger_fabric import LoggerFabric


# region Запуск приложения
def main():
    """
    Создаёт главный логгер и запускает терминал тестов.
    Параметр -c задаёт JSON-конфиг с разделами logger и runner.
    """
    parser = argparse.ArgumentParser(description="Robot dog test runner")
    parser.add_argument(
        "-c", "--config", type=Path,
        default=Path("/home/pi/robotrio_ws/config.json"),
        help="Path to a JSON configuration file (default: %(default)s)"
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
    main_logger = logging.getLogger("tester.startup")
    runner = None
    result = 0
    try:
        config = {}
        if args.config is not None:
            with args.config.open(encoding="utf-8") as file:
                config = json.load(file)
        if not isinstance(config, dict):
            main_logger.error("Configuration must be a JSON object")
            return 1
        logger_params = config.get("logger", {})
        runner_params = config.get("runner", {})
        if not isinstance(logger_params, dict) or not isinstance(runner_params, dict):
            main_logger.error("The logger and runner sections must be JSON objects")
            return 1
        camera_params = runner_params.get("camera_handler_params", {})
        if camera_params is None:
            camera_params = {}
        if not isinstance(camera_params, dict):
            main_logger.error("camera_handler_params must be a JSON object")
            return 1

        logger_fabric = LoggerFabric(**logger_params)
        main_logger = logger_fabric.get_logger()
        main_logger.info("Starting test runner")

        from .runner import TestRunner

        runner_params = dict(runner_params)
        runner_params["camera_handler_params"] = {
            "apriltags_families": ["tag36h11", "tag25h9"],
            "save_to": "./captures",
            **camera_params
        }
        runner = TestRunner(logger_fabric, **runner_params)
        if not runner.ready:
            main_logger.error("Test runner initialization failed")
            return 1
        runner.run_terminal(main_logger)
    except KeyboardInterrupt:
        main_logger.info("Test runner interrupted")
        result = 130
    except Exception:
        main_logger.exception("Test runner failed")
        result = 1
    finally:
        if runner is not None:
            try:
                runner.stop()
            except Exception:
                main_logger.exception("Failed to stop test runner")
                result = 1
        main_logger.info("Test runner stopped")
        logging.shutdown()
    return result
# endregion


if __name__ == "__main__":
    main()
