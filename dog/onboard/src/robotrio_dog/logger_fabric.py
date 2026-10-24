import logging, json
from pathlib import Path
from datetime import datetime
from typing import Dict, Any


MAIN_LOGGER_NAME = "onboard"

class Logger(logging.Logger):
    def trace(self, msg, *args, **kwargs):
        self.log(5, msg, *args, **kwargs)

class LoggerFabric:
    def __init__(
        self, 
        config : Dict[str, Any]
    ):
        self._level_dict = {
            "trace": 5, 
            "debug": logging.DEBUG,
            "info": logging.INFO, 
            "warn": logging.WARNING, 
            "warning": logging.WARNING,
            "error": logging.ERROR,
            "critical": logging.CRITICAL
        }
        self._basic_level = self._level_dict[config.get("basic_level", "info")]
        self._logs_dir = Path(config.get("logs_dir", Path.home() / ".local/state/robotrio-dog/logs")).expanduser()
        use_time_dir = config.get("use_time_dir", False)
        if use_time_dir:
            self._logs_dir /= datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self._format = config.get("format", "%(asctime)s | %(levelname)-7s | %(name)s | %(message)s")
        self._date_format = config.get("date_format", "%d-%m-%Y %H:%M:%S")
        self._logs_dir.mkdir(parents=True, exist_ok=True)

        self._loggers = {}

        self._formatter = logging.Formatter(fmt=self._format, datefmt=self._date_format)

        logging.addLevelName(5, "TRACE")
        logging.setLoggerClass(Logger)

    def get_logger(
        self, *,
        name : str, 
        level : str | None = None,
        no_format : bool = False
    ) -> Logger:
        if name not in self._loggers:
            logger = logging.getLogger(name)
            logger.setLevel(
                self._basic_level if level is None else self._level_dict[level]
            )
            file_handler = logging.FileHandler(
                self._logs_dir / f"{name.split('.')[-1]}.log", 
                encoding="utf-8"
            )   
            if not no_format:
                file_handler.setFormatter(self._formatter)
            logger.addHandler(file_handler)
            if name == MAIN_LOGGER_NAME:
                console_handler = logging.StreamHandler()
                console_handler.setLevel(logging.WARNING)
                console_handler.setFormatter(self._formatter)
                logger.addHandler(console_handler)
                logger.propagate = False
            self._loggers[name] = logger
        return self._loggers[name]
