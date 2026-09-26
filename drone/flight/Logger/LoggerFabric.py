import logging
import json
import os
import platform
import socket
import subprocess
from datetime import datetime
from pathlib import Path
import uuid
import time


class LoggerFabric:
    def __init__(
        self,
        basic_level : str,
        logs_dir : Path,
        run_config : dict | None = None,
    ):
        self.level_dict = {
            "trace" : 5,
            "debug": logging.DEBUG,
            "info" : logging.INFO,
            "warning" : logging.WARNING,
            "error" : logging.ERROR
        }
        if not basic_level in self.level_dict:
            raise TypeError("Unknown logging level. Use: trace, debug, info, warning or error")
        
        self.basic_level = self.level_dict[basic_level]
        self._loggers = {}

        self.session_id = uuid.uuid4().hex

        self.logs_dir = Path(logs_dir) / self.session_id
        self.logs_dir.mkdir(parents=True, exist_ok=True)

        self._started_monotonic = time.monotonic()
        started_at = datetime.now().astimezone()
        try:
            git_commit = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=Path(__file__).resolve().parent,
                capture_output=True,
                text=True,
                check=True,
                timeout=2,
            ).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            git_commit = None
        self.meta = {
            "session_id": self.session_id,
            "started_at": started_at.isoformat(),
            "started_at_unix": started_at.timestamp(),
            "hostname": socket.gethostname(),
            "pid": os.getpid(),
            "python_version": platform.python_version(),
            "git_commit": git_commit,
            "logging_level": basic_level,
            "run_config": run_config if run_config is not None else {},
            "ended_at": None,
            "duration_seconds": None,
            "stop_reason": None,
        }
        self._write_meta()
        
        self.formatter = logging.Formatter(
            fmt="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        self._console_handler = logging.StreamHandler()
        self._console_handler.setFormatter(self.formatter)

        logging.addLevelName(5, "TRACE")

    def _write_meta(self) -> None:
        with (self.logs_dir / "meta.json").open("w", encoding="utf-8") as file:
            json.dump(self.meta, file, ensure_ascii=False, indent=2)
            file.write("\n")

    def finish_session(self, reason : str = "normal_exit"):
        if self.meta["ended_at"] is not None:
            return
        self.meta.update(
            ended_at=datetime.now().astimezone().isoformat(),
            duration_seconds=time.monotonic() - self._started_monotonic,
            stop_reason=reason,
        )
        self._write_meta()

    def get_logger(
        self, 
        name : str, 
        level : str | None = None, 
        use_stderr : bool = False
    ) -> logging.Logger:
        if name not in self._loggers:
            logger = logging.Logger(name, self.basic_level)
            logger.propagate = False
            file_handler = logging.FileHandler(
                self.logs_dir / f"drone.{name}.log", encoding="utf-8"
            )
            file_handler.setFormatter(self.formatter)
            logger.addHandler(file_handler)
            self._loggers[name] = logger
        logger = self._loggers[name]
        if level is not None:
            logger.setLevel(self.level_dict[level])
        if use_stderr:
            logger.addHandler(self._console_handler)
        return logger    
