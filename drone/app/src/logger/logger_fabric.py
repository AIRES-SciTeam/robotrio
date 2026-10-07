import logging, json
import os, platform, socket, subprocess
import uuid, time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


class LoggerFabric:
    def __init__(
        self,
        basic_level : str,
        logs_dir : Path,
        timezone : str = "Asia/Novosibirsk",
        use_unique_dir : bool = True
    ):
        self.timezone = timezone
        self.use_unique_dir = use_unique_dir
        self._level_dict = {
            "trace" : 5,
            "debug": logging.DEBUG,
            "info" : logging.INFO,
            "warning" : logging.WARNING,
            "error" : logging.ERROR
        }
        if not basic_level in self._level_dict:
            raise TypeError("Unknown logging level. Use: trace, debug, info, warning or error")
        self.basic_level = self._level_dict[basic_level]
        
        self._loggers = {}

        self._started_monotonic = time.monotonic()
        self._started_at = datetime.now().astimezone(ZoneInfo(timezone))
        self.logs_dir = Path(logs_dir)
        if use_unique_dir:
            self.logs_dir /= str(self._started_at)
        self.logs_dir.mkdir(parents=True, exist_ok=True)

        self._meta = self._compute_meta()
        if use_unique_dir:
            self._write_meta()
        
        self._formatter = logging.Formatter(
            fmt="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        self._console_handler = logging.StreamHandler()
        self._console_handler.setFormatter(self._formatter)

        self._stopped = False

        logging.addLevelName(5, "TRACE")

    def is_alive(self) -> bool:
        return not self._stopped

    def get_logs_dir(self) -> Path:
        return self.logs_dir

    def _compute_meta(self) -> dict:
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
        self._meta = {
            "session_id": uuid.uuid4().hex,
            "started_at": self._started_at.isoformat(),
            "started_at_unix": self._started_at.timestamp(),
            "hostname": socket.gethostname(),
            "pid": os.getpid(),
            "python_version": platform.python_version(),
            "git_commit": git_commit,
            "logging_level": self.basic_level,
            "ended_at": None,
            "duration_seconds": None,
            "stop_reason": None,
        }
        return self._meta

    def _write_meta(self) -> None:
        with (self.logs_dir / "meta.json").open("w" if self.use_unique_dir else "a", encoding="utf-8") as file:
            json.dump(self._meta, file, ensure_ascii=False, indent=2)
            file.write("\n")

    def finish_session(self, reason : str = "normal_exit") -> None:
        if self._meta["ended_at"] is not None:
            return
        self._meta.update(
            ended_at=datetime.now().astimezone(ZoneInfo(self.timezone)).isoformat(),
            duration_seconds=time.monotonic() - self._started_monotonic,
            stop_reason=reason,
        )
        self._write_meta()
        self._stopped = True

    def get_logger(
        self, 
        name : str, 
        level : str | None = None,
        module_name : str = "main",
        use_common : bool = True, 
        use_stderr : bool = False,
        no_format : bool = False
    ) -> logging.Logger:
        dir = self.logs_dir if module_name == "main" else self.logs_dir / module_name
        dir.mkdir(parents=True, exist_ok=True)
        if name not in self._loggers:
            logger = logging.Logger(name, self.basic_level)
            logger.propagate = False
            file_handler = logging.FileHandler(
                dir / f"{name}.log", encoding="utf-8"
            )
            if not no_format:
                file_handler.setFormatter(self._formatter)
            logger.addHandler(file_handler)
            self._loggers[name] = logger
        logger = self._loggers[name]
        if use_common:
            module_handler = logging.FileHandler(
                dir / f"common.log", encoding="utf-8"
            )
            if not no_format:
                module_handler.setFormatter(self._formatter)
            logger.addHandler(module_handler)
        if level is not None:
            logger.setLevel(self._level_dict[level])
        if use_stderr:
            logger.addHandler(self._console_handler)
        return logger   

    
