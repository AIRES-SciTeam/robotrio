import logging
from pathlib import Path
from datetime import datetime


class Logger(logging.Logger):
    """
    Кастомизация классического logging.Logger.
    Добавляет метод trace, позволяющий логгировать данные
    на уровне ниже дебаг.
    """
    def trace(self, msg, *args, **kwargs):
        self.log(5, msg, *args, **kwargs)

class Handler(logging.Handler):
    """
    Кастомизация классического logging.Handler
    """
    def __init__(
        self,
        target : Logger
    ):
        super().__init__()
        self.target = target

    def emit(self, record):
        if self.target.isEnabledFor(record.levelno):
            self.target.handle(record)

class LoggerFabric:
    """
    Модуль, создающий и кэширующий логгеры Logger.
    
    Параметры:
    - basic_level: Минимальный уровень сообщений логгеров по умолчанию.
        Допустимые значения: "trace", "debug", "info", "warn",
        "warning", "error", "critical".
    - logs_dir: Каталог логов.
    - use_time_dir: Флаг, показывающий, сохранять ли логи напрямую в logs_dir,
        или использовать подкаталог, названный по текущему дате-времени.
    - format: Формат записей в синтаксисе logging.Formatter.
    - date_format: Формат времени для поля asctime.
    - main_logger_name: Имя главного логгера.
    - main_logger_level: Минимальный уровень консольного обработчика.
    - clean_slate : False - логи нового запуска будут добавляться к старым, 
        True - логи нового запуска будут перезатирать старые.

    Пояснения:
    Каждый логгер создаётся методом или выбирается из кэшированных методом get_logger.
    main_logger - единственный логгер, имеющий доступ к консоли. Может быть
    родительским для остальных.
    """
    def __init__(
        self, 
        basic_level : str = "info",
        logs_dir : str = "./logs",
        use_time_dir : bool = False,
        *,
        format : str = "%(asctime)s | %(levelname)-7s | %(name)s | %(message)s",
        date_format : str = "%d-%m-%Y %H:%M:%S",
        main_logger_name : str = "onboard",
        main_logger_console_level : str = "info",
        clean_slate : bool = False
    ):
        logging.addLevelName(5, "TRACE")
        logging.setLoggerClass(Logger)

        self._level_dict = {
            "trace": 5, 
            "debug": logging.DEBUG,
            "info": logging.INFO, 
            "warn": logging.WARNING, 
            "warning": logging.WARNING,
            "error": logging.ERROR,
            "critical": logging.CRITICAL
        }
        self._basic_level = self._level_dict[basic_level.lower()]
        self.main_logger_console_level = self._level_dict[main_logger_console_level.lower()]
        self._logs_dir = Path(logs_dir)
        if use_time_dir:
            self._logs_dir /= datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self._logs_dir.mkdir(parents=True, exist_ok=True)

        self.main_logger_name = main_logger_name
        self._formatter = logging.Formatter(fmt=format, datefmt=date_format)

        self._loggers = {}

    def get_logger(
        self,
        name : str |  None = None, 
        level : str | None = None,
        *,
        no_format : bool = False,
        propagate_to_main : bool = True
    ) -> Logger:
        """
        Возвращает логгер из кеша или настраивает его при первом обращении.

        Файл <последняя часть имени (разделитель ".")>.log открывается в режиме добавления,
        если self.clean_slate == False, иначе в режиме записи.
        Например, для имени "onboard.camera" используется файл camera.log.
        Имена с одинаковой последней частью используют один файл.
        Повторный вызов для того же итогового имени возвращает прежний
        объект, не меняя его уровень, формат или обработчики.

        Параметры:
        - name: Имя логгера. None выбирает главный логгер фабрики.
        - level: Минимальный уровень сообщений. None использует basic_level.
        - no_format: Не назначать форматтер файловому обработчику.
            В файл записывается текст сообщения и, при наличии,
            сведения об исключении. Формат консоли не меняется.
        - propagate_to_main: Включает распространение логгеров в main_logger.
            При name=None параметр не используется. Передача сообщений
            определяется иерархией имён и свойством logger.propagate;
            главный логгер автоматически не создаётся.
        """
        if name is None:
            name = self.main_logger_name
        else:
            name = f"{self.main_logger_name}." + name if propagate_to_main else name
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
            if name == self.main_logger_name:
                console_handler = logging.StreamHandler()
                console_handler.setLevel(self.main_logger_console_level)
                console_handler.setFormatter(self._formatter)
                logger.addHandler(console_handler)
                logger.propagate = False
            self._loggers[name] = logger
        return self._loggers[name]

    @staticmethod
    def redirect_logging(
        source_name : str,
        target : Logger
    ):
        """
        Перенаправляет сообщения постороннего логгера c именем source_name в target.
        Необходимо вызывать до создания объекта, создающего посторонний логгер.
        """
        source = logging.getLogger(source_name)
        source.handlers.clear()
        source.addHandler(Handler(target))
        source.setLevel(target.getEffectiveLevel())
        source.propagate = False
