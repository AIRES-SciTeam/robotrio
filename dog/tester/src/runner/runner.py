from typing import Dict, Any
from pathlib import Path
import shlex, importlib.util, sys, uuid
from prompt_toolkit import PromptSession
from prompt_toolkit.history import FileHistory
from prompt_toolkit.patch_stdout import StdoutProxy
from xgolib import XGO_DOG

from .logger_fabric import LoggerFabric, logging
from .camera_handler import CameraHandler


# region TestRunner
class TestRunner:
    """
    Поддерживает трансляцию камеры, приём команд и запуск тестов на робо-собаке.

    Параметры:
    - logger_fabric: Фабрика логгеров.
    - dog_port: Последовательный порт робота.
    - dog_baud: Скорость подключения к роботу.
    - camera_handler_params: Параметры CameraHandler.
    """
    # region Инициализация и остановка
    def __init__(
        self,
        logger_fabric : LoggerFabric,
        *,
        dog_port : str = "/dev/ttyAMA0",
        dog_baud : int = 115200,
        camera_handler_params : Dict[str, Any] | None = None
    ):
        self._logger = logger_fabric.get_logger(f"{type(self).__name__}")
        self._cam_handler = None
        self._dog = None
        self._ready = False
        self._stopped = False
        try:
            self._cam_handler = CameraHandler(logger_fabric, **(camera_handler_params or {}))
            self._dog = XGO_DOG(port=dog_port, baud=dog_baud)
        except (Exception, KeyboardInterrupt):
            self._logger.exception("Runner initialization failed")
            self.stop()
            return
        self._ready = True

    @property
    def ready(self):
        """
        Показывает, завершилась ли инициализация и доступен ли запуск тестов.
        """
        return self._ready

    def _reset_dog(self):
        """
        Останавливает движение и сбрасывает состояние робота.
        Ошибка одной команды не препятствует выполнению следующей.
        """
        if self._dog is None:
            return
        for name in ("stop", "reset"):
            try:
                getattr(self._dog, name)()
            except Exception:
                self._logger.exception("Robot command failed: %s", name)

    def stop(self):
        """
        Останавливает робота и камеру, закрывает порт. Повторный вызов допустим.
        """
        if self._stopped:
            return
        self._stopped = True
        self._ready = False
        self._reset_dog()
        if self._cam_handler is not None:
            try:
                self._cam_handler.stop()
            except Exception:
                self._logger.exception("Failed to stop camera")
        if self._dog is not None:
            try:
                self._dog.ser.close()
            except Exception:
                self._logger.exception("Failed to close robot connection")
    # endregion

    # region Терминал и команды
    def run_terminal(self, main_logger : logging.Logger):
        """
        Принимает команды с историей и завершает ресурсы при выходе.

        Параметры:
        - main_logger: Главный логгер, созданный в main. Его консольные
            обработчики временно перенаправляются через StdoutProxy.
        """
        if not self._ready:
            self._logger.error("Runner is not ready")
            return
        try:
            session = PromptSession(history=FileHistory(".runner_history"))
            with StdoutProxy() as output:
                handlers = [
                    handler for handler in main_logger.handlers
                    if isinstance(handler, logging.StreamHandler)
                    and not isinstance(handler, logging.FileHandler)
                ]
                streams = [(handler, handler.stream) for handler in handlers]
                try:
                    for handler in handlers:
                        handler.setStream(output)
                    while True:
                        try:
                            command = session.prompt("runner > ").strip()
                        except KeyboardInterrupt:
                            continue
                        except EOFError:
                            break
                        if not command:
                            continue
                        self._logger.info("Command: %s", command)
                        try:
                            if not self._execute_command(command):
                                break
                        except KeyboardInterrupt:
                            self._logger.info("Command interrupted")
                        except Exception:
                            self._logger.exception("Command execution failed")
                finally:
                    for handler, stream in streams:
                        handler.setStream(stream)
        except (Exception, KeyboardInterrupt):
            self._logger.exception("Terminal stopped unexpectedly")
        finally:
            self.stop()

    def _execute_command(self, command : str):
        """
        Выполняет команду; возвращает False при запросе выхода.
        """
        if not self._ready:
            self._logger.error("Runner is not ready")
            return False
        try:
            parts = shlex.split(command)
        except ValueError as error:
            self._logger.error("Invalid command: %s", error)
            return True
        if not parts:
            return True
        name, *args = parts
        if name in ("exit", "help", "shot") and args:
            self._logger.warning("Usage: %s", name)
            return True
        if name == "exit":
            return False
        if name == "help":
            self._logger.info(
                "Commands:\n"
                "  - help -- show help\n"
                "  - shot -- take a camera snapshot\n"
                "  - run <name> -- run a Python test file\n"
                "  - exit -- exit the program"
            )
        elif name == "shot":
            self._cam_handler.shot()
        elif name == "run":
            if len(args) != 1:
                self._logger.error("Usage: run <name>")
            else:
                self._run(args[0])
        else:
            self._logger.error("Unknown command: %s", name)
        return True
    # endregion

    # region Загрузка и выполнение тестов
    def _import_module(self, name):
        """
        Загружает Python-файл по пути относительно рабочей папки.
        Расширение .py можно опустить. Каждый запуск использует новый модуль.
        """
        module_name = f"_robotrio_test_{uuid.uuid4().hex}"
        try:
            file_path = Path(name)
            if file_path.suffix != ".py":
                file_path = Path(f"{file_path}.py")
            file_path = file_path.resolve()
            if not file_path.is_file():
                self._logger.error("Could not find file: %s", file_path)
                return
            spec = importlib.util.spec_from_file_location(module_name, file_path)
            if spec is None or spec.loader is None:
                self._logger.error("Could not load module: %s", file_path)
                return
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)
        except (Exception, KeyboardInterrupt, SystemExit):
            sys.modules.pop(module_name, None)
            self._logger.exception("Module loading failed: %s", name)
            return
        return module

    def _run(self, name):
        """
        Вызывает main(logger, dog, camera); после теста останавливает робота.
        """
        if not self._ready:
            self._logger.error("Runner is not ready")
            return
        module = None
        try:
            module = self._import_module(name)
            if module is None:
                return
            if not callable(getattr(module, "main", None)):
                self._logger.error("Test module has no callable main: %s", name)
                return
            module.main(self._logger, self._dog, self._cam_handler)
            self._logger.info("Test completed: %s", name)
        except KeyboardInterrupt:
            self._logger.info("Test interrupted: %s", name)
        except (Exception, SystemExit):
            self._logger.exception("Test execution failed: %s", name)
        finally:
            try:
                self._reset_dog()
            finally:
                if module is not None:
                    sys.modules.pop(module.__name__, None)
    # endregion
# endregion
