from abc import ABC, abstractmethod
import threading

from Modules.GripController.GripController import GripController
from Modules.Commander.Commander import Commander
from Modules.Logger.LoggerFabric import LoggerFabric


class ManualController(ABC):
    def __init__(
        self,
        commander: Commander,
        grip_ctrl: GripController,
        logger_fabric: LoggerFabric,
        rate : int = 50,
        type : str | None = None
    ):
        self._logger = logger_fabric.get_logger(f"Manual Controller {type}")
        
        self.commander = commander
        self.gripper = grip_ctrl
        self.rate = rate

        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._spin,
            daemon=True
        )

    def _toggle_arming(self):
        res = self.commander.toggle_arming()
        self._logger.info("Arming result: %s", res.name)
        
    def _manual_control(self, roll, pitch, yaw, throttle):
        res = self.commander.manual_control(roll, pitch, yaw, throttle)
        self._logger.log(5, "Manual control sending res: %s", res.name)

    def _toggle_gripping(self) -> None:
        res = self.gripper.toggle()
        self._logger.info("Grip/release result: %s", res)

    def stop(self):
        self._stop_event.set()
        if (
            self._thread.is_alive()
            and self._thread != threading.current_thread()
        ):
            self._thread.join()

    @abstractmethod
    def _spin(self) -> None:
        pass
