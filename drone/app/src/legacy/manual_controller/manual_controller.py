from abc import ABC, abstractmethod
import threading

from logger.logger_fabric import LoggerFabric
from grip_controller.grip_controller import GripController
from safety_commander.commander import Commander, CommandResult


MODULE_NAME="manual_controller"

class ManualController(ABC):
    def __init__(
        self,
        commander: Commander,
        grip_controller: GripController,
        logger_fabric: LoggerFabric,
        rate : int = 50,
    ):
        if not logger_fabric.is_alive():
            raise RuntimeError("LoggerFabric is stopped")
        self._logger = logger_fabric.get_logger(f"{type(self).__name__}", module_name=MODULE_NAME, level="trace")
        self._logger.info("Initializing")
        self.commander = commander
        self.gripper = grip_controller
        self.rate = rate

        self._stop_event = threading.Event()
        self._arming_thread = None
        self._thread = threading.Thread(
            target=self._spin,
            daemon=True
        )

    def _toggle_arming(self):
        if self._stop_event.is_set():
            return
        if self._arming_thread is not None and self._arming_thread.is_alive():
            return
        self._arming_thread = threading.Thread(target=self._run_arming, daemon=True)
        self._arming_thread.start()

    def _run_arming(self):
        try:
            self._perform_arming()
        except Exception:
            self._logger.exception("Arming sequence failed")

    def _perform_arming(self):
        armed = self.commander.armed()
        if armed is None:
            self._logger.warning("Cannot toggle arming: armed state is unknown")
            return
        if armed:
            res = self.commander.disarm()
            self._logger.info("Disarming result: %s", res.name)
            return
        res = self.commander.loiter()
        if res != CommandResult.SUCCESS:
            self._logger.error("Cannot arm: LOITER transition failed: %s", res.name)
            return
        if self._stop_event.is_set():
            return
        res = self.commander.arm()
        self._logger.info("Arming result: %s", res.name)
        if res != CommandResult.SUCCESS:
            return
        if self._stop_event.is_set():
            return
        res = self.commander.set_flight_mode("POSCTL")
        if res != CommandResult.SUCCESS:
            self._logger.error("Armed, but POSCTL transition failed: %s", res.name)
            return
        self._logger.info("POSCTL ready for manual control")
        
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
        if self._arming_thread is not None and self._arming_thread != threading.current_thread():
            self._arming_thread.join()

    @abstractmethod
    def _spin(self) -> None:
        pass
