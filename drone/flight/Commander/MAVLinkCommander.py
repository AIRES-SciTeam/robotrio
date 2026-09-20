#!/opt/python-venv/bin/python3
from pymavlink import mavutil
import logging
import threading
from dataclasses import dataclass
import time


@dataclass
class DRONE_DroneState:
    flight_mode : str | None = None                         # режим полёта
    armed : bool | None = None                              # взведённость 

    x: float | None = None                                  # позиция, м, локальная NED
    y: float | None = None
    z: float | None = None                                  # вниз — положительное направление
    q : tuple[float, float, float, float] | None = None     # Кватернион ориентации дрона
    vx: float | None = None                                 # скоротсть, м/с, локальная NED
    vy: float | None = None
    vz: float | None = None

    last_heartbeat_at: float | None = None  # время получения последнего heartbeat, time.monotonic()
    last_position_at: float | None = None   # время получения последнего положения, time.monotonic()
    last_attitude_at: float | None = None   # время получения последнего кватерниона, time.monotonic()
    

class DRONE_MAVLinkCommander:
    """
    Класс, реализуюший двустороннюю связь с дроном.
    Ответственность:
        1.  Роль передатчика.
            * Отправка heartbeat, как системы GCS.
            * Отправка команд запуска и остановки моторов.
            * Отправка команды ручного управления дроном.
            * Отправка команды смены режима полёта.
            * Отправка команды управления дроном через целевое положение.
            * Отправка команды управления дроном через целевую скорость. (coming soon)
        2. Роль приёмника
            * Контроль подключения к дрону через heartbeat от PX4.
            * Контроль режима полёта.
            * Контроль текущего положения дрона.
            * Контроль исполнения команды.
    Интерфейс:
        Подключение и завершение:
        1. Инициализация __init__. Аргументы:
            * conn_address : str -- адрес соединения с PX4.
            * logger : logging.Logger -- объект для записи сообщений в лог.
            * heartbeat_period : float = 1.0 -- период отправки heartbeat в секундах.
            * heartbeat_timeout : float = 5.0 -- таймаут получения heartbeat в секундах.
            * offboard_period : float = 0.05 -- период отправки цели в Offboard в секундах.
            * offboard_timeout : float = 1.0 -- максимальный интервал между вызовами
              set_target_position() в секундах.
        2. Метод _reconnect() -- сбрасывает состояние и пытается переподключиться к дрону.
        3. Метод stop() -- останавливает потоки состояния, heartbeat и Offboard, ожидает их
           завершения и закрывает соединение с дроном.

        Служебная отправка:
        4. Метод _heartbeat() -- отправка heartbeat для PX4.

        Получение состояния:
        5. Метод _get_state() -- получение сообщений от PX4, отслеживание состояния дрона. 
           Если heartbeat не приходит в течение heartbeat_timeout -- пытается переоткрыть соединение.
        6. Метод get_drone_position() -- возвращает положение дрона в формате (x, y, z).

        Ручное управление и режимы:
        7. Методы arm()/disarm() -- включение/выключение двигателей. Аргументы:
            * check : bool = True -- требовать подтверждения исполнения команды от дрона.
           Подтверждение определяется по текущему режиму дрона в течение 10 мс.
        8. Метод manual_control() -- оправка команды ручного управления. Аргументы:
            * roll : int[-1000, 1000] -- значение крена.
            * pitch : int[-1000, 1000] -- значение тангажа.
            * yaw : int[-1000, 1000] -- значение рысканья.
            * throttle : int[0, 1000] -- значение тяги.
           Обеспечить необходимую частоту должен вызывющий класс.
        9. Метод flight_mode() -- отправка команды на изменение режима полёта. Аргументы:
            * name : str -- название режима. 
            * check : bool = True -- ожидание подтверждения перехода.

        Управление в Offboard:
        10. Метод set_target_position() -- назначить параметры команды управления через указание 
           целевой позиции в системе координат NED. Аргументы:
            * x : float -- положение по оси X в м.
            * y : float -- положение по оси Y в м.
            * z : float -- положение по оси Z в м (направлена вниз).
           Первый запуск запускает постоянную отправку цели, запрашивает переход в режим Offboard, 
           ожидает подтвержения смены по текущему состоянию дрона, возвращает True. При отсутсвии 
           подтверждения перехода прекращает отправку, выводит warning. Возвращает False.
        11. Метод _send_target_position() -- отправка команды управления через целевую позицию. Аргументы:
            * perios_ms : int = 50 -- частота отправки в мс.
           Обеспечивает постоянную отправку команды. При отсутствии вызова команды set_target_position
           в течение timeout переводит дрон в ручной режим.

    По контракту класса ручное управление должно быть доступно и отправлять команды всегда при жизни класса.
    """
    def __init__(
        self,
        conn_address : str,
        logger : logging.Logger,
        heartbeat_period : float = 1.0,
        heartbeat_timeout : float = 5.0,
        offboard_period : float = 0.05,
        offboard_timeout : float = 1.0
    ): 
        self.logger = logger
        self.conn_address = conn_address

        self.heartbeat_period = heartbeat_period
        self.heartbeat_timeout = heartbeat_timeout
        self.offboard_period = offboard_period
        self.offboard_timeout = offboard_timeout

        self._state_lock = threading.Lock()
        self._conn_lock = threading.Lock()
        self._stop_event = threading.Event()
        self.drone_state = DRONE_DroneState()
        self.ready = False
        self._ever_connected = False
        self.conn : mavutil.mavfile | None = None
        self.logger.info(f"DRONE_MAVLinkCommander: Waiting for PX4 heartbeat at {self.conn_address}")
        while not self._reconnect():
            pass
        self.state_thread = threading.Thread(
            target=self._get_state,
            daemon=True
        )
        self.state_thread.start()

        self.heartbeat_thread = threading.Thread(
            target=self._heartbeat,
            daemon=True
        )
        self.heartbeat_thread.start()

        self.target_position = None
        self.last_target_at = None
        self._target_lock = threading.Lock()
        self._offboard_stop = threading.Event()
        self._offboard_starting = False
        self.offboard_thread = None

        self.logger.info("DRONE_MAVLinkCommander: Initialized")

    def _reconnect(self):
        self.logger.debug(f"DRONE_MAVLinkCommander: Opening PX4 connection at {self.conn_address}")
        self.ready = False
        with self._conn_lock:
            if self.conn is not None:
                self.conn.close()
                self.conn = None
        with self._state_lock:
            self.drone_state = DRONE_DroneState()

        conn = mavutil.mavlink_connection(
            self.conn_address,
            source_system=255
        )

        msg = conn.wait_heartbeat(timeout=self.heartbeat_timeout)
        if msg is not None:
            if self._stop_event.is_set():
                conn.close()
                return False
            with self._state_lock:
                self.drone_state.last_heartbeat_at = time.monotonic()
                self.drone_state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.drone_state.flight_mode = mavutil.mode_string_v10(msg)
            with self._conn_lock:
                self.conn = conn
            self.ready = True
            if self._ever_connected:
                self.logger.info("DRONE_MAVLinkCommander: PX4 connection restored")
            else:
                self.logger.info("DRONE_MAVLinkCommander: PX4 connected")
            self._ever_connected = True
            return True

        conn.close()
        self.logger.debug(f"DRONE_MAVLinkCommander: PX4 heartbeat not received within {self.heartbeat_timeout:.1f} s")
        return False

    def stop(self):
        self._stop_event.set()
        self._offboard_stop.set()
        self.ready = False
        for thread in (self.state_thread, self.heartbeat_thread, self.offboard_thread):
            if thread is not None and thread.is_alive() and thread is not threading.current_thread():
                thread.join()
        with self._conn_lock:
            if self.conn is not None:
                self.conn.close()
                self.conn = None
        self.logger.info("DRONE_MAVLinkCommander: Stopped")

    def _heartbeat(self):
        while not self._stop_event.wait(self.heartbeat_period):
            with self._conn_lock:
                if not self.ready or self.conn is None:
                    continue
                self.conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )

    def _get_state(self):
        while not self._stop_event.is_set():
            with self._state_lock:
                last_heartbeat_at = self.drone_state.last_heartbeat_at
            if last_heartbeat_at is not None and time.monotonic() - last_heartbeat_at <= self.heartbeat_timeout:
                conn = self.conn
                if conn is None:
                    continue
                msg = conn.recv_match(blocking=True, timeout=0.2)
                if msg is None:
                    pass
                elif msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == conn.target_system and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                    with self._state_lock:
                        previous_armed = self.drone_state.armed
                        previous_mode = self.drone_state.flight_mode
                        self.drone_state.last_heartbeat_at = time.monotonic()
                        self.drone_state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        self.drone_state.flight_mode = mavutil.mode_string_v10(msg)
                        armed = self.drone_state.armed
                        flight_mode = self.drone_state.flight_mode
                    if armed != previous_armed:
                        self.logger.info(f"DRONE_MAVLinkCommander: PX4 {'armed' if armed else 'disarmed'}")
                    if flight_mode != previous_mode:
                        self.logger.info(f"DRONE_MAVLinkCommander: PX4 flight mode: {flight_mode}")
                elif msg.get_type() == "LOCAL_POSITION_NED":
                    with self._state_lock:
                        self.drone_state.last_position_at = time.monotonic()
                        self.drone_state.x = msg.x
                        self.drone_state.y = msg.y
                        self.drone_state.z = msg.z
                        self.drone_state.vx = msg.vx
                        self.drone_state.vy = msg.vy
                        self.drone_state.vz = msg.vz
                elif msg.get_type() == "ATTITUDE_QUATERNION":
                    with self._state_lock:
                        self.drone_state.last_attitude_at = time.monotonic()
                        self.drone_state.q = (msg.q1, msg.q2, msg.q3, msg.q4)
            else:
                if self.ready:
                    self.logger.warning(f"DRONE_MAVLinkCommander: PX4 heartbeat lost for more than {self.heartbeat_timeout:.1f} s; reconnecting")
                self.ready = False
                if not self._reconnect():
                    self._stop_event.wait(0.2)

    def get_drone_position(self):
        return self.drone_state.x, self.drone_state.y, self.drone_state.z, self.drone_state.q

    def arm(self, check : bool = True):
        with self._conn_lock:
            if not self.ready or self.conn is None:
                return False
            self.logger.info("DRONE_MAVLinkCommander: Requesting PX4 arm")
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
        if check:
            now = time.monotonic()
            while time.monotonic() - now <= 2:
                if self.drone_state.armed == True:
                    self.logger.debug("DRONE_MAVLinkCommander: PX4 arm confirmed")
                    return True
            self.logger.warning("DRONE_MAVLinkCommander: PX4 did not arm within 2 s")
            return False
        self.logger.debug("DRONE_MAVLinkCommander: PX4 arm command sent without waiting for confirmation")
        return True

    def disarm(self, check : bool = True):
        with self._conn_lock:
            if not self.ready or self.conn is None:
                return False
            self.logger.info("DRONE_MAVLinkCommander: Requesting PX4 disarm")
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
            )
        if check:
            now = time.monotonic()
            while time.monotonic() - now <= 2:
                if self.drone_state.armed == False:
                    self.logger.debug("DRONE_MAVLinkCommander: PX4 disarm confirmed")
                    return True
            self.logger.warning("DRONE_MAVLinkCommander: PX4 did not disarm within 2 s")
            return False
        self.logger.debug("DRONE_MAVLinkCommander: PX4 disarm command sent without waiting for confirmation")
        return True

    def manual_control(self, roll, pitch, yaw, throttle):
        if self.drone_state.flight_mode != "MANUAL":
            return
        with self._conn_lock:
            if not self.ready or self.conn is None:
                return
            self.conn.mav.manual_control_send(
                self.conn.target_system,
                roll, pitch, throttle, yaw, 0
            )

    def flight_mode(self, name: str, check: bool = True) -> bool:
        name = name.upper()
        with self._conn_lock:
            if not self.ready or self.conn is None:
                return False
            mode_mapping = self.conn.mode_mapping()
            if mode_mapping is None or name not in mode_mapping:
                self.logger.warning(f"DRONE_MAVLinkCommander: Unknown PX4 flight mode: {name}")
                return False
            self.logger.info(f"DRONE_MAVLinkCommander: Requesting PX4 flight mode: {name}")
            self.conn.set_mode(name)

        if check:
            start = time.monotonic()
            while time.monotonic() - start <= 2:
                with self._state_lock:
                    current_mode = self.drone_state.flight_mode
                if current_mode == name:
                    self.logger.debug(f"DRONE_MAVLinkCommander: PX4 flight mode confirmed: {name}")
                    return True
                time.sleep(0.02)

            self.logger.warning(f"DRONE_MAVLinkCommander: PX4 did not enter flight mode {name} within 2 s")
            return False

        self.logger.debug(f"DRONE_MAVLinkCommander: PX4 flight mode command sent without waiting: {name}")
        return True


    def set_target_position(self, x, y, z):
        if not self.ready or self.conn is None:
            return False

        with self._target_lock:
            self.target_position = (x, y, z)
            self.last_target_at = time.monotonic()
            if self.offboard_thread is not None and self.offboard_thread.is_alive():
                return True

            self._offboard_starting = True
            self._offboard_stop.clear()
            self.offboard_thread = threading.Thread(
                target=self._send_target_position,
                daemon=True,
            )
            self.offboard_thread.start()

        time.sleep(1.1)
        if not self.ready or not self.flight_mode("OFFBOARD"):
            self._offboard_stop.set()
            self.offboard_thread.join(timeout=self.offboard_period + 0.2)
            self.logger.warning("DRONE_MAVLinkCommander: Could not enter PX4 Offboard mode")
            return False

        with self._target_lock:
            self.last_target_at = time.monotonic()
            self._offboard_starting = False
        return True

    def _send_target_position(self):
        m = mavutil.mavlink
        manual_requested_at = None
        mask = (
            m.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | m.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | m.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        while not self._offboard_stop.is_set():
            with self._target_lock:
                target = self.target_position
                last_target_at = self.last_target_at
                starting = self._offboard_starting

            if not self.ready or self.conn is None or target is None:
                break
            now = time.monotonic()
            if not starting and now - last_target_at > self.offboard_timeout:
                with self._state_lock:
                    current_mode = self.drone_state.flight_mode
                if current_mode == "MANUAL":
                    self.logger.info("DRONE_MAVLinkCommander: PX4 entered MANUAL after target timeout")
                    break
                if manual_requested_at is None or now - manual_requested_at >= 1.0:
                    if manual_requested_at is None:
                        self.logger.warning("DRONE_MAVLinkCommander: Target position timed out; requesting MANUAL")
                    else:
                        self.logger.debug("DRONE_MAVLinkCommander: Retrying MANUAL after target timeout")
                    self.flight_mode("MANUAL", check=False)
                    manual_requested_at = now
            else:
                manual_requested_at = None

            with self._conn_lock:
                if not self.ready or self.conn is None:
                    break
                self.conn.mav.set_position_target_local_ned_send(
                    0,
                    self.conn.target_system,
                    self.conn.target_component,
                    m.MAV_FRAME_LOCAL_NED,
                    mask,
                    *target,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0,
                )
            self._offboard_stop.wait(self.offboard_period)

        with self._target_lock:
            self.target_position = None
            self.last_target_at = None
            self._offboard_starting = False

                
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[
            logging.StreamHandler(),
        ],
    )
    logger = logging.getLogger("DRONE_MAVLinkCommander")
    com: DRONE_MAVLinkCommander | None = None

    try:
        com = DRONE_MAVLinkCommander(
            conn_address="udpin:127.0.0.1:14541",
            logger=logger,
        )

        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        if com is not None:
            com.stop()
