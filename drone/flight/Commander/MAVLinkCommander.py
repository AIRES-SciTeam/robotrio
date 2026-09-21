#!/opt/python-venv/bin/python3
from pymavlink import mavutil
import logging
import threading
from dataclasses import dataclass
import time


@dataclass
class DRONE_DroneState:
    flight_mode : str | None = None     # режим полёта
    armed : bool | None = None          # взведённость 
    landed : bool | None = None
    last_landed_at : float | None = None

    x: float | None = None                                  # позиция, м, локальная NED
    y: float | None = None
    z: float | None = None                                  # вниз — положительное направление
    q : tuple[float, float, float, float] | None = None     # Кватернион ориентации дрона
    vx: float | None = None                                 # скорость, м/с, локальная NED
    vy: float | None = None
    vz: float | None = None

    last_heartbeat_at: float | None = None      # время получения последнего heartbeat, time.monotonic()
    last_position_at: float | None = None       # время получения последнего положения, time.monotonic()
    last_attitude_at: float | None = None       # время получения последнего кватерниона, time.monotonic()
    

class DRONE_MAVLinkCommander:
    """
    Двусторонняя связь с PX4 и отправка команд управления через MAVLink.
    Ответственность:
        * Отправка heartbeat от GCS и приём телеметрии в фоновых потоках.
        * Контроль соединения по heartbeat, переподключение при его потере.
        * Взведение и выключение моторов, смена режима, посадка, ручное управление.
        * Периодическая отправка цели положения или скорости и параметров поворота.

    Инициализация __init__. Аргументы:
        * conn_address : str -- адрес соединения с PX4.
        * logger : logging.Logger -- журнал сообщений.
        * heartbeat_period : float = 1.0 -- период отправки heartbeat, с.
        * heartbeat_timeout : float = 5.0 -- таймаут приёма heartbeat, с;
          также используется для проверки давности состояния приземления.
        * offboard_period : float = 0.05 -- период отправки цели Offboard, с.
        * offboard_timeout : float = 1.0 -- максимальный интервал между
          вызовами set_target() после включения Offboard, с.

    Получение состояния:
        * get_drone_position() -- возвращает (x, y, z, q): координаты в метрах
          в локальной NED (север, восток, вниз) и кватернион q=(w, x, y, z).
          Неполученные значения равны None; метод не проверяет давность данных.
        * is_armed() -- None при отсутствии готового соединения, иначе bool.
          В текущей реализации неизвестное значение armed также даёт False.
        * is_landed() -- True при подтверждённом нахождении на земле,
          False при другом определённом landed_state, None при неизвестном,
          устаревшем состоянии или отсутствии готового соединения.
          Источник -- EXTENDED_SYS_STATE от целевого компонента PX4.

    Команды и режимы:
        * arm(check=True) / disarm(check=True) -- взведение / выключение моторов.
          При check=True ожидают нужное значение armed до 2 с; уже достигнутое
          состояние считается успехом. Возвращают True при успехе и False
          при отсутствии соединения или таймауте подтверждения.
          При check=False успешная отправка считается успехом без ожидания.
        * flight_mode(name, check=True) -- запрашивает режим по имени из таблицы
          соединения. При check=True ожидает его подтверждения до 2 с.
          Возвращает False при отсутствии соединения, неизвестном имени или
          таймауте; True при подтверждении либо отправке с check=False.
          Сам по себе этот метод не останавливает поток Offboard.
        * manual() -- останавливает поток Offboard, ожидает его завершения
          и вызывает flight_mode("MANUAL"). Возвращает результат смены режима.
        * land() -- останавливает поток Offboard, ожидает его завершения,
          сбрасывает состояние приземления и вызывает flight_mode("LAND").
          Возвращаемый True подтверждает режим, а не приземление;
          приземление проверяется последующими вызовами is_landed().
        * manual_control(roll, pitch, yaw, throttle) -- отправляет ручную команду
          только при текущем режиме MANUAL и готовом соединении.
          roll, pitch, yaw -- значения от -1000 до 1000; throttle -- от 0 до 1000.
          Не переключает режим и не поддерживает частоту отправки самостоятельно;
          её обеспечивает вызывающий контроллер. Возвращает None.

    Управление в Offboard:
        * set_target(pos=None, vel=None, yaw=None, yaw_rate=None):
            * pos -- (x, y, z), м, локальная NED. При наличии имеет приоритет:
              vel игнорируется независимо от переданного значения.
            * vel -- (vx, vy, vz), м/с, локальная NED, используется при pos=None.
              Если pos и vel равны None, задаётся нулевая линейная скорость.
            * yaw -- абсолютный курс NED, рад; None отключает это поле.
            * yaw_rate -- угловая скорость вокруг оси Z NED, рад/с;
              None отключает это поле. Если заданы оба поля поворота, активны оба.
          Атомарно обновляет маску, данные цели и время последнего вызова.
          Если поток работает, возвращает True после обновления цели.
          Иначе запускает поток, ждёт 1.1 с перед запросом OFFBOARD и ожидает
          подтверждения режима. При неудаче останавливает поток и возвращает False.
          При отсутствии готового соединения также возвращает False.
          Достижение цели не проверяется. Для продолжения управления нужно
          регулярно вызывать set_target(), даже если сама цель не меняется.
        * _offboard_control() -- отправляет сохранённую цель с offboard_period.
          По истечении offboard_timeout запрашивает MANUAL и повторяет запрос
          не чаще раза в секунду до подтверждения. До выхода из цикла продолжает
          отправлять последнюю цель. При завершении очищает цель и таймер.

    Жизненный цикл:
        * _heartbeat() -- фоновая отправка heartbeat.
        * _get_state() -- приём состояния и контроль потери heartbeat.
        * _reconnect() -- сброс состояния и попытка восстановления соединения.
        * stop() -- остановка потоков состояния, heartbeat и Offboard,
          ожидание их завершения и закрытие соединения. Режим полёта не меняет.
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

        self.offboard_mask = None
        self.offboard_target = None
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
                elif msg.get_type() == "EXTENDED_SYS_STATE" and msg.get_srcSystem() == conn.target_system and msg.get_srcComponent() == conn.target_component:
                    with self._state_lock:
                        self.drone_state.last_landed_at = time.monotonic()
                        if msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED:
                            self.drone_state.landed = None
                        else:
                            self.drone_state.landed = msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
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

    def is_landed(self):
        if not self.ready or self.conn is None:
            return None
        with self._state_lock:
            last_at = self.drone_state.last_landed_at
            if last_at is None or time.monotonic() - last_at > self.heartbeat_timeout:
                return None
            return self.drone_state.landed

    def land(self):
        self._offboard_stop.set()
        if self.offboard_thread is not None and self.offboard_thread.is_alive():
            self.offboard_thread.join()
        with self._state_lock:
            self.drone_state.landed = None
            self.drone_state.last_landed_at = None
        return self.flight_mode("LAND")

    def is_armed(self):
        if not self.ready or self.conn is None:
            return None
        if self.drone_state.armed:
            return True
        else:
            return False

    def arm(self, check : bool = True):
        with self._conn_lock:
            if not self.ready or self.conn is None:
                return False
            if self.drone_state.armed:
                return True
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
            if self.drone_state.armed is False:
                return True
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

    def manual(self):
        self._offboard_stop.set()
        if self.offboard_thread is not None and self.offboard_thread.is_alive():
            self.offboard_thread.join()
        return self.flight_mode("MANUAL")

    def manual_control(self, roll, pitch, yaw, throttle):
        if self.drone_state.flight_mode != "MANUAL":
            return
        with self._conn_lock:
            if not self.ready or self.conn is None:
                return
            self.conn.mav.manual_control_send(
                self.conn.target_system,
                pitch, roll, throttle, yaw, 0
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


    def set_target(self, pos : tuple[float, float, float] = None, vel : tuple[float, float, float] = None, yaw=None, yaw_rate=None):
        if not self.ready or self.conn is None:
            return False

        with self._target_lock:
            self.offboard_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            )

            if pos is not None:
                self.offboard_mask |= (
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                )
                self.offboard_target = pos
            else:
                self.offboard_mask |= (
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
                    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
                    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
                )
                self.offboard_target = (0, 0, 0)

            self.offboard_target = (*self.offboard_target, *vel) if vel is not None else (*self.offboard_target, 0, 0, 0)

            self.offboard_target = (*self.offboard_target, 0, 0, 0)

            if yaw is None:
                self.offboard_mask |= (
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
                )
                self.offboard_target = (*self.offboard_target, 0)
            else:
                self.offboard_target = (*self.offboard_target, yaw)

            if yaw_rate is None:
                self.offboard_mask |= (
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
                )
                self.offboard_target = (*self.offboard_target, 0)
            else:
                self.offboard_target = (*self.offboard_target, yaw_rate)

            self.last_target_at = time.monotonic()
            if self.offboard_thread is not None and self.offboard_thread.is_alive():
                return True

            self._offboard_starting = True
            self._offboard_stop.clear()
            self.offboard_thread = threading.Thread(
                target=self._offboard_control,
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

    def _offboard_control(self):
        m = mavutil.mavlink
        manual_requested_at = None

        while not self._offboard_stop.is_set():
            with self._target_lock:
                target = self.offboard_target
                mask = self.offboard_mask
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
                    *target
                )
            self._offboard_stop.wait(self.offboard_period)

        with self._target_lock:
            self.offboard_target = None
            self.offboard_mask = None
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
