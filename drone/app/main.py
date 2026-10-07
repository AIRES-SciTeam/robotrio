"""Короткая автономная миссия с уже запущенными Gazebo и PX4.

Запуск в окружении симулятора: python3 drone/app/main.py
"""

from pathlib import Path
import signal
import sys
import threading
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from logger.logger_fabric import LoggerFabric
from safety_commander.commander import Commander
from safety_commander.mavlink_conn import MAVLinkConnConfig
from safety_commander.control_offboard import OffboardConfig
from mission.image_handlers import ImageReceiver, ImagePublisher
from mission.mission_env import MissionEnv
from mission.mission_controller import MissionController, MissionStep, MissionState
from mission.mission_block import FlightMode, Arm, Takeoff, MoveAboveTag
from mission.tag_detector import TagDetector


MAVLINK_ADDRESS = "udpin:127.0.0.1:14541"
CAMERA_IMAGE_TOPIC = "/x500/down_cam/image"
CAMERA_INFO_TOPIC = "/x500/down_cam/camera_info"
CAMERA_OUTPUT_TOPIC = "/mission/camera/image"
TAG_SIZE = 0.04  # В atlas00: чёрный квадрат 160 px на грани 400 px × 0.1 м.
HELIPAD_TAG_ID = 585
HELIPAD_TAG_SIZE = 0.08  # Чёрная граница 8/10 от площадки тега 0.1 м.
HEIGHT_ABOVE_TAG = 1.0


def flight_plan():
    return [
        MissionStep(FlightMode, {"mode": "LOITER", "timeout": 3.0}),
        MissionStep(Arm),
        MissionStep(Takeoff),  # z=-1 м в локальной NED.
        MissionStep(MoveAboveTag, {
            "tag_id": 0, "tag_size": TAG_SIZE, "height": HEIGHT_ABOVE_TAG,
        }),
        MissionStep(MoveAboveTag, {
            "tag_id": 5, "tag_size": TAG_SIZE, "height": HEIGHT_ABOVE_TAG,
        }),
        MissionStep(MoveAboveTag, {
            "tag_id": HELIPAD_TAG_ID, "tag_size": HELIPAD_TAG_SIZE, "height": HEIGHT_ABOVE_TAG,
        }),
    ]


def main() -> int:
    stop_event = threading.Event()
    previous_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.signal(sig, lambda *_: stop_event.set())

    logger_fabric = None
    logger = None
    commander = None
    mission = None
    ros_context = None
    ros_node = None
    ros_executor = None
    exit_code = 0
    reason = "normal_exit"

    try:
        logger_fabric = LoggerFabric(
            basic_level="info",
            logs_dir=Path(__file__).resolve().parents[2] / "logs",
        )
        logger = logger_fabric.get_logger("main", use_stderr=True)
        logger.info("Логи: %s", logger_fabric.get_logs_dir())
        logger.info("Подключение к PX4: %s", MAVLINK_ADDRESS)
        commander = Commander(
            logger_fabric=logger_fabric,
            conn_config=MAVLinkConnConfig(
                address=MAVLINK_ADDRESS,
                max_reconnects=3,
            ),
            offboard_config=OffboardConfig(),
        )
        if stop_event.is_set():
            return 0
        if not commander.is_alive() or commander.armed() is None:
            raise RuntimeError("Нет связи с PX4. Проверьте запуск симулятора и MAVLink.")

        ros_context = Context()
        rclpy.init(context=ros_context, signal_handler_options=SignalHandlerOptions.NO)
        ros_node = rclpy.create_node("camera_mission", context=ros_context)
        ros_executor = SingleThreadedExecutor(context=ros_context)
        ros_executor.add_node(ros_node)
        receiver = ImageReceiver(
            logger_fabric, ros_node,
            image_topic=CAMERA_IMAGE_TOPIC,
            camerainfo_topic=CAMERA_INFO_TOPIC,
        )
        publisher = ImagePublisher(logger_fabric, ros_node, topic=CAMERA_OUTPUT_TOPIC)
        mission_env = MissionEnv(
            commander=commander,
            logger=logger_fabric.get_logger("flight_mission", module_name="mission"),
            # Down camera: optical right -> body right, optical down -> aft.
            camera_rotation=np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]),
            camera_position=(0.1, 0., -0.22),
            image_receiver=receiver,
            image_publisher=publisher,
            tag_detector=TagDetector("tag36h11", logger_fabric),
        )
        mission = MissionController(
            flight_plan(),
            mission_env,
            tick_period=0.1,
            end_steps=[MissionStep(FlightMode, {"mode": "LOITER", "timeout": 3.0})],
            fail_steps=[MissionStep(FlightMode, {"mode": "LOITER", "timeout": 3.0})],
        )
        mission.start()
        logger.info("Миссия: LOITER → ARM → взлёт 1 м → тег 00 → тег 05 → helipad (%s) → LOITER", HELIPAD_TAG_ID)
        logger.info("Камера: %s. Завершение приложения: Ctrl+C.", CAMERA_IMAGE_TOPIC)
        completed_reported = False
        failed_reported = False
        while not stop_event.is_set():
            ros_executor.spin_once(timeout_sec=0.1)
            if not commander.is_alive():
                raise RuntimeError("Commander остановлен")
            if mission.state == MissionState.FAILED and not failed_reported:
                logger.error("Ошибка миссии: %s. Связь с PX4 сохраняется до Ctrl+C.", mission.last_error)
                exit_code = 1
                reason = "mission_failed"
                failed_reported = True
            if mission.state == MissionState.COMPLETED and not completed_reported:
                logger.info("Миссия завершена, удержание в LOITER")
                completed_reported = True
    except KeyboardInterrupt:
        reason = "interrupted"
    except Exception:
        exit_code = 1
        reason = "error"
        if logger is not None:
            logger.exception("Ошибка приложения")
        else:
            raise
    finally:
        # Остановить миссию до закрытия связи с автопилотом.
        for component in (mission, commander):
            if component is not None:
                try:
                    component.stop()
                except Exception:
                    exit_code = 1
                    reason = "cleanup_error"
                    if logger is not None:
                        logger.exception("Ошибка остановки %s", type(component).__name__)
        for resource, close in (
            (ros_executor, "shutdown"),
            (ros_node, "destroy_node"),
            (ros_context, "try_shutdown"),
        ):
            if resource is not None:
                try:
                    getattr(resource, close)()
                except Exception:
                    exit_code = 1
                    reason = "cleanup_error"
                    if logger is not None:
                        logger.exception("Ошибка остановки ROS: %s", close)
        if logger_fabric is not None:
            logger_fabric.finish_session(reason)
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
