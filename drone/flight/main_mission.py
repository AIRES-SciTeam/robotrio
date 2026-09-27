#!/opt/python-venv/bin/python3
"""Transfer three cargo links using predefined coordinates, without vision."""

from pathlib import Path
import math
import time

from Modules.Logger.LoggerFabric import LoggerFabric
from Modules.GripController.GripController import GripController, GZConfig, GripConfig
from Modules.SimpleCommander.Commander import Commander, MAVLinkConnConfig, OffboardConfig
from Modules.MissionController.MissionBlocks import Arm, Land, MoveTo, FlightMode, Grip, Release
from Modules.MissionController.MissionController import (
    DRONE_MissionController, MissionStep, MissionState,
)


# Gazebo world ENU coordinates, metres; scene/scene.world and model SDFs.
BASE_XY = (-4.98, 4.75)
CARGO_TRANSFERS = (
    ("00", (-4.5, 4.7), (3.6, 4.0)),
    ("01", (-4.3, 4.7), (4.0, 4.0)),
    ("02", (-4.5, 4.5), (3.6, 3.6)),
)
FLIGHT_ALTITUDE = 3.0
PICKUP_APPROACH_ALTITUDE = 1.0
# Heights above the initial PX4 position on the helipad, not world Z.
GRIP_HEIGHT = 0.3
RELEASE_HEIGHT = 0.4
# Empirical target correction: forward/right in the initial body frame, metres.
# Estimated from the observed cargo position under the front-right motor.
CARGO_CENTERING_FR = (0.0, 0.0)
TICK_PERIOD = 0.1


def build_plan(home, grip_height, release_height, initial_yaw=0.0):
    """Anchor the fixed ENU map to the initial local PX4 NED position."""
    forward, right = CARGO_CENTERING_FR
    correction_n = forward * math.cos(initial_yaw) - right * math.sin(initial_yaw)
    correction_e = forward * math.sin(initial_yaw) + right * math.cos(initial_yaw)

    def point(xy, height):
        north = home[0] + xy[1] - BASE_XY[1]
        east = home[1] + xy[0] - BASE_XY[0]
        if xy != BASE_XY:
            north += correction_n
            east += correction_e
        return (north, east, home[2] - height)

    def move(xy, height, precise=False, yaw=None):
        return MissionStep(MoveTo, {
            "pos": point(xy, height),
            "yaw": yaw,
            "eps": 0.03 if precise else 0.15,
            "speed_eps": 0.05 if precise else 0.15,
        })

    def course(origin, destination):
        start = point(origin, 0)
        end = point(destination, 0)
        return math.atan2(end[1] - start[1], end[0] - start[0])

    plan = [
        MissionStep(FlightMode, {"mode": "LOITER"}),
        MissionStep(Arm),
    ]
    previous = BASE_XY
    for index, (tag, pickup, destination) in enumerate(CARGO_TRANSFERS):
        pickup_yaw = course(previous, pickup)
        delivery_yaw = course(pickup, destination)
        approach_altitude = PICKUP_APPROACH_ALTITUDE if index == 0 else FLIGHT_ALTITUDE
        plan.extend([
            move(previous, approach_altitude, yaw=pickup_yaw),
            move(pickup, approach_altitude, yaw=pickup_yaw),
        ])
        if index > 0:
            plan.append(move(pickup, PICKUP_APPROACH_ALTITUDE, yaw=initial_yaw))
        plan.extend([
            move(pickup, grip_height, precise=True, yaw=initial_yaw),
            MissionStep(Grip, {"tag": tag}),
            move(pickup, FLIGHT_ALTITUDE, yaw=delivery_yaw),
            move(destination, FLIGHT_ALTITUDE, yaw=delivery_yaw),
            move(destination, release_height, precise=True, yaw=initial_yaw),
            MissionStep(Release),
        ])
        previous = destination
    base_yaw = course(previous, BASE_XY)
    plan.extend([
        move(previous, FLIGHT_ALTITUDE, yaw=base_yaw),
        move(BASE_XY, FLIGHT_ALTITUDE, yaw=base_yaw),
        move(BASE_XY, grip_height, precise=True, yaw=initial_yaw),
        MissionStep(Land),
        MissionStep(FlightMode, {"mode": "LOITER"}),
    ])
    return plan


def main():
    logger_fabric = LoggerFabric(
        basic_level="debug",
        logs_dir=Path(__file__).parents[2] / "logs",
    )
    logger = logger_fabric.get_logger("MainMission", use_stderr=True)
    commander = None
    grip_ctrl = None
    mission = None
    try:
        # Gripper sends detach commands for every configured cargo on initialization.
        grip_ctrl = GripController(
            gz_config=GZConfig(tags=["00", "01", "02", "03", "04", "05"]),
            grip_config=GripConfig(
                gripping_point=(0, 0, 0),
                gripping_eps=(0.1, 0.1, 0.3),
            ),
            logger_fabric=logger_fabric,
        )
        logger.info("GripController initialized; initial cargo detach commands sent")
        commander = Commander(
            logger_fabric=logger_fabric,
            conn_config=MAVLinkConnConfig(),
            offboard_config=OffboardConfig(),
        )
        logger.info("Waiting for local position")
        deadline = time.monotonic() + 10.0
        while True:
            x, y, z, q = commander.get_drone_position()
            if all(value is not None for value in (x, y, z, q)):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("PX4 local position unavailable")
            time.sleep(TICK_PERIOD)

        w, qx, qy, qz = q
        initial_yaw = math.atan2(2 * (w*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
        logger.info("Cargo centering forward/right=%s, initial_yaw=%.3f", CARGO_CENTERING_FR, initial_yaw)
        plan = build_plan((x, y, z), GRIP_HEIGHT, RELEASE_HEIGHT, initial_yaw)
        for index, step in enumerate(plan):
            logger.info("Step %d: %s %s", index + 1, step.block_class.__name__, step.block_params)
        mission = DRONE_MissionController(
            plan=plan,
            commander=commander,
            grip_ctrl=grip_ctrl,
            logger_fabric=logger_fabric,
            tick_period=TICK_PERIOD,
        )
        logger.info("Starting fixed-coordinate cargo mission")
        mission.start()
        while mission.state not in (MissionState.COMPLETED, MissionState.FAILED):
            time.sleep(TICK_PERIOD)
        logger.info("Mission finished: %s", mission.state.name)
        return 0 if mission.state == MissionState.COMPLETED else 1
    except KeyboardInterrupt:
        logger.info("Mission interrupted")
        return 130
    except Exception:
        logger.exception("Mission execution failed")
        return 1
    finally:
        if mission is not None:
            mission.stop()
        if commander is not None:
            commander.stop()


if __name__ == "__main__":
    raise SystemExit(main())
