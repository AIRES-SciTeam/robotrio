from __future__ import annotations

from xgolib import XGO_DOG
import time
import numpy as np
import inspect
import math
import csv
import os
from pathlib import Path

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .runner import ImageReceiver


BODY_Y_LIMIT = 18.0
SIDE_STEP_SPEED = 5
SIDE_STEP_SECONDS = 1.0
MAX_SIDE_STEPS = 8


R = np.array([
    [ 0,  0,  1],
    [-1,  0,  0],
    [ 0, -1,  0],
])


def find_and_align_tag(dog: XGO_DOG, img_recv: ImageReceiver, *, tag_id=0):
    """Search for the cargo tag, then turn towards its signed bearing."""
    positive_turn_speed = 7
    negative_turn_speed = -6
    turn_seconds = 1.5
    angle_tolerance = np.deg2rad(5)
    search_direction = -1

    dog.pace("slow")
    dog.attitude("p", 10)
    dog.translation("z", 80)
    dog.stop()
    time.sleep(1)

    while True:
        # Observe before moving, including when the tag is already in view.
        target = _observe_target(img_recv, tag_id=tag_id)
        if target is not None:
            position = np.asarray(target.pose_t).reshape(3) @ R.T

        if target is None:
            # If the tag disappears, continue the last directed turn.
            print("Target tag not visible; searching")
        else:
            # Body X points forward, Y left; positive SDK turn is left.
            angle = float(np.arctan2(position[1], position[0]))
            print(f"Tag {target.tag_id}: angle={np.rad2deg(angle):.2f} deg, "
                  f"center={target.center}")
            if abs(angle) <= angle_tolerance:
                dog.stop()
                return target
            search_direction = 1 if angle > 0 else -1

        # Keep a fixed step for now, even if it overshoots the center.
        # The next observation determines the new turn direction.
        turn_speed = positive_turn_speed if search_direction > 0 else negative_turn_speed
        print(f"Turn step: speed={turn_speed}, "
              f"duration={turn_seconds:.2f}s", flush=True)
        try:
            dog.turn(turn_speed)
            time.sleep(turn_seconds)
        finally:
            dog.stop()
        time.sleep(1)



def approach_tag(dog: XGO_DOG, img_recv: ImageReceiver, *, tag_id=None):
    """Approach along body X to 70–100 mm, keeping bearing within 5 degrees.

    With tag_id=None, choose among all visible tags by their normal's
    alignment with camera Y, using the same rule during turns and approach.
    """
    move_speed = 6
    min_step_seconds = 1.5
    max_step_seconds = 6.0
    min_step_distance_mm = 170.0
    doubling_distance_mm = 170.0
    min_x_mm, max_x_mm = 70, 100
    angle_tolerance = np.deg2rad(5)

    try:
        find_and_align_tag(dog, img_recv, tag_id=tag_id)
        while True:
            target = _observe_target(img_recv, tag_id=tag_id)
            if target is None:
                dog.stop()
                find_and_align_tag(dog, img_recv, tag_id=tag_id)
                continue

            position = np.asarray(target.pose_t).reshape(3) @ R.T * 1000
            angle = float(np.arctan2(position[1], position[0]))
            if abs(angle) > angle_tolerance:
                dog.stop()
                find_and_align_tag(dog, img_recv, tag_id=tag_id)
                continue  # Re-measure distance and select the target after turning.

            x_mm = float(position[0])
            print(f"Tag {target.tag_id}: X={x_mm:.1f} mm, "
                  f"angle={np.rad2deg(angle):.2f} deg")
            if min_x_mm <= x_mm <= max_x_mm:
                return target

            # Back up if a step has carried us closer than the lower bound.
            speed = move_speed if x_mm > max_x_mm else -move_speed
            # Minimum duration up to 170 mm, then double every 170 mm.
            # Cap the exponent before evaluation and re-observe at least
            # every 6 seconds of walking. Reverse steps stay at the minimum.
            exponent = np.clip(
                (x_mm - min_step_distance_mm) / doubling_distance_mm,
                0.0, np.log2(max_step_seconds / min_step_seconds),
            )
            step_seconds = float(min_step_seconds * 2.0 ** exponent)
            print(f"X step: speed={speed}, duration={step_seconds:.2f}s")
            try:
                dog.move_x(speed, step_seconds)
            finally:
                dog.stop()
            time.sleep(1)
    finally:
        dog.stop()


def observe_tag_pair(img_recv: ImageReceiver, *, tag_size_mm):
    """Return the physical midpoint (body axes, mm) of two tag25h9:1 tags.

    Both poses must come from the same frame. Do not guess a pair when
    fewer or more than two matching, valid detections are visible.
    """
    positions = []
    poses = img_recv.get_poses(family="tag25h9", tag_size=tag_size_mm / 1000)
    for pose in poses:
        family = pose.tag_family
        if isinstance(family, bytes):
            family = family.decode("ascii")
        if family != "tag25h9" or pose.tag_id != 1 or pose.pose_t is None:
            continue
        position = np.asarray(pose.pose_t).reshape(-1)
        if (position.size == 3 and np.isfinite(position).all()
                and position[2] > 0):
            positions.append(position)
    if len(positions) != 2:
        return None
    # Average 3D translations, not image pixels: depths may differ.
    return np.mean(positions, axis=0) @ R.T * 1000


def approach_tag_pair(dog: XGO_DOG, img_recv: ImageReceiver, *,
                      tag_size_mm=100.0, stop_distance_mm=100.0,
                      y_tolerance_mm=5.0):
    """Find two tag25h9:1 tags and approach their midpoint.

    Stop once measured X is at most the requested distance, with bearing within
    5 degrees and lateral error within y_tolerance_mm. Correct Y before
    advancing along X. This approaches the pair; it does not drive through it.
    Return the midpoint in body axes (mm) on arrival, or None if the pair
    disappears after acquisition. Observe only while stopped, after settling.
    """
    if not np.isfinite(tag_size_mm) or tag_size_mm <= 0:
        raise ValueError("tag_size_mm must be positive and finite")
    if not np.isfinite(stop_distance_mm) or stop_distance_mm <= 0:
        raise ValueError("stop_distance_mm must be positive and finite")
    if not np.isfinite(y_tolerance_mm) or y_tolerance_mm <= 0:
        raise ValueError("y_tolerance_mm must be positive and finite")
    pair_seen = False
    search_direction = -1
    angle_tolerance = np.deg2rad(5)
    try:
        dog.stop()
        dog.pace("slow")
        dog.attitude("p", 0)
        dog.translation("y", 0)
        dog.translation("z", 80)
        time.sleep(1)
        while True:
            midpoint = observe_tag_pair(img_recv, tag_size_mm=tag_size_mm)
            if midpoint is None:
                if pair_seen:
                    print("Pair not visible after settling; stopping approach", flush=True)
                    return None
                print("Need two tag25h9:1 tags; searching", flush=True)
                turn_speed = 7 if search_direction > 0 else -6
            else:
                pair_seen = True
                x_mm = float(midpoint[0])
                y_mm = float(midpoint[1])
                angle = float(np.arctan2(midpoint[1], midpoint[0]))
                print(f"Pair midpoint: X={x_mm:.1f} mm, Y={y_mm:.1f} mm, "
                      f"angle={np.rad2deg(angle):.2f} deg", flush=True)
                if abs(angle) <= angle_tolerance:
                    if abs(y_mm) > y_tolerance_mm:
                        # Body Y points left; walk towards the midpoint.
                        side_speed = float(np.copysign(SIDE_STEP_SPEED, y_mm))
                        print(f"Pair Y step: speed={side_speed:g}, "
                              f"duration={SIDE_STEP_SECONDS:.2f}s", flush=True)
                        try:
                            dog.move_y(side_speed, SIDE_STEP_SECONDS)
                        finally:
                            dog.stop()
                        time.sleep(1)
                        continue  # Re-check both bearing and Y while stopped.
                    if x_mm <= stop_distance_mm:
                        return midpoint
                    # Use the cargo approach duration curve relative to
                    # the requested stopping distance (minimum + 70 mm).
                    exponent = np.clip(
                        (x_mm - stop_distance_mm - 70.0) / 170.0, 0.0, 2.0,
                    )
                    duration = float(1.5 * 2.0 ** exponent)
                    print(f"Pair X step: speed=6, duration={duration:.2f}s", flush=True)
                    try:
                        dog.move_x(6)
                        time.sleep(duration)
                    finally:
                        dog.stop()
                    time.sleep(1)
                    continue
                search_direction = 1 if angle > 0 else -1
                turn_speed = 7 if search_direction > 0 else -6

            # No forward motion until both tags are visible and aligned.
            print(f"Pair turn: speed={turn_speed}, duration=1.50s", flush=True)
            try:
                dog.turn(turn_speed)
                time.sleep(1.5)
            finally:
                dog.stop()
            time.sleep(1)
    finally:
        dog.stop()


def select_tag(poses):
    """Choose the tag whose Z axis is most parallel to camera Y (either sign)."""
    best_pose = None
    best_score = -1.0
    for pose in poses:
        if pose.pose_R is None or pose.pose_t is None:
            continue
        rotation = np.asarray(pose.pose_R)
        position = np.asarray(pose.pose_t).reshape(3)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.isfinite(position).all() or position[2] <= 0):
            continue
        tag_z = rotation[:, 2]
        length = float(np.linalg.norm(tag_z))
        if length == 0:
            continue
        score = abs(float(tag_z[1])) / length
        if score > best_score:
            best_pose, best_score = pose, score
    if best_pose is None:
        raise RuntimeError("No tag with a valid pose visible")
    return best_pose


def _observe_target(img_recv: ImageReceiver, *, tag_id=None):
    poses = img_recv.get_poses()
    if tag_id is not None:
        poses = [pose for pose in poses if pose.tag_id == tag_id]
    try:
        return select_tag(poses)
    except RuntimeError:
        return None


def grip(dog : XGO_DOG, img_recv : ImageReceiver):
    # Refit from arm_tag_pairs_10mm.csv in arm.ipynb.
    M = np.array([
        [ 0.61951821, -0.00891379],
        [-0.1621911 ,  0.76268587]
    ])
    b = np.array([75.08762923, 22.50635963])

    poses = img_recv.get_poses()
    pose = select_tag(poses)
    if pose.pose_t is None:
        raise ValueError("Target tag pose unavailable")
    position = np.asarray(pose.pose_t).reshape(3) * 1000
    position = (position @ R.T)[[0, 2]]

    print("position: ", position)
    print(position @ M + b)
    x, z = position @ M + b
    dog.claw(30)
    time.sleep(1)
    dog.arm(x, z)
    time.sleep(1)
    dog.claw(255)
    time.sleep(1)
    dog.arm(0, 100)
    # input()


def align_y(dog : XGO_DOG, img_recv : ImageReceiver):
    body_y = 0.0
    side_steps = 0
    side_step_pending = False
    dog.stop()
    dog.translation("y", body_y)
    dog.translation("z", 60)
    time.sleep(2)

    for _ in range(30):
        poses = img_recv.get_poses()
        pose = select_tag(poses)
        if pose.pose_t is None:
            raise ValueError("Target tag pose unavailable")
        position = np.asarray(pose.pose_t).reshape(3)
        if not np.isfinite(position).all() or position[2] <= 0:
            raise ValueError("Invalid target tag pose")
        x_offset = float(position[0]) * 1e3
        print(f"Current offset (camera X): {x_offset:.2f} mm; body Y: {body_y:.2f} mm")
        if abs(x_offset) <= 1:
            break
        # Estimate the absolute body position required to cancel the error.
        required_y = body_y - x_offset
        if side_step_pending or abs(required_y) > BODY_Y_LIMIT:
            if side_steps >= MAX_SIDE_STEPS:
                raise RuntimeError("Side step limit reached before centering tag")
            if body_y != 0:
                # Keep the walking decision across the neutral-pose measurement.
                side_step_pending = True
                dog.translation("y", 0)
                body_y = 0.0
                time.sleep(2)
                continue  # Re-measure after returning the camera to neutral.
            speed = float(np.copysign(SIDE_STEP_SPEED, required_y))
            print(f"Side step: speed={speed:g}, duration={SIDE_STEP_SECONDS}s")
            try:
                dog.move_y(speed, SIDE_STEP_SECONDS)
            finally:
                dog.stop()
            side_steps += 1
            side_step_pending = False
            time.sleep(2)
            continue  # Correct the remaining error using a fresh observation.
        step = float(np.clip(-0.5 * x_offset, -3, 3))
        new_y = float(np.clip(body_y + step, -BODY_Y_LIMIT, BODY_Y_LIMIT))
        if new_y == body_y:
            raise RuntimeError("Body Y limit reached before centering tag")
        dog.translation("y", new_y)
        body_y = new_y
        time.sleep(2)
    else:
        raise TimeoutError("Could not center tag in 30 observations")
    print(f"Reached offset: {x_offset:.2f} mm")


def go_away(dog: XGO_DOG):
    dog.attitude("p", 0)
    dog.translation("z", 80)
    dog.turn_to(180)
    dog.move_x_by(200)
    dog.arm(120, -50)
    time.sleep(1)
    dog.claw(30)


def check_leg(dog, leg=3, *, full_range=False, amplitude=10.0,
                         step=2.0, settle=1.0, tolerance=5.0):
    """Test the leg; keep joint 1 commanded to -65 degrees while testing joint 2."""
    if leg not in (1, 2, 3, 4):
        raise ValueError("leg must be 1, 2, 3 or 4")
    log_path = Path.home() / "robotrio_ws" / f"leg_diag_{leg}.csv"
    motor_ids = (leg * 10 + 1, leg * 10 + 2, leg * 10 + 3)
    telemetry_order = (11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43)
    motor_indices = tuple(telemetry_order.index(motor_id) for motor_id in motor_ids)
    for name, value in (("amplitude", amplitude), ("step", step),
                        ("settle", settle), ("tolerance", tolerance)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be positive and finite")
    if step > 2 or settle < 1:
        raise ValueError("Use steps <= 2 degrees and settle >= 1 second")
    sdk = inspect.getmodule(type(dog))
    print(f"Motor SDK: {getattr(sdk, '__file__', 'unknown')}; "
          f"firmware={getattr(dog, 'version', 'unknown')}", flush=True)
    print(f"Motor telemetry mapping: {dict(zip(motor_ids, motor_indices))}", flush=True)
    params = getattr(sdk, "XGOparam", {})
    limits = params.get("MOTOR_LIMIT")
    if limits is None or len(limits) < 3:
        raise RuntimeError("Installed SDK does not expose MOTOR_LIMIT; no motion sent")
    if not str(getattr(dog, "version", "")).startswith(("M", "L")):
        raise RuntimeError("Expected a detected Mini/Lite model; no motion sent")
    bounds = []
    for limit in limits[:3]:
        low, high = map(float, limit)
        if not (math.isfinite(low) and math.isfinite(high) and low + 10 < high):
            raise ValueError("Invalid SDK motor limits")
        bounds.append((low + 5, high - 5))

    if not float(limits[0][0]) <= -65.0 <= float(limits[0][1]):
        raise ValueError("Joint 1 target -65 is outside SDK limits")

    def read_angles():
        angles = dog.read_motor()
        if len(angles) < 12:
            raise RuntimeError(f"Incomplete motor telemetry: {angles}")
        # Print the same response used for actual; do not issue another read.
        print(f"read_motor raw ({len(angles)}): {angles}", flush=True)
        print("Leg telemetry: " + ", ".join(
            f"{motor_id}[{i}]={float(angles[i]):.2f}"
            for motor_id, i in zip(motor_ids, motor_indices)
        ), flush=True)
        values = [float(angles[i]) for i in motor_indices]
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError("Non-finite motor telemetry")
        return values

    rows = []
    dog.stop()
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("w", newline="", encoding="utf-8") as log_file:
            writer = csv.writer(log_file)
            writer.writerow(("motor", "target", "actual"))
            log_file.flush()
            os.fsync(log_file.fileno())
            print(f"Leg diagnostic CSV: {log_path}", flush=True)
            time.sleep(1)
            initial = read_angles()
            for motor_id, start, (low, high) in zip(motor_ids, initial, bounds):
                if not low <= start <= high:
                    raise RuntimeError(f"Motor {motor_id}: initial {start} outside test bounds [{low}, {high}]")
            def position_first_joint(target):
                current = read_angles()[0]
                while abs(target - current) > 1e-8:
                    current += math.copysign(min(step, abs(target - current)), target - current)
                    dog.motor(motor_ids[0], current)
                    time.sleep(settle)
                    actual = read_angles()[0]
                    writer.writerow((motor_ids[0], current, actual))
                    log_file.flush()
                    os.fsync(log_file.fileno())
                    rows.append((motor_ids[0], current, actual, actual - current))
                    print(f"Preparation motor {motor_ids[0]}: target={current:.2f}, "
                          f"actual={actual:.2f}", flush=True)
                return read_angles()[0]

            for index, (motor_id, start, (low, high)) in enumerate(zip(motor_ids, initial, bounds)):
                if index == 1:
                    actual_first = position_first_joint(-65.0)
                    print(f"Testing motor {motor_id} with motor {motor_ids[0]} "
                          f"target=-65, actual={actual_first:.2f}", flush=True)
                    start = read_angles()[index]
                    if not low <= start <= high:
                        raise RuntimeError(f"Motor {motor_id}: invalid starting angle {start}")
                elif index == 2:
                    position_first_joint(initial[0])
                lower = low if full_range else max(low, start - amplitude)
                upper = high if full_range else min(high, start + amplitude)
                print(f"Motor {motor_id}: start={start:.2f}, test=[{lower:.2f}, {upper:.2f}]", flush=True)
                current = start
                for endpoint in (lower, start, upper, start):
                    while abs(endpoint - current) > 1e-8:
                        current += math.copysign(min(step, abs(endpoint - current)), endpoint - current)
                        dog.motor(motor_id, current)
                        time.sleep(settle)
                        actual = read_angles()[index]
                        error = actual - current
                        row = (motor_id, current, actual, error)
                        rows.append(row)
                        writer.writerow((motor_id, current, actual))
                        log_file.flush()
                        os.fsync(log_file.fileno())
                        print(f"Motor {motor_id}: target={current:.2f}, actual={actual:.2f}, error={error:+.2f} deg", flush=True)
                        if abs(error) > tolerance:
                            print(f"Motor {motor_id} did not reach target; continuing", flush=True)
            print("Sweep completed; all targets sent, including the starting angles.", flush=True)
            return rows
    finally:
        dog.stop()


def exec(dog: XGO_DOG, img_recv: ImageReceiver):
    dog.pace("slow")
    dog.translation("z", 60)
    dog.move_y(-6)
    time.sleep(2)
    dog.stop()
    print(dog.read_firmware())
    # check_leg(dog, leg=2, full_range=True)
    # dog.pace("slow")
    # dog.translation("z", 60)    
    # approach_tag_pair(dog, img_recv, tag_size_mm=100, stop_distance_mm=100)
    input("Подход к середине завершён. Нажмите Enter для завершения и сброса позы...")


def run(img_recv):
    dog = None
    try:
        dog = XGO_DOG(port="/dev/ttyAMA0", baud=115200)
        exec(dog, img_recv)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if dog is not None:
            dog.stop()
            dog.claw(30)
            dog.reset()



