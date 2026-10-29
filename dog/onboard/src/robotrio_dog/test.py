#!/usr/bin/env python3

import inspect
import time


def conver2u8_fixed(data, limit, min_value=0):
    """Round to the nearest byte: zero in a symmetric range maps to 128."""
    if data is None or limit is None:
        return min_value
    low, high = limit if isinstance(limit, list) else (-limit, limit)
    if data >= high:
        return 255
    if data <= low:
        return min_value
    return round(255 * (data - low) / (high - low))


def main():
    from xgolib import XGO_DOG

    sdk = inspect.getmodule(XGO_DOG)
    original_converter = sdk.conver2u8
    sdk.conver2u8 = conver2u8_fixed
    dog = None
    try:
        dog = XGO_DOG(port="/dev/ttyAMA0", baud=115200)
        dog.stop()
        print("Исправление округления активно. Тест: 10 циклов claw(0) → claw(255).", flush=True)
        for cycle in range(1, 11):
            for position in (0, 255):
                print(f"Цикл {cycle}/10: claw({position})", flush=True)
                dog.claw(position)
                time.sleep(3)
                angles = dog.read_motor()
                if len(angles) != 15:
                    raise RuntimeError(f"Ожидалось 15 показаний моторов, получено: {angles}")
                print(f"Команда claw({position}); показание мотора 51: {angles[12]}°", flush=True)
        print("Тест завершён. Последняя команда захвату — claw(255).", flush=True)
    except KeyboardInterrupt:
        print("Тест прерван", flush=True)
        return 130
    finally:
        try:
            if dog is not None:
                try:
                    dog.stop()
                finally:
                    dog.ser.close()
        finally:
            sdk.conver2u8 = original_converter
    return 0


def camera():
    from .image_handlers import ImageReceiver
    from pathlib import Path
    from .logger_fabric import LoggerFabric
    import cv2
    

    dir = Path("/home/pi/robotrio_ws/imgs")
    dir.mkdir(parents=True, exist_ok=True)

    logger_fabric = LoggerFabric({"basic_level": "debug", "logs_dir": "~/robotrio_logs"})
    logger = logger_fabric.get_logger(name="imgs")

    ir = ImageReceiver(
        config={
            "log_level": "debug",
            "fx": 0,
            "fy": 0,
            "cx": 0,
            "cy": 0,
            "dist_coeffs": [
            0, 0, 0, 0, 0
            ]
        },
        logger_fabric=logger_fabric
    )

    time.sleep(1)

    for i in range(6):
        ts, img = ir.get_image()
        cv2.imwrite(dir / f"{ts}.jpg", img)
        print(f"Снимок {i + 1}/6: ts = {ts}")
        time.sleep(5)



if __name__ == "__main__":
    raise SystemExit(main())
