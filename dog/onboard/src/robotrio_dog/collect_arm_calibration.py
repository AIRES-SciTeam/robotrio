#!/usr/bin/env python3
"""Interactively position the XGO arm and record camera-to-arm correspondences."""

import argparse
import csv
import math
import time
from pathlib import Path

FIELDS = ["camera_x_mm", "camera_y_mm", "camera_z_mm", "arm_x", "arm_z"]


def coordinates(text, count):
    values = [float(value) for value in text.replace(",", " ").split()]
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f"Нужно {count} конечных чисел через пробел")
    return values


def measure_tag(receiver, tag_id):
    previous = receiver.get_frame()
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        frame = receiver.get_frame()
        if frame is None or frame is previous:
            time.sleep(0.02)
            continue
        previous = frame
        tags = [tag for tag in receiver.get_poses(frame) if tag.tag_id == tag_id]
        if len(tags) > 1:
            raise ValueError("Видно несколько тэгов с этим ID; оставьте видимым только калибровочный")
        if tags and tags[0].pose_t is not None:
            position = [float(value) * 1000 for value in tags[0].pose_t.reshape(3)]
            if all(math.isfinite(value) for value in position) and position[2] > 0:
                return position
        time.sleep(0.02)
    raise ValueError(f"Нет свежей позы тэга ID={tag_id}; откройте его для камеры")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("arm_calibration.csv"))
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--stream-port", type=int, default=8080)
    args = parser.parse_args()
    if args.output.exists() and args.output.stat().st_size:
        with args.output.open(newline="", encoding="utf-8") as source:
            if next(csv.reader(source), None) != FIELDS:
                parser.error("CSV имеет другой формат; выберите другой --output")
    from xgolib import XGO_DOG
    from robotrio_dog.hands import ImageReceiver

    args.output.parent.mkdir(parents=True, exist_ok=True)
    dog = XGO_DOG(port=args.port, baud=args.baud)
    receiver = None
    arm = None
    try:
        dog.stop()
        dog.attitude("p", 22)
        time.sleep(0.5)
        receiver = ImageReceiver()
        receiver.body_pitch = 22
        receiver.start_stream(args.stream_port)
        print("Корпус наклонён: pitch=22°. Камера: tag36h11, размер тэга 27 мм.")
        try:
            print(f"Тэг найден: {measure_tag(receiver, args.tag_id)} мм")
        except ValueError as e:
            print(e)
        print("Не запускайте одновременно другое приложение, управляющее тем же UART.")
        print("look — найти тэг и вывести его координаты камеры в мм.")
        print("X Z — немедленно отправить dog.arm(x, z); повторяйте для подбора.")
        print("claw N — задать раскрытие клешни (0..255).")
        print("save — измерить тэг камерой и сохранить пару после совмещения клешни с его центром.")
        print("q — выход. Десятичный разделитель — точка.")
        print("При save тэг должен быть виден; старые координаты не используются.")
        with args.output.open("a", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            if output.tell() == 0:
                writer.writerow(FIELDS)
                output.flush()
            while True:
                text = input("> ").strip()
                if text.lower() in ("q", "quit", "exit"):
                    break
                if not text:
                    continue
                try:
                    command, *parts = text.split()
                    if command.lower() == "look":
                        print(f"Точка камеры: {measure_tag(receiver, args.tag_id)} мм")
                    elif command.lower() == "claw":
                        value, = coordinates(" ".join(parts), 1)
                        if not value.is_integer() or not 0 <= value <= 255:
                            raise ValueError("claw: целое число 0..255")
                        dog.claw(int(value))
                        print(f"Отправлено claw({int(value)})")
                    elif command.lower() == "save":
                        if arm is None:
                            raise ValueError("Сначала подберите X Z клешни")
                        camera = measure_tag(receiver, args.tag_id)
                        writer.writerow(camera + arm)
                        output.flush()
                        print(f"Сохранено: камера={camera}, команда клешни={arm}")
                    else:
                        target = coordinates(text, 2)
                        dog.arm(*target)
                        arm = target
                        print(f"Отправлено arm({arm[0]}, {arm[1]}). Проверьте положение; save — записать пару.")
                except ValueError as e:
                    print(f"Ошибка ввода: {e}")
    except (KeyboardInterrupt, EOFError):
        print("\nПодбор завершён")
    finally:
        try:
            dog.stop()
        finally:
            try:
                if receiver is not None:
                    receiver.close()
            finally:
                dog.ser.close()
    print(f"Данные: {args.output.resolve()}")


if __name__ == "__main__":
    main()
