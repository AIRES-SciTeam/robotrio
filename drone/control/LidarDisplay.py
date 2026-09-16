#!/usr/bin/env python3
import argparse
import math
import time

import tkinter as tk
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


class LidarDisplay(Node):
    def __init__(self, topic):
        super().__init__("LidarSisplay")
        self.last_received = None
        self.distance = None
        self.subscription = self.create_subscription(
            Range, topic, self.on_range, qos_profile_sensor_data
        )

    def on_range(self, message):
        self.last_received = time.monotonic()
        distance = message.range
        self.distance = (
            distance
            if math.isfinite(distance)
            and message.min_range <= distance <= message.max_range
            else None
        )

    def display_text(self):
        if self.last_received is None:
            return "Ожидание данных", (190, 195, 205)
        if time.monotonic() - self.last_received > 2.0:
            return "Нет данных", (255, 195, 100)
        if self.distance is None:
            return "Вне диапазона", (255, 195, 100)
        return f"{self.distance:.2f} м".replace(".", ","), (110, 230, 160)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/x500/lidar/range")
    args = parser.parse_args()
    rclpy.init(args=[])
    node = None
    root = None
    try:
        node = LidarDisplay(args.topic)
        root = tk.Tk()
        root.title("Лидар — расстояние")
        root.geometry("480x190")
        root.configure(bg="#1e2228")
        root.resizable(False, False)
        root.bind("<Escape>", lambda event: root.destroy())
        tk.Label(root, text="Расстояние вдоль луча", font=("DejaVu Sans", 16),
                 bg="#1e2228", fg="#dce1eb").pack(pady=(16, 8))
        value = tk.Label(root, text="Ожидание данных", font=("DejaVu Sans", 28),
                         bg="#1e2228", fg="#bec3cd")
        value.pack(expand=True)
        tk.Label(root, text=args.topic, font=("DejaVu Sans", 10),
                 bg="#1e2228", fg="#96a0af").pack(pady=(8, 16))

        def update():
            if not rclpy.ok():
                root.destroy()
                return
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except ExternalShutdownException:
                root.destroy()
                return
            text, color = node.display_text()
            value.configure(text=text, fg="#{:02x}{:02x}{:02x}".format(*color))
            root.after(33, update)

        root.after(0, update)
        root.mainloop()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
