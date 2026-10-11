"""Synthetic camera streams without hardware, ROS or PX4.

Run from the repository root:
    python3 drone/app/src/gcs/gui/camera_demo.py
"""

import math
from pathlib import Path
import sys
import threading
import time

from PIL import Image, ImageDraw

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from gcs.gui.gui2 import GUI


def make_frame(size, elapsed, *, front=False):
    """Different aspect ratios and moving targets make switching visible."""
    width, height = size
    background = "#17243B" if front else "#102C22"
    accent = "#FF73CE" if front else "#36E4DA"
    frame = Image.new("RGB", size, background)
    draw = ImageDraw.Draw(frame)
    for x in range(0, width, 40):
        draw.line((x, 0, x, height), fill="#334454")
    for y in range(0, height, 40):
        draw.line((0, y, width, y), fill="#334454")
    x = width / 2 + math.sin(elapsed) * width / 4
    y = height / 2 + math.cos(elapsed * 0.7) * height / 4
    draw.ellipse((x - 24, y - 24, x + 24, y + 24), outline=accent, width=4)
    draw.line((width / 2 - 15, height / 2, width / 2 + 15, height / 2), fill="white", width=2)
    draw.line((width / 2, height / 2 - 15, width / 2, height / 2 + 15), fill="white", width=2)
    label = "FRONT" if front else "DOWN"
    draw.text((12, 12), f"{label} | {width}x{height} | {elapsed:.1f} s", fill="white")
    return frame


class CameraDemo:
    """Simulate frame reception in a worker; Tk is only accessed on its thread."""

    def __init__(self, gui):
        self.viewer = gui.camera
        self._stop_event = threading.Event()
        self.viewer.add_camera("Нижняя")
        self.viewer.add_camera("Передняя")
        self.viewer.add_camera("Без сигнала")
        gui.terminal.write_parts((
            "Демо камер: выберите Нижняя / Передняя / Без сигнала.\n"
            "Тестовые изображения обновляются с частотой около 20 Гц.\n",
            "info",
        ))
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        started = time.monotonic()
        while not self._stop_event.is_set():
            elapsed = time.monotonic() - started
            self.viewer.set_frame("Нижняя", make_frame((640, 480), elapsed))
            self.viewer.set_frame("Передняя", make_frame((640, 360), elapsed, front=True))
            self._stop_event.wait(0.05)

    def stop(self):
        self._stop_event.set()
        self._thread.join()


class CameraDemoGUI(GUI):
    def __init__(self):
        super().__init__()
        self._camera_demo = CameraDemo(self)

    def destroy(self):
        self._camera_demo.stop()
        super().destroy()


def main():
    gui = CameraDemoGUI()
    try:
        gui.mainloop()
    finally:
        gui._camera_demo.stop()


if __name__ == "__main__":
    main()
