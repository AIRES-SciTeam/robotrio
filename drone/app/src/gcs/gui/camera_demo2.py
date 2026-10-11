"""Three animated camera feeds for gui2.

Run: python3 drone/app/src/gcs/gui/camera_demo2.py
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
from gcs.gui.camera_demo import make_frame


def make_side_frame(elapsed):
    frame = Image.new("RGB", (480, 360), "#342015")
    draw = ImageDraw.Draw(frame)
    for x in range(0, 480, 40):
        draw.line((x, 0, x, 360), fill="#62402A")
    for y in range(0, 360, 40):
        draw.line((0, y, 480, y), fill="#62402A")
    x = 240 + math.sin(elapsed * 1.3) * 150
    y = 180 + math.cos(elapsed) * 90
    draw.rectangle((x - 25, y - 25, x + 25, y + 25), fill="#FFD580")
    draw.text((12, 12), f"SIDE | 480x360 | {elapsed:.1f} s", fill="white")
    return frame


class CameraDemo:
    def __init__(self, gui):
        self._feeds = (
            (gui.camera, "Нижняя", lambda t: make_frame((640, 480), t)),
            (gui.camera2, "Передняя", lambda t: make_frame((640, 360), t, front=True)),
            (gui.camera3, "Боковая", make_side_frame),
        )
        self._stop_event = threading.Event()
        for viewer, name, _ in self._feeds:
            viewer.add_camera(name)
            viewer.select_camera(name)
        gui.terminal.write_parts((
            "Демо GUI2: слева — нижняя камера (зелёная), справа — передняя "
            "(синяя), в малом окне — боковая (оранжевая).\n"
            "Все три потока синтетические, обновляются одновременно.\n",
            "info",
        ))
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        started = time.monotonic()
        while not self._stop_event.is_set():
            elapsed = time.monotonic() - started
            for viewer, name, generate in self._feeds:
                viewer.set_frame(name, generate(elapsed))
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
