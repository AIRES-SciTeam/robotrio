"""pygame → GUI demo, without PX4 or ROS.

Run from the repository root:
    python3 drone/app/src/gcs/gui/joystick_demo.py

Mappings and normalization match manual_controller/gamepad_controller.py.
"""

import math
import os
from pathlib import Path
import sys
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

import pygame
from pygame._sdl2 import controller as sdl_controller

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from gcs.gui.gui import GUI


class GamepadReader:
    """Nonblocking input source; does not access GUI widgets."""

    def __init__(self):
        pygame.display.init()
        pygame.joystick.init()
        sdl_controller.init()
        self.device = None
        self.controller = None
        self.dpad_buttons = {}
        self.sensitivity = 1.0
        self.next_connect = 0.0

    def disconnect(self):
        for device in (self.controller, self.device):
            if device is not None:
                try:
                    device.quit()
                except pygame.error:
                    pass
        self.device = self.controller = None
        self.dpad_buttons.clear()

    def connect(self):
        self.next_connect = time.monotonic() + 1.0
        for index in range(pygame.joystick.get_count()):
            try:
                self.device = pygame.joystick.Joystick(index)
                self.device.init()
                if self.device.get_numaxes() < 6:
                    self.disconnect()
                    continue
                if sdl_controller.is_controller(index):
                    self.controller = sdl_controller.Controller(index)
                    mapping = self.controller.get_mapping()
                    for name, direction in {
                        "dpleft": (-1, 0), "dpright": (1, 0),
                        "dpdown": (0, -1), "dpup": (0, 1),
                    }.items():
                        binding = mapping.get(name, "")
                        if binding.startswith("b") and binding[1:].isdigit():
                            self.dpad_buttons[int(binding[1:])] = direction
                return
            except pygame.error:
                self.disconnect()

    def change_sensitivity(self, direction):
        horizontal, vertical = direction
        if vertical:
            self.sensitivity = 1.0 if vertical > 0 else 0.1
        elif horizontal:
            self.sensitivity = round(
                max(0.1, min(1.0, self.sensitivity + horizontal * 0.1)), 1
            )

    def read(self):
        """Return (latest axes or None, button actions)."""
        events = pygame.event.get()
        for event in events:
            if (event.type == pygame.JOYDEVICEREMOVED and self.device is not None
                    and event.instance_id == self.device.get_instance_id()):
                self.disconnect()
            elif event.type == pygame.JOYDEVICEADDED:
                self.next_connect = 0.0
        if self.device is None:
            if time.monotonic() >= self.next_connect:
                self.connect()
            # Do not replay button events from before connection.
            return None, []

        actions = []
        for event in events:
            if getattr(event, "instance_id", None) != self.device.get_instance_id():
                continue
            if event.type == pygame.JOYHATMOTION and event.hat == 0:
                self.change_sensitivity(event.value)
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button in self.dpad_buttons:
                    self.change_sensitivity(self.dpad_buttons[event.button])
                elif event.button in (0, 2):
                    actions.append("Arm / Disarm" if event.button == 0 else "Grip / Release")

        scale = 0.13 if self.sensitivity == 0.1 else self.sensitivity

        def axis(index, invert=False):
            value = self.device.get_axis(index)
            normalized = int((-value if invert else value) * 1000) if abs(value) >= 0.1 else 0
            return int(normalized * scale)

        def trigger(index):
            value = max(-1.0, min(1.0, self.device.get_axis(index)))
            return int(500 * (1 + math.copysign(value ** 2, value))) if value + 1 > 0.1 else 0

        return {
            "roll": axis(0), "pitch": axis(1, invert=True), "yaw": axis(2),
            "throttle": 500 + int((trigger(5) - trigger(4)) * scale / 2),
        }, actions

    def stop(self):
        self.disconnect()
        sdl_controller.quit()
        pygame.quit()


class JoystickDemo:
    """Bridge input snapshots to Tk on its main thread at approximately 50 Hz."""

    def __init__(self, gui):
        self.gui = gui
        self.reader = GamepadReader()
        self.after_id = None
        self.last_device = None
        self.gui._joystick_demo = self
        self.log("Ожидание джойстика с 6 осями. Демонстрация: команды только отображаются.")
        self.log("Левый стик: крен/тангаж; правый X: рыскание; RT/LT: газ.")
        self.log("A: Arm/Disarm; X: Grip/Release; крестовина: ←/→ ±10%, ↓ 10%, ↑ 100%.")
        self.tick()

    def log(self, message):
        self.gui.terminal.write_parts((message + "\n", "info"))

    def tick(self):
        try:
            values, actions = self.reader.read()
        except pygame.error as exc:
            self.log(f"Ошибка чтения: {exc}")
            self.reader.disconnect()
            self.reader.next_connect = time.monotonic() + 1.0
            values, actions = None, []

        device = self.reader.device
        identity = device.get_instance_id() if device is not None else None
        if identity != self.last_device:
            self.log(f"Подключён: {device.get_name()}" if device else "Джойстик отключён. Ожидание подключения.")
            self.last_device = identity

        panel = self.gui.joystick_vis
        active = values is not None and panel.enabled
        shown = values if active else {"roll": 0, "pitch": 0, "yaw": 0, "throttle": 500}
        for name, value in shown.items():
            panel.axis_values[name].set(value)
        panel.sensitivity_text.set(f"{self.reader.sensitivity:.0%}" if values is not None else "—%")
        if active:
            for action in actions:
                self.log(f"Кнопка: {action}")
        self.after_id = self.gui.after(20, self.tick)

    def stop(self):
        if self.after_id is not None:
            self.gui.after_cancel(self.after_id)
            self.after_id = None
        self.reader.stop()


def main():
    gui = GUI()
    try:
        JoystickDemo(gui)
        gui.mainloop()
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
