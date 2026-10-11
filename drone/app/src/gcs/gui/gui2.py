import tkinter as tk

from .terminal import Terminal
from .manual_control import JoystickVis, KeyboardVis
from .colors import *
from .command_parser import CommandParser
from .telemetry import TelemetryVis
from .camera2 import CameraImage
from .mission_panel import MissionPanel
from .control_source_panel import ControlSourcePanel
from .grip_panel import GripPanel
from .main_panel import MainPanel

class GUI(tk.Tk):
    def __init__(self, *, keyboard=False):
        super().__init__()

        self.parser = CommandParser(self)

        self._joystick_demo = None
        self.title("RoboTrio.DRONE")
        self.geometry("1280x1024")
        self.minsize(100, 102)
        self.configure(bg=BACKGROUND)
        self.option_add("*Font", ("DejaVu Sans", 11))

        for index in range(20):
            self.columnconfigure(index, weight=1, uniform="main_columns")
        for index in range(16):
            self.rowconfigure(index, weight=1, uniform="main_rows")

        self.main_panel = MainPanel(self)
        self.main_panel.grid(
            row=0, column=0, columnspan=20,
            sticky="nsew", padx=8, pady=8,
        )

        self.control_source_panel = ControlSourcePanel(self)
        self.control_source_panel.grid(
            row=12, column=10, rowspan=1, columnspan=5,
            sticky="nsew", padx=8, pady=8,
        )

        self.control_source_panel2 = ControlSourcePanel(self)
        self.control_source_panel2.grid(
            row=9, column=0, rowspan=1, columnspan=5,
            sticky="nsew", padx=8, pady=8,
        )

        self.mission = MissionPanel(self)
        self.mission.grid(
            row=9, column=15, rowspan=7, columnspan=5,
            sticky="nsew", padx=8, pady=8,
        )

        self.camera = CameraImage(self)
        self.camera.grid(
            row=1, column=0, rowspan=8, columnspan=10,
            sticky="nsew", padx=8, pady=8,
        )

        self.camera2 = CameraImage(self)
        self.camera2.grid(
            row=1, column=10, rowspan=8, columnspan=10,
            sticky="nsew", padx=8, pady=8,
        )

        self.camera3 = CameraImage(self)
        self.camera3.grid(
            row=9, column=10, rowspan=3, columnspan=5,
            sticky="nsew", padx=8, pady=8,
        )

        self.grip_panel = GripPanel(self)
        self.grip_panel.grid(
            row=10, column=0, rowspan=2, columnspan=5,
            sticky="nsew", padx=8, pady=8,
        )

        self.telemetry = TelemetryVis(self)
        self.telemetry.grid(
            row=9, column=5, rowspan=3, columnspan=5,
            sticky="nsew", padx=8, pady=8,
        )

        if keyboard:
            self.keyboard_vis = KeyboardVis(self)
            self.control_panel = self.keyboard_vis
        else:
            self.joystick_vis = JoystickVis(self)
            self.control_panel = self.joystick_vis
        self.control_panel.grid(
            row=13, column=10, rowspan=3, columnspan=5,
            sticky="nsew", padx=8, pady=8,
        )

        self.terminal = Terminal(self, self._on_command)
        self.terminal.grid(row=12, column=0, rowspan=4, columnspan=10, sticky="nsew", padx=8, pady=8)

    def destroy(self):
        if self._joystick_demo is not None:
            self._joystick_demo.stop()
        super().destroy()

    def _on_command(self, cmd):
        self.parser.parse(cmd)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--keyboard", action="store_true",
        help="Управлять с клавиатуры"
    )
    args = parser.parse_args()
    GUI(keyboard=args.keyboard).mainloop()
