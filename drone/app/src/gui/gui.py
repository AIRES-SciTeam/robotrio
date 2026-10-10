import tkinter as tk

from .terminal import Terminal
from .manual_control import JoystickVis, KeyboardVis
from .colors import *
from .command_parser import CommandParser

class GUI(tk.Tk):
    def __init__(self, *, keyboard=False, demo_joystick=False):
        super().__init__()

        self.parser = CommandParser(self)

        self._joystick_demo = None
        self.title("RoboTrio.DRONE")
        self.geometry("1280x1024")
        self.minsize(640, 512)
        self.configure(bg=BACKGROUND)
        self.option_add("*Font", ("DejaVu Sans", 11))

        for index in range(20):
            self.columnconfigure(index, weight=1, uniform="main_columns")
            self.rowconfigure(index, weight=1, uniform="main_rows")

        if keyboard:
            self.keyboard_vis = KeyboardVis(self)
            self.control_panel = self.keyboard_vis
        else:
            self.joystick_vis = JoystickVis(self)
            self.control_panel = self.joystick_vis
        self.control_panel.grid(
            row=12, column=0, rowspan=4, columnspan=6,
            sticky="nsew", padx=8, pady=8,
        )
        
        self.terminal = Terminal(self, self._on_command)
        self.terminal.grid(row=16, column=0, rowspan=4, columnspan=20, sticky="nsew", padx=8, pady=8)

        if not keyboard and demo_joystick:
            from .joystick_demo import JoystickDemo

            self.title("RoboTrio.DRONE — ДЕМО джойстика")
            self._joystick_demo = JoystickDemo(self.joystick_vis)
            self._joystick_demo.start()

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
        "--demo-joystick", action="store_true",
        help="Анимировать индикаторы джойстика без подключения устройства",
    )
    parser.add_argument(
        "--keyboard", action="store_true",
        help="Управлять с клавиатуры"
    )
    args = parser.parse_args()
    GUI(keyboard=args.keyboard, demo_joystick=args.demo_joystick).mainloop()
