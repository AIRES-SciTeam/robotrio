import tkinter as tk
from datetime import datetime

from .colors import *


class TelemetryVis(tk.Frame):
    def __init__(self, parent):
        super().__init__(
            parent, bg=PANEL, bd=0,
            width=1, height=1,
            padx=0, pady=0,
            highlightthickness=2,
            highlightbackground=CYAN,
            highlightcolor=CYAN
        )

        self._pos = (
            tk.StringVar(value="-"),
            tk.StringVar(value="-"),
            tk.StringVar(value="-")
        )
        self._vel = (
            tk.StringVar(value="-"),
            tk.StringVar(value="-"),
            tk.StringVar(value="-")
        )
        self._q = (
            tk.StringVar(value="-"),
            tk.StringVar(value="-"),
            tk.StringVar(value="-")
        )

        self._received_at = tk.StringVar(self, value="Получено: —")

        self._configure_grid()
        self._create_header()
        self._create_body()
        self._create_footer()

    def _configure_grid(self):
        self.grid_propagate(False)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)
        self.rowconfigure(3, weight=1)
        self.rowconfigure(4, weight=0)

    def _create_header(self):
        tk.Label(
            self, text="Телеметрия дрона (NED, см)",
            bg=PANEL, fg=CYAN,
            padx=10, pady=2
        ).grid(row=0, column=0, sticky="w")

    def _telemetry_value(self, name, text_vars):
        frame = tk.Frame(
            self, bg=PANEL, bd=0,
            width=1, height=1,
            padx=0, pady=0
        )
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=0)
        for index in range(1, len(text_vars) + 1):
            frame.columnconfigure(index, weight=1, uniform="values")
        tk.Label(
            frame, bg=PANEL, fg=TEXT,
            text=name, width=12, anchor="w",
            font=("DejaVu Sans Mono", 9)
        ).grid(row=0, column=0, sticky="w")
        for index in range(len(text_vars)):
            tk.Label(
                frame, textvariable=text_vars[index], width=1,
                bg=PANEL, fg=TEXT,
                font=("DejaVu Sans", 9),
                highlightthickness=1,
                highlightbackground=CYAN,
                highlightcolor=CYAN,
                pady=6
            ).grid(row=0, column=index+1, sticky="ew") 
        return frame

    def _create_body(self):
        self._telemetry_value("Поз  (X,Y,Z)", self._pos).grid(row=1, column=0, sticky="nsew")
        self._telemetry_value("Скор (X,Y,Z)", self._vel).grid(row=2, column=0, sticky="nsew")
        self._telemetry_value("Углы (К,Т,Р)", self._q).grid(row=3, column=0, sticky="nsew")

    def _create_footer(self):
        tk.Label(
            self, textvariable=self._received_at,
            bg=PANEL, fg=TEXT, font=("DejaVu Sans Mono", 8), anchor="e",
        ).grid(row=4, column=0, sticky="ew", padx=8, pady=(2, 4))

    def update(self, pos, vel, q, *, received_at=None):
        """Update on the Tk thread; received_at is the reception datetime."""
        if received_at is None:
            received_at = datetime.now().astimezone()
        for idx in range(3):
            self._pos[idx].set(pos[idx])
            self._vel[idx].set(vel[idx])
            self._q[idx].set(q[idx])
        self._received_at.set(f"Получено: {received_at.isoformat(sep=' ', timespec='milliseconds')}")
