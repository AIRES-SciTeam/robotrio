import tkinter as tk

from .colors import *

class ConnQuality(tk.Frame):
    def __init__(self, parent):
        super().__init__(
            parent, bg=PANEL, bd=0,
            padx=0, pady=0,
            highlightthickness=2,
            highlightbackground=CYAN,
            highlightcolor=CYAN
        )
        self._RTT = 