import tkinter as tk

from .colors import PANEL, TEXT, CYAN, MAGENTA


class ControlSourcePanel(tk.Frame):
    def __init__(self, parent):
        super().__init__(
            parent, bg=PANEL, bd=0, width=1, height=1,
            highlightthickness=2, highlightbackground=CYAN,
            highlightcolor=MAGENTA,
        )
        self._source = tk.StringVar(self, value="—")

        self._configure_grid()
        self._create_header()
        self._create_body()

    def _configure_grid(self):
        self.grid_propagate(False)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

    def _create_header(self):
        tk.Label(
            self, text="Источник управления:", bg=PANEL, fg=CYAN,
            font=("DejaVu Sans", 10),
            padx=10, pady=2,
        ).grid(row=0, column=0, sticky="w")

    def _create_body(self):
        tk.Label(
            self, textvariable=self._source, bg=PANEL, fg=TEXT,
            font=("DejaVu Sans Mono", 10, "bold"), width=7, anchor="e",
        ).grid(row=0, column=1, sticky="e", padx=(0, 8), pady=2)

    def set_source(self, source):
        if source not in (None, "GCS", "onboard"):
            raise ValueError("Control source must be 'GCS', 'onboard' or None")
        self._source.set(source if source is not None else "—")
