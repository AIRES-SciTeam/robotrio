import tkinter as tk

from .colors import BACKGROUND, PANEL, TEXT, CYAN, MAGENTA


class GripPanel(tk.Frame):
    def __init__(self, parent):
        super().__init__(
            parent, bg=PANEL, bd=0, width=1, height=1,
            highlightthickness=2, highlightbackground=CYAN,
            highlightcolor=MAGENTA,
        )
        self._holding = None
        self._tag = tk.StringVar(self, value="—")

        self._configure_grid()
        self._create_header()
        self._create_body()

    def _configure_grid(self):
        self.grid_propagate(False)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

    def _create_header(self):
        tk.Label(
            self, text="Захват", bg=PANEL, fg=CYAN, padx=10, pady=2,
        ).grid(row=0, column=0, sticky="w")

    def _create_body(self):
        body = tk.Frame(self, bg=PANEL)
        body.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 6))
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)
        self.action_button = tk.Button(
            body, text="Взять / Отпустить", state="disabled",
            command=self._request_action,
            bg=PANEL, fg=TEXT, disabledforeground="#667080",
            activebackground=CYAN, activeforeground=BACKGROUND,
            highlightthickness=1, highlightbackground=CYAN,
            highlightcolor=MAGENTA, relief="flat", bd=0,
            font=("DejaVu Sans", 10), pady=4,
        )
        self.action_button.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        tk.Label(body, text="Тег:", bg=PANEL, fg=CYAN).grid(row=0, column=1)
        tk.Label(
            body, textvariable=self._tag, bg=PANEL, fg=TEXT,
            font=("DejaVu Sans Mono", 10), width=5, anchor="e",
        ).grid(row=0, column=2)

    def set_state(self, holding, tag_id=None):
        """Display confirmed onboard state on the Tk thread; None is unknown."""
        if holding is not None and not isinstance(holding, bool):
            raise ValueError("holding must be True, False or None")
        self._holding = holding
        self._tag.set(str(tag_id) if holding and tag_id is not None else "—")
        self.action_button.configure(
            text=("Взять / Отпустить" if holding is None
                  else "Отпустить" if holding else "Взять"),
            state="disabled" if holding is None else "normal",
        )

    def _request_action(self):
        if self._holding is not None:
            self.event_generate("<<GripRelease>>" if self._holding else "<<GripTake>>")
