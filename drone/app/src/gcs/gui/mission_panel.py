import tkinter as tk

from .colors import BACKGROUND, PANEL, TEXT, CYAN, MAGENTA, GREEN, RED, YELLOW


class MissionPanel(tk.Frame):
    def __init__(self, parent):
        super().__init__(
            parent, bg=PANEL, bd=0, width=1, height=1,
            highlightthickness=2, highlightbackground=CYAN,
            highlightcolor=MAGENTA,
        )
        self.buttons = {}
        self.mode = "planning"

        self._configure_grid()
        self._create_header()
        self._create_controls()
        self._create_body()
        self._update_mode()

    def _configure_grid(self):
        self.grid_propagate(False)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

    def _create_header(self):
        tk.Label(
            self, text="Миссия", bg=PANEL, fg=CYAN,
            padx=10, pady=2,
        ).grid(row=0, column=0, sticky="w")

    def _create_controls(self):
        controls = tk.Frame(self, bg=PANEL)
        controls.grid(row=1, column=0, sticky="ew", padx=6, pady=4)
        for column in range(5):
            controls.columnconfigure(column, weight=1, uniform="mission_buttons")

        self.mode_button = tk.Button(
            controls, bg=PANEL, fg=CYAN,
            activebackground=CYAN, activeforeground=BACKGROUND,
            highlightbackground=CYAN, highlightcolor=MAGENTA,
            highlightthickness=1, relief="flat", bd=0,
            font=("DejaVu Sans", 10), pady=5,
            command=self._toggle_mode,
        )
        self.mode_button.grid(row=0, column=0, columnspan=5, sticky="ew", padx=2, pady=2)

        for index, (name, text, color, event) in enumerate((
            ("start", "Старт", GREEN, "<<MissionStart>>"),
            ("stop", "Стоп", RED, "<<MissionStop>>"),
            ("pause", "Пауза", YELLOW, "<<MissionPause>>"),
            ("prev", "Пред", CYAN, "<<MissionPrev>>"),            
            ("next", "След", CYAN, "<<MissionNext>>"),
            ("add_block", "Добавить блок", CYAN, "<<MissionAddBlock>>"),
        )):
            button = tk.Button(
                controls, text=text, bg=PANEL, fg=TEXT,
                activebackground=color, activeforeground=BACKGROUND,
                highlightbackground=color, highlightcolor=MAGENTA,
                highlightthickness=1, relief="flat", bd=0,
                font=("DejaVu Sans", 10), pady=5,
                width=1, anchor="center", justify="center",
                disabledforeground="#667080",
                command=lambda event=event: self.event_generate(event),
            )
            button.grid(
                row=1 + index // 5, column=index % 5,
                columnspan=5 if name == "add_block" else 1,
                sticky="ew", padx=2, pady=2,
            )
            self.buttons[name] = button

    def _toggle_mode(self):
        self.mode = "execution" if self.mode == "planning" else "planning"
        self._update_mode()

    def _update_mode(self):
        planning = self.mode == "planning"
        self.mode_button.configure(
            text="Режим: Планирование" if planning else "Режим: Исполнение",
        )
        for name, enabled, color in (
            ("start", not planning, GREEN),
            ("add_block", planning, CYAN),
        ):
            self.buttons[name].configure(
                state="normal" if enabled else "disabled",
                highlightbackground=color if enabled else "#667080",
                takefocus=enabled,
            )

    def _create_body(self):
        self.blocks_frame = tk.Frame(
            self, bg=BACKGROUND, width=1, height=1,
            highlightthickness=1, highlightbackground=CYAN,
            highlightcolor=MAGENTA,
        )
        self.blocks_frame.grid(
            row=2, column=0, sticky="nsew", padx=8, pady=(4, 8),
        )
        self.blocks_frame.columnconfigure(0, weight=1)
