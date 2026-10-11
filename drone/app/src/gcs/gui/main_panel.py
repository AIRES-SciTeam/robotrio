import tkinter as tk

from .colors import PANEL, TEXT, CYAN, MAGENTA, GREEN, RED, YELLOW


class MainPanel(tk.Frame):
    FIELDS = {
        "drone_id": "Дрон",
        "armed": "Взведённость",
        "flight_mode": "Режим",
        "onboard_connected": "Борт",
        "px4_connected": "PX4",
        "landed": "Положение",
        "battery_percent": "Батарея",
    }

    def __init__(self, parent):
        super().__init__(
            parent, bg=PANEL, width=1, height=1,
            highlightthickness=1, highlightbackground=CYAN,
            highlightcolor=MAGENTA,
        )
        self._labels = {}
        self._configure_grid()
        self._create_body()

    def _configure_grid(self):
        self.grid_propagate(False)
        self.rowconfigure(0, weight=1)
        for index in range(len(self.FIELDS)):
            self.columnconfigure(index * 2, weight=1)

    def _create_body(self):
        for index, (key, caption) in enumerate(self.FIELDS.items()):
            if index:
                tk.Frame(self, bg=CYAN, width=1).grid(
                    row=0, column=index * 2 - 1, sticky="ns",
                )
            label = tk.Label(
                self, text=f"{caption}: —", bg=PANEL, fg=TEXT,
                font=("DejaVu Sans", 10), anchor="w",
            )
            label.grid(row=0, column=index * 2, sticky="ew", padx=8, pady=4)
            self._labels[key] = label

    @staticmethod
    def _format_value(key, value):
        if value is None:
            return "—", TEXT
        if key in ("armed", "landed", "onboard_connected", "px4_connected"):
            if not isinstance(value, bool):
                raise ValueError(f"{key} must be bool or None")
            if key == "armed":
                return ("взведён", YELLOW) if value else ("не взведён", TEXT)
            if key == "landed":
                return ("на земле", TEXT) if value else ("в воздухе", YELLOW)
            return ("связь", GREEN) if value else ("нет связи", RED)
        if key == "battery_percent":
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 <= value <= 100:
                raise ValueError("battery_percent must be between 0 and 100")
            return f"{value:.0f}%", TEXT
        return str(value), TEXT

    def set_status(self, **values):
        """Update supplied fields on the Tk thread; None clears a field."""
        unknown = values.keys() - self.FIELDS.keys()
        if unknown:
            raise KeyError(f"Unknown status fields: {sorted(unknown)}")
        formatted = {key: self._format_value(key, value) for key, value in values.items()}
        for key, (text, color) in formatted.items():
            self._labels[key].configure(text=f"{self.FIELDS[key]}: {text}", fg=color)
