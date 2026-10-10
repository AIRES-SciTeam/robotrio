import tkinter as tk

from .colors import *


def _dim_color(color, background=PANEL):
    rgb = [round(int(color[i:i + 2], 16) * .35 +
                 int(background[i:i + 2], 16) * .65) for i in (1, 3, 5)]
    return "#" + "".join(f"{value:02x}" for value in rgb)

class _ControlPanel(tk.Frame):
    def __init__(self, parent, title, *, takefocus=False):
        super().__init__(
            parent, bg=PANEL, bd=0, takefocus=takefocus,
            width=1, height=1,
            highlightthickness=2, highlightbackground=CYAN,
            highlightcolor=MAGENTA,
        )
        self.enabled = True
        self._enabled_style = {}
        self.grid_propagate(False)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        header = tk.Frame(self, bg=PANEL)
        header.grid(row=0, column=0, sticky="ew", padx=6, pady=3)
        header.columnconfigure(0, weight=1)
        title_label = tk.Label(
            header, text=title, bg=PANEL, fg=CYAN,
            font=("DejaVu Sans", 10),
        )
        title_label.grid(row=0, column=0, sticky="w")
        sensitivity_box = tk.Frame(
            header, width=34, height=34, bg=PANEL,
            highlightbackground=MAGENTA, highlightthickness=1,
        )
        sensitivity_box.grid(row=0, column=1, sticky="e")
        sensitivity_box.grid_propagate(False)
        sensitivity_box.columnconfigure(0, weight=1)
        sensitivity_box.rowconfigure(0, weight=1)
        self.sensitivity_text = tk.StringVar(self, value="—%")
        sensitivity_label = tk.Label(
            sensitivity_box, textvariable=self.sensitivity_text,
            bg=PANEL, fg=MAGENTA, font=("DejaVu Sans", 8),
        )
        sensitivity_label.grid(row=0, column=0)
        self.body = tk.Frame(self, bg=PANEL)
        self.body.grid(row=1, column=0, sticky="nsew", padx=6, pady=(0, 6))
        if takefocus:
            for widget in (header, title_label, sensitivity_box, sensitivity_label, self.body):
                widget.bind("<Button-1>", lambda event: self.focus_set())

    def focus_set(self):
        if self.enabled:
            super().focus_set()

    def set_enabled(self, enabled: bool):
        """Включить ввод или приглушить панель и заблокировать её (GUI-поток)."""
        enabled = bool(enabled)
        if enabled == self.enabled:
            return
        if not enabled:
            if isinstance(self, KeyboardVis):
                self._reset()
        self.enabled = enabled

        def descendants(widget):
            yield widget
            for child in widget.winfo_children():
                yield from descendants(child)

        if not enabled:
            for widget in descendants(self):
                options = widget.keys()
                style = {}
                faded = {}
                for option in (
                    "background", "foreground", "activebackground",
                    "activeforeground", "highlightbackground", "highlightcolor",
                    "disabledforeground",
                ):
                    if option in options:
                        color = widget.cget(option)
                        style[option] = color
                        r, g, b = widget.winfo_rgb(color)
                        faded[option] = _dim_color(f"#{r // 257:02x}{g // 257:02x}{b // 257:02x}")
                style["takefocus"] = widget.cget("takefocus")
                faded["takefocus"] = False
                if isinstance(widget, tk.Button):
                    style["state"] = widget.cget("state")
                    faded["state"] = "disabled"
                    faded["disabledforeground"] = faded["foreground"]
                self._enabled_style[widget] = style
                widget.configure(**faded)
                if isinstance(widget, _AxisIndicator):
                    widget.set_enabled(False)
            focused = self.focus_get()
            if focused is not None and (focused is self or str(focused).startswith(str(self) + ".")):
                self.winfo_toplevel().focus_set()
        else:
            for widget, style in self._enabled_style.items():
                widget.configure(**style)
                if isinstance(widget, _AxisIndicator):
                    widget.set_enabled(True)
            self._enabled_style.clear()


class KeyboardVis(_ControlPanel):
    def __init__(self, parent):
        super().__init__(parent, "Ручное управление (Клавиатура)", takefocus=True)
        self.sensitivity = 1.0
        self.sensitivity_text.set(f"{self.sensitivity:.0%}")
        self.pressed_keys = set()
        self._pending_releases = {}
        self.buttons = {}
        captions = {
            "q": "Q · −\nРыскание", "e": "E · +\nРыскание",
            "w": "W · +\nТангаж", "s": "S · −\nТангаж",
            "a": "A · −\nКрен", "d": "D · +\nКрен",
            "space": "Space · +\nТяга", "shift": "Shift · −\nТяга",
        }
        for column in range(4):
            self.body.columnconfigure(column, weight=1, uniform="keys")
        for row in range(2):
            self.body.rowconfigure(row, weight=1, uniform="key_rows")
        for row, keys in enumerate((('q', 'w', 'e', 'space'), ('a', 's', 'd', 'shift'))):
            for column, key in enumerate(keys):
                button = tk.Button(
                    self.body, text=captions[key], width=8, height=2, padx=0, pady=2,
                    bg=PANEL, fg=TEXT, activebackground=PANEL,
                    activeforeground=TEXT, relief="flat", bd=0,
                    highlightthickness=1, highlightbackground=CYAN,
                    font=("DejaVu Sans", 9),
                    takefocus=False, command=self.focus_set,
                )
                button.grid(row=row, column=column, sticky="nsew", padx=2, pady=2)
                button.bind("<Button-1>", lambda event: self.focus_set())
                self.buttons[key] = button

        self.bind("<Button-1>", lambda event: self.focus_set())
        self.bind("<KeyPress>", self._key_down)
        self.bind("<KeyRelease>", self._key_up)
        self.bind("<FocusOut>", self._reset)

    def _key_down(self, event):
        if not self.enabled:
            return "break"
        key = event.keysym.lower()
        if key in tuple("0123456789"):
            self.sensitivity = 1.0 if key == "0" else int(key) / 10
            self.sensitivity_text.set(f"{self.sensitivity:.0%}")
            return "break"
        if self._button_name(key) not in self.buttons:
            return
        pending = self._pending_releases.pop(key, None)
        if pending is not None:
            self.after_cancel(pending)
        self.pressed_keys.add(key)
        self._paint_key(key, True)
        return "break"

    def _key_up(self, event):
        if not self.enabled:
            return "break"
        key = event.keysym.lower()
        if self._button_name(key) not in self.buttons:
            return
        if key not in self._pending_releases:
            self._pending_releases[key] = self.after_idle(self._release_key, key)
        return "break"

    def _release_key(self, key):
        self._pending_releases.pop(key, None)
        self.pressed_keys.discard(key)
        self._paint_key(key, False)

    @staticmethod
    def _button_name(key):
        return "shift" if key in ("shift_l", "shift_r") else key

    def _paint_key(self, key, pressed):
        if not self.enabled:
            return
        button_name = self._button_name(key)
        if button_name == "shift":
            pressed = bool(self.pressed_keys & {"shift_l", "shift_r"})
        self.buttons[button_name].configure(
            bg=CYAN if pressed else PANEL,
            fg=BACKGROUND if pressed else TEXT,
            activebackground=CYAN if pressed else PANEL,
            activeforeground=BACKGROUND if pressed else TEXT,
        )

    def _reset(self, event=None):
        for pending in self._pending_releases.values():
            self.after_cancel(pending)
        self._pending_releases.clear()
        self.pressed_keys.clear()
        for key in self.buttons:
            self._paint_key(key, False)

    def destroy(self):
        self._reset()
        super().destroy()

class _AxisIndicator(tk.Canvas):
    def __init__(self, parent, variable, minimum, maximum):
        super().__init__(
            parent, width=150, height=20, bg=PANEL,
            bd=0, highlightthickness=0,
        )
        self.variable = variable
        self.minimum = minimum
        self.maximum = maximum
        self.enabled = True
        self.bind("<Configure>", self._draw)
        self._trace = variable.trace_add("write", self._draw)

    def _draw(self, *_):
        self.delete("all")
        width = self.winfo_width()
        height = self.winfo_height()
        thumb_width = min(44, width - 2)
        if thumb_width <= 0:
            return
        value = self.variable.get()
        fraction = max(0, min(1, (value - self.minimum) / (self.maximum - self.minimum)))
        left = 1 + fraction * (width - thumb_width - 2)
        accent = CYAN if self.enabled else _dim_color(CYAN)
        background = BACKGROUND if self.enabled else _dim_color(BACKGROUND)
        self.create_rectangle(1, 1, width - 1, height - 1, fill=background, outline=accent)
        self.create_line(width / 2, 3, width / 2, height - 3, fill=accent)
        self.create_rectangle(left, 1, left + thumb_width, height - 1, fill=accent, outline=accent)
        self.create_text(
            left + thumb_width / 2, height / 2, text=str(value),
            fill=background, font=("DejaVu Sans Mono", 9, "bold"),
        )

    def set_enabled(self, enabled):
        self.enabled = enabled
        self._draw()

    def destroy(self):
        self.variable.trace_remove("write", self._trace)
        super().destroy()


class JoystickVis(_ControlPanel):
    def __init__(self, parent):
        super().__init__(parent, "Ручное управление (Джойстик)")
        self.body.columnconfigure(1, weight=1)

        self.axis_values = {}
        self.axis_sliders = {}
        for row, (name, title, minimum, initial) in enumerate((
            ("roll", "Крен", -1000, 0),
            ("pitch", "Тангаж", -1000, 0),
            ("yaw", "Рыскание", -1000, 0),
            ("throttle", "Газ", 0, 500),
        )):
            self.body.rowconfigure(row, weight=1, uniform="axis_rows")
            tk.Label(
                self.body, text=title, bg=PANEL, fg=TEXT,
                font=("DejaVu Sans", 9),
            ).grid(row=row, column=0, sticky="w", padx=(6, 4), pady=1)
            value = tk.IntVar(self, value=initial)
            slider = _AxisIndicator(self.body, value, minimum, 1000)
            slider.grid(
                row=row, column=1, sticky="ew",
                padx=(0, 6), pady=(0, 2),
            )
            self.axis_values[name] = value
            self.axis_sliders[name] = slider
