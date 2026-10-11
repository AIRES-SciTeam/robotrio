import tkinter as tk
from tkinter import ttk
import threading

from PIL import Image, ImageOps, ImageTk

from .colors import BACKGROUND, PANEL, TEXT, CYAN, MAGENTA


class CameraImage(tk.Frame):
    def __init__(self, parent):
        super().__init__(
            parent, bg=PANEL, width=1, height=1,
            highlightthickness=2, highlightbackground=CYAN,
            highlightcolor=MAGENTA,
        )
        self._lock = threading.Lock()
        self._frames = {}
        self._closed = False
        self._photo = None
        self._rendered = None

        self._configure_grid()
        self._create_header()
        self._configure_selector_style()
        self._create_selector()
        self._create_body()
        self._after_id = self.after(50, self._refresh)

    def _configure_grid(self):
        self.grid_propagate(False)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

    def _create_header(self):
        header = tk.Frame(self, bg=PANEL)
        header.grid(row=0, column=0, sticky="ew", padx=6, pady=4)
        tk.Label(header, text="Камера", bg=PANEL, fg=CYAN).grid(
            row=0, column=0, padx=(0, 8),
        )
        
    def _configure_selector_style(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        muted = "#667080"
        style.configure(
            "Camera.TCombobox",
            fieldbackground=PANEL, background=PANEL, foreground=TEXT,
            arrowcolor=CYAN, bordercolor=CYAN,
            lightcolor=PANEL, darkcolor=PANEL,
            selectbackground=PANEL, selectforeground=TEXT,
            padding=5, arrowsize=14,
        )
        style.map(
            "Camera.TCombobox",
            fieldbackground=[("disabled", PANEL), ("readonly", PANEL)],
            background=[("disabled", PANEL), ("active", BACKGROUND)],
            foreground=[("disabled", muted), ("readonly", TEXT)],
            arrowcolor=[("disabled", muted), ("active", MAGENTA)],
            bordercolor=[("disabled", muted), ("focus", MAGENTA), ("active", MAGENTA)],
            selectbackground=[("readonly", PANEL)],
            selectforeground=[("disabled", muted), ("readonly", TEXT)],
        )

    def _create_selector(self):
        self.selected_camera = tk.StringVar(self, value="")
        self.selector = ttk.Combobox(
            self, textvariable=self.selected_camera, values=(),
            state="disabled", width=12, style="Camera.TCombobox",
            font=("DejaVu Sans", 10),
        )
        for option, value in {
            "background": PANEL, "foreground": TEXT,
            "selectBackground": CYAN, "selectForeground": BACKGROUND,
            "highlightBackground": CYAN, "highlightColor": MAGENTA,
            "highlightThickness": 1, "borderWidth": 0,
            "font": "{DejaVu Sans} 10",
        }.items():
            self.option_add(f"*{self.selector.winfo_name()}*Listbox.{option}", value)
        self.selector.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 6))
        self.selector.bind("<<ComboboxSelected>>", self._draw)

    def _create_body(self):
        self.canvas = tk.Canvas(
            self, bg=BACKGROUND, width=1, height=1, highlightthickness=0,
        )
        self.canvas.grid(row=2, column=0, sticky="nsew", padx=6, pady=(0, 6))
        self._image_item = self.canvas.create_image(0, 0, anchor="center")
        self._message_item = self.canvas.create_text(
            0, 0, text="Нет камер", fill=TEXT, anchor="center",
        )
        self.canvas.bind("<Configure>", self._draw)

    def add_camera(self, name):
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Camera name must be a non-empty string")
        with self._lock:
            if self._closed:
                raise RuntimeError("Camera viewer is closed")
            if name in self._frames:
                return
            self._frames[name] = None
            names = tuple(self._frames)
        self.selector.configure(values=names, state="readonly")
        if not self.selected_camera.get():
            self.selected_camera.set(name)
        self._draw()

    def set_frame(self, camera, frame, *, bgr=True):
        if camera not in self._frames:
            raise KeyError(f"Unknown camera: {camera}")
        if frame is None:
            image = None
        elif isinstance(frame, Image.Image):
            image = frame.convert("RGB").copy()
        else:
            if bgr and frame.ndim == 3 and frame.shape[2] == 3:
                frame = frame[:, :, ::-1]
            image = Image.fromarray(frame).convert("RGB").copy()
        with self._lock:
            if not self._closed:
                self._frames[camera] = image

    def _draw(self, event=None):
        name = self.selected_camera.get()
        with self._lock:
            frame = self._frames.get(name)
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        previous = self._rendered
        if (previous is not None and previous[0] == name
                and previous[1] is frame and previous[2:] == (width, height)):
            return
        self._rendered = (name, frame, width, height)
        self.canvas.coords(self._image_item, width / 2, height / 2)
        self.canvas.coords(self._message_item, width / 2, height / 2)
        if frame is None:
            self.canvas.itemconfigure(self._image_item, image="")
            self._photo = None
            self.canvas.itemconfigure(
                self._message_item, state="normal",
                text="Нет кадра" if name else "Нет камер",
            )
            return
        image = ImageOps.contain(
            frame, (max(1, width), max(1, height)), Image.Resampling.BILINEAR,
        )
        self._photo = ImageTk.PhotoImage(image, master=self.canvas)
        self.canvas.itemconfigure(self._image_item, image=self._photo)
        self.canvas.itemconfigure(self._message_item, state="hidden")

    def _refresh(self):
        self._draw()
        self._after_id = self.after(50, self._refresh)

    def destroy(self):
        with self._lock:
            self._closed = True
            for name in self._frames:
                self._frames[name] = None
        self.after_cancel(self._after_id)
        self._rendered = self._photo = None
        super().destroy()
