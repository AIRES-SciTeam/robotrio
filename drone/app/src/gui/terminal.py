import tkinter as tk

from .colors import *


class Terminal(tk.Frame):
    def __init__(
        self, 
        parent, 
        on_command
    ):
        super().__init__(
            parent, bg=PANEL, bd=0,
            padx=0, pady=0, 
            highlightthickness=2,
            highlightbackground=CYAN,
            highlightcolor=CYAN,
        )
        self.on_command = on_command
        self.history = []
        self.history_idx = 0
        self.command_draft = ""

        self._configure_grid()
        self._set_name()
        self._create_log()
        self._create_cmd()

    def _configure_grid(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=1)
        self.rowconfigure(2, weight=0)

    def _set_name(self):           
        tk.Label(
            self, text=" Терминал ",
            bg=PANEL, fg=CYAN
        ).grid(row=0, column=0, sticky="w")

    def _create_log(self):
        self.log = tk.Text(
            self, state="disabled",
            bg=PANEL, fg=TEXT,
            width=1, height=1,
            bd=0, highlightthickness=0,
            selectbackground=CYAN,
            selectforeground=BACKGROUND,
            inactiveselectbackground=CYAN
        )
        self.log.grid(row=1, column=0, sticky="nsew")
        self.log.tag_configure("error", foreground=RED)
        self.log.tag_configure("warning", foreground=YELLOW)
        self.log.tag_configure("info", foreground=GREEN)
        self.log.tag_configure("debug", foreground=BLUE)
        self.log.tag_configure("accent", foreground=CYAN)
        self.log.tag_raise("sel")

    def _create_cmd(self):
        self.command_line = tk.Frame(
            self, bg=PANEL, bd=0,
            highlightbackground=CYAN,
            highlightcolor=CYAN,
            highlightthickness=2
        )
        self.command_line.place(relx=0, rely=1, relwidth=1, anchor="sw", bordermode="outside")
        self.command_line.columnconfigure(1, weight=1)
        tk.Label(
            self.command_line, text="robotrio.drone >",
            bg=PANEL, fg=CYAN
        ).grid(row=0, column=0, padx=(8, 6), pady=8)
        self.entry = tk.Entry(
            self.command_line, bg=PANEL, fg=TEXT,
            insertbackground=CYAN, bd=0,
            highlightthickness=0
        )
        self.entry.grid(row=0, column=1, sticky="ew", padx=(0,8))

        self.entry.bind("<Return>", self._submit)
        self.entry.bind("<FocusIn>", self._focus_in)
        self.entry.bind("<FocusOut>", self._focus_out)
        self.entry.bind("<Up>", self._history_prev)
        self.entry.bind("<Down>", self._history_next)

    def clear(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", tk.END)
        self.log.configure(state="disabled")

    def _focus_in(self, _):
        self.configure(highlightcolor=MAGENTA)
        self.command_line.configure(highlightcolor=MAGENTA)

    def _focus_out(self, _):
        self.configure(highlightcolor=CYAN)
        self.command_line.configure(highlightcolor=CYAN)

    def _submit(self, _):
        command = self.entry.get().strip()
        if not command:
            return "break"
        if not self.history or self.history[-1] != command:
            self.history.append(command)
        self.history_idx = len(self.history)
        self.command_draft = ""
        self.entry.delete(0, tk.END)
        self.write_parts(
            ("robotrio.drone > ", "accent"),
            (command, None)
        )
        self.on_command(command)
        return "break"

    def _history_prev(self, _):
        if self.history_idx > 0:
            if self.history_idx == len(self.history):
                self.command_draft = self.entry.get()

            self.history_idx -= 1
            self._replace_entry(self.history[self.history_idx])
        return "break"

    def _history_next(self, _):
        if self.history_idx < len(self.history):
            self.history_idx += 1
            if self.history_idx == len(self.history):
                self._replace_entry(self.command_draft)
            else:
                self._replace_entry(self.history[self.history_idx])
        return "break"

    def _replace_entry(self, msg):
        self.entry.delete(0, tk.END)
        self.entry.insert(0, msg)
        self.entry.icursor(tk.END)
        self.entry.xview_moveto(1)
 
    def write(self, msg):
        max_lines = 500
        self.log.configure(state="normal")
        self.log.insert(tk.END, str(msg) + "\n")
        line_count = int(self.log.index("end-1c").split(".")[0]) - 1
        excess = line_count - max_lines
        if excess > 0:
            self.log.delete("1.0", f"{excess + 1}.0")
        self.log.configure(state="disabled")
        self.log.see(tk.END)

    def write_parts(self, *parts):
        self.log.configure(state="normal")
        for text, tag in parts:
            self.log.insert(tk.END, text, (tag,) if tag else ())
        self.log.insert(tk.END, "\n")
        line_count = int(self.log.index("end-1c").split(".")[0]) - 1
        if line_count > 500:
            self.log.delete("1.0", f"{line_count - 500 + 1}.0")
        self.log.configure(state="disabled")
        self.log.see(tk.END)
