from __future__ import annotations
from typing import TYPE_CHECKING
import shlex
import argparse
import re

if TYPE_CHECKING:
    from .gui import GUI

class ArgParser(argparse.ArgumentParser):
    def error(self, msg):
        raise ValueError(msg)

class CommandParser:
    def __init__(
        self,
        gui : GUI
    ):
        self.gui = gui
        self.map = {
            # Команда (словарь):
            #   (ключ) name - имя при вызове
            #   handler     - процедура, вызываемая командой
            #   args        - аргументы 
            #   help        - описание
            # Аргумент (кортеж):
            #   0  - название
            #   1  - тип данных
            #   2  - допустимые варианты:
            #     * Конкретный список
            #     * None - любой значение подходящего типа 
            #     * pos/neg/nonneg/nonpos/nonzero - отношение к 0 для числовых
            #     * re.compile(...) - регулярное выражение
            #   3  - описание (help)
            #   4+ - альтернативные имена
            # Стиль команды: name [--arg1(-a1)  val] [--arg2(-a2) val]...
            "hello": {
                "handler": self._hello,
                "args": [],
                "help": "Проверка работоспособности терминала"
            },
            "manual": {
                "handler": self._manual,
                "args": [("mode", str, ["on", "off"], "Переключить режим ручного управления", "m")],
                "help": "Не добавлена"
            },
            "clear": {
                "handler": self._clear,
                "args": [],
                "help": "Очистить терминал"
            }
        }
        self.parsers = {}
        for name, spec in self.map.items():
            parser = ArgParser(
                prog=name,
                add_help=True,
                allow_abbrev=False
            )
            for arg, arg_t, allowed, help, *aliases in spec["args"]:
                flags = [f"--{arg}", *(f"-{alias}" for alias in aliases)]
                options = {"dest": arg, "help": help}
                if arg_t is bool:
                    options["action"] = "store_true"
                else:
                    options["type"] = self._arg_type(arg_t, allowed)
                parser.add_argument(*flags, **options)
            self.parsers[name] = parser

    def _arg_type(self, val_t, allowed):
        checks = {
            "pos":     lambda x: x > 0,
            "neg":     lambda x: x < 0,
            "nonpos":  lambda x: x <= 0,
            "nonneg":  lambda x: x >= 0,
            "nonzero": lambda x: x != 0,
        }
        def parse_val(raw):
            try:
                val = val_t(raw)
            except (ValueError, TypeError) as e:
                raise argparse.ArgumentTypeError(
                    f"Не удалось преобразовать {raw!r} в {val_t.__name__}"
                ) from e
            if allowed is None:
                return val
            if isinstance(allowed, re.Pattern):
                valid = allowed.fullmatch(raw) is not None
                req = f"Соответствие шаблону {allowed.pattern!r}"
            elif isinstance(allowed, str) and allowed in checks:
                valid = checks[allowed](val)
                req = allowed
            elif isinstance(allowed, (list, tuple, set)):
                valid = val in allowed
                req = f"одно из {allowed!r}"
            else:
                raise ValueError(f"Нераспознанное ограничение: {allowed!r}")
            if not valid:
                raise argparse.ArgumentTypeError(
                    f"Недопустимое значение {raw!r}: требуется {req}"
                )
            return val
        return parse_val

    def _wrong_command(self, cmd):
        self.gui.terminal.write_parts(
            ("[ERROR] ", "error"),
            ("Unknown command ", None),
            (cmd, "accent")
        )

    def parse(self, command : str):
        try:
            tokens = shlex.split(command)
        except ValueError as e:
            self.gui.terminal.write_parts(
                ("[ERROR] ", "error"),
                (f"Ошибка парсинга: {e}", None)
            )
            return
        if not tokens:
            return
        cmd, *args = tokens
        if cmd not in self.map:
            self._wrong_command(cmd)
            return
        try:
            parsed = self.parsers[cmd].parse_args(args)
        except ValueError as e:
            self.gui.terminal.write_parts(
                ("[ERROR] ", "error"),
                (f"{e}", None),
            )
            return
        self.map[cmd]["handler"](**vars(parsed))
    
    def _hello(self):
        self.gui.terminal.write_parts(
            ("[DEBUG] ", "debug"),
            ("Hello World!\n", "accent"),
            ("[INFO] ", "info"),
            ("Hello World!\n", None),
            ("[WARNING] ", "warning"),
            ("Hello World!\n", None),
            ("[ERROR] ", "error"),
            ("Hello World!", None)
        )

    def _manual(self, mode : str= None):
        if mode == "on":
            self.gui.control_panel.set_enabled(True)
        elif mode == "off":
            self.gui.control_panel.set_enabled(False)

    def _clear(self):
        self.gui.terminal.clear()