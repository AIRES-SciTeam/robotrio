import re

class CommandParser:
    def __init__(
        self,
        gui
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
            #          * Конкретный список
            #          * None - любой значение подходящего типа 
            #          * pos/neg/nonneg/nonpos/nonzero - отношение к 0 для числовых
            #          * re.compile(...) - регулярное выражение
            #   3  - описание (help)
            #   4+ - альтернативные имена
            # Стиль команды: name [--arg1(-a1)  val] [--arg2(-a2) val]...
            "hello": {
                "handler": self._hello,
                "args": [],
            },
            "manual": {
                "handler": self._manual,
                "args": [("mode", str, ["on", "off"], "Переключить режим ручного управления", "m")],
            }
        }

    def _wrong_command(self, cmd):
        self.gui.terminal.write_parts(
            ("[ERROR] ", "error"),
            ("Unknown command ", None),
            (cmd, "accent")
        )

    def _wrong_arg(self, cmd, arg):
        self.gui.terminal.write_parts(
            ("[ERROR] ", "error"),
            ("Wrong argument ", None),
            (arg, "accent"),
            (" for ", None),
            (f"{cmd}", "accent"), 
            (". Type ", None),
            (f" help {cmd} ", "accent"),
            ("to see available variants.", None)
        )

    def parse(self, command : str):
        self._hello()
        # command = command.split()
        # cmd, args = command[0], command[1:]
        # if cmd not in self.map:
        #     self._wrong_command(cmd)
        # else:
        #     self.map[cmd][1](*args)

    def _hello(self):
        self.gui.terminal.write_parts(
            ("[DEBUG] ", "debug"),
            ("Hello World!\n", "accent"),
            ("[INFO] ", "info"),
            ("Hello World!\n", None),
            ("[WARNING] ", "warning"),
            ("Hello World!\n", None),
            ("[ERROR] ", "error"),
            ("Hello World!\n", None)
        )

    def _manual(self, ):
        pass

        
