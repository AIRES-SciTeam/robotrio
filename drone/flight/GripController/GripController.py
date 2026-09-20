import subprocess
import re
import math
import logging
from concurrent.futures import ThreadPoolExecutor

from Utils.Configs import DRONE_ModelConfig, DRONE_TagConfig


class DRONE_GripController:
    """
    Управление захватом груза моделью дрона в Gazebo.
    Ответственность:
        * Получение положений дрона и грузов через команду gz model.
        * Выбор ближайшего груза в зоне захвата.
        * Публикация команд прикрепления и освобождения через gz topic.
    Интерфейс:
        1. Инициализация __init__. Аргументы:
            * model_config : DRONE_ModelConfig -- имена моделей дрона и груза.
            * tag_config : DRONE_TagConfig -- семейство и список меток груза.
            * logger : logging.Logger -- объект для записи сообщений в лог.
            * grip_distance : float = 0.6 -- допустимая разница высот в метрах.
           При запуске отправляет команду освобождения для каждой метки.
        2. Метод toggle() -- если груз отмечен как прикреплённый, освобождает его;
           иначе ищет ближайший доступный груз и прикрепляет его.
           Если подходящего груза нет, состояние захвата не меняется.
    Служебные методы:
        * _get_poses() и _parse_poses() -- получают и разбирают положения из Gazebo.
        * _is_attachable() и _nearest() -- проверяют зону захвата и выбирают груз.
        * _attach_nearest(), _attach(), _detach() и _publish() -- публикуют команды.
    attached_tag содержит метку груза после успешной публикации команды
    прикрепления и None после публикации команды освобождения. Gazebo не
    подтверждает фактическое исполнение этих команд через данный интерфейс.
    """

    def __init__(
        self,
        model_config: DRONE_ModelConfig,
        tag_config: DRONE_TagConfig,
        logger: logging.Logger,
        grip_distance: float = 0.6,
    ) -> None:
        self.model_name = model_config.model
        self.cargo_name = model_config.cargo
        self.tags_list = [tag_config.family + "-" + tag for tag in tag_config.list]

        self.grip_distance = grip_distance

        self.logger = logger

        self.attached_tag: str | None = None

        for tag in self.tags_list:
            self._detach(cargo_tag=tag)

        self.logger.info(f"DRONE_GripController: Initialized for {self.model_name} and {self.cargo_name}")

    def _attach(self, cargo_tag: str) -> None:
        topic = f"/model/{self.model_name}/gripper/{self.cargo_name}/{cargo_tag}/attach"
        self._publish(topic)
        self.logger.info(f"DRONE_GripController: Attach command published for {self.cargo_name}/{cargo_tag}")

    def _detach(self, cargo_tag: str) -> None:
        topic = f"/model/{self.model_name}/gripper/{self.cargo_name}/{cargo_tag}/detach"
        self._publish(topic)
        self.logger.info(f"DRONE_GripController: Detach command published for {self.cargo_name}/{cargo_tag}")

    def _publish(self, topic: str) -> None:
        cmd = [
            "gz", "topic", "--topic", topic,
            "--msgtype", "gz.msgs.Empty",
            "--pub", "",
        ]
        subprocess.run(
            cmd,
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        self.logger.debug(f"DRONE_GripController: Published gz.msgs.Empty to {topic}")

    def _parse_poses(self, output: str) -> tuple[list[float], dict[str, list[float]]]:
        number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
        vector = rf"\[[ \t]*({number})[, \t]+({number})[, \t]+({number})[ \t]*\]"
        pattern = (
            rf"^[ \t]*- Name: ([^\r\n]+)\r?\n"
            rf".*?"
            rf"^[ \t]*- Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:[ \t]*\r?\n"
            rf"[ \t]*{vector}[ \t]*\r?\n[ \t]*{vector}[ \t]*\r?$"
        )
        sections = re.split(r"^[ \t]*- Link \[\d+\][ \t]*\r?$", output, flags=re.MULTILINE)
        poses = []
        for section in sections:
            match = re.search(pattern, section, flags=re.DOTALL | re.MULTILINE)
            if match is None:
                raise ValueError(f"DRONE_GripController: Gazebo returned an unknown pose format: {section!r}")
            poses.append((match.group(1).strip(), [float(value) for value in match.groups()[1:]]))

        model_pose = poses[0][1]
        model_x, model_y, model_z, roll, pitch, yaw = model_pose
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        absolute_positions = {}
        for name, pose in poses[1:]:
            x, y, z = pose[:3]
            y, z = cr * y - sr * z, sr * y + cr * z
            x, z = cp * x + sp * z, -sp * x + cp * z
            x, y = cy * x - sy * y, sy * x + cy * y
            absolute_positions[name] = [model_x + x, model_y + y, model_z + z]
        return model_pose[:3], absolute_positions

    def _get_poses(
        self, model_name: str, include_links: bool = False,
    ) -> tuple[list[float], dict[str, list[float]]]:
        cmd = ["gz", "model", "--model", model_name, "--pose"]
        if include_links:
            cmd.append("--link")
        result = subprocess.run(
            cmd,
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        return self._parse_poses(result.stdout)

    def _is_attachable(self, drone_pose: list[float], cargo_pose: list[float]) -> bool:
        good_x = abs(drone_pose[0] - cargo_pose[0]) <= 0.3
        good_y = abs(drone_pose[1] - cargo_pose[1]) <= 0.3
        good_z = drone_pose[2] - cargo_pose[2] <= self.grip_distance

        return good_x and good_y and good_z

    def _nearest(self) -> str | None:
        with ThreadPoolExecutor(max_workers=2) as executor:
            drone_future = executor.submit(self._get_poses, self.model_name)
            cargo_future = executor.submit(self._get_poses, self.cargo_name, include_links=True)
            drone_pose, _ = drone_future.result()
            _, goods_poses = cargo_future.result()
        min_distance = float("inf")
        nearest_tag = None
        for tag in self.tags_list:
            if tag not in goods_poses:
                raise ValueError(f"DRONE_GripController: Gazebo did not return a pose for link {tag!r}")
            cargo_pose = goods_poses[tag]
            attachable = self._is_attachable(drone_pose, cargo_pose)
            if attachable:
                distance = math.sqrt(
                    (drone_pose[0] - cargo_pose[0]) ** 2 +
                    (drone_pose[1] - cargo_pose[1]) ** 2 +
                    (drone_pose[2] - cargo_pose[2]) ** 2
                )
                self.logger.debug(f"DRONE_GripController: Cargo {self.cargo_name}/{tag} is attachable; pose={cargo_pose}, distance={distance:.3f} m")
                if distance < min_distance:
                    min_distance = distance
                    nearest_tag = tag
            else:
                self.logger.debug(f"DRONE_GripController: Cargo {self.cargo_name}/{tag} is not attachable")
        return nearest_tag

    def _attach_nearest(self) -> None:
        nearest_tag = self._nearest()
        if nearest_tag is not None:
            self._attach(cargo_tag=nearest_tag)
            self.attached_tag = nearest_tag
            self.logger.debug(f"DRONE_GripController: Nearest cargo: {nearest_tag}")
        else:
            self.logger.warning("DRONE_GripController: No attachable cargo found nearby")

    def toggle(self) -> None:
        self.logger.debug("DRONE_GripController: Toggling cargo attachment")
        if self.attached_tag is not None:
            self._detach(cargo_tag=self.attached_tag)
            self.attached_tag = None
        else:
            self._attach_nearest()
