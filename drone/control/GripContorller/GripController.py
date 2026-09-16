import subprocess
import re
import math
import logging
from concurrent.futures import ThreadPoolExecutor

from drone.control.Utils.Configs import DRONE_ModelConfig, DRONE_TagConfig


class DRONE_GripController:
    def __init__(
        self, 
        model_config : DRONE_ModelConfig,
        tag_config : DRONE_TagConfig,
        logger: logging.Logger,
        grip_distance: int = 0.6,
    ):
        self.model_name = model_config.model
        self.cargo_name = model_config.cargo
        self.tags_list = [tag_config.family + "-" + tag for tag in tag_config.list]
        self.grip_distance = grip_distance

        self.logger = logger

        self.attached_tag = None

        for tag in self.tags_list:
            self._detach(cargo_tag=tag)

        self.logger.debug("DRONE_Gripper: GripperCTRL initialized.")

    def _attach(self, cargo_tag):
        topic = f"/model/{self.model_name}/gripper/{self.cargo_name}/{cargo_tag}/attach"
        self._publish(topic)
        self.logger.info(f"DRONE_Gripper: Attached cargo {self.cargo_name}/{cargo_tag}")

    def _detach(self, cargo_tag):
        topic = f"/model/{self.model_name}/gripper/{self.cargo_name}/{cargo_tag}/detach"
        self._publish(topic)
        self.logger.info(f"DRONE_Gripper: Detached cargo {self.cargo_name}/{cargo_tag}")

    def _publish(self, topic):
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
            timeout=2.0
        )
        self.logger.debug(f"DRONE_Gripper: Published \"{cmd[4]}\" to topic: {topic}")

    def _parse_poses(self, output):
        """Return the model world position and a mapping of link world positions."""
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
                raise ValueError(f"DRONE_Gripper: azebo returned an unknown pose format: {section!r}")
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

    def _get_poses(self, model_name, include_links=False):
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

    def _is_attachable(self, drone_pose, cargo_pose):   
        good_x = abs(drone_pose[0] - cargo_pose[0]) <= 0.3
        good_y = abs(drone_pose[1] - cargo_pose[1]) <= 0.3
        good_z = drone_pose[2] - cargo_pose[2] <= self.grip_distance

        return good_x and good_y and good_z

    def _nearest(self):
        with ThreadPoolExecutor(max_workers=2) as executor:
            drone_future = executor.submit(self._get_poses, self.model_name)
            cargo_future = executor.submit(self._get_poses, self.cargo_name, include_links=True)
            drone_pose, _ = drone_future.result()
            _, goods_poses = cargo_future.result()
        min_distance = float("inf")
        nearest_tag = None
        for tag in self.tags_list:
            if tag not in goods_poses:
                raise ValueError(f"DRONE_Gripper: Gazebo did not return a pose for link {tag!r}")
            cargo_pose = goods_poses[tag]
            attachable = self._is_attachable(drone_pose, cargo_pose)
            if attachable:
                distance = math.sqrt(
                    (drone_pose[0] - cargo_pose[0]) ** 2 +
                    (drone_pose[1] - cargo_pose[1]) ** 2 +
                    (drone_pose[2] - cargo_pose[2]) ** 2
                )
                self.logger.debug(f"DRONE_Gripper: Cargo {self.cargo_name}/{tag} is attachable. Pose: {cargo_pose}, distance:{distance}")
                if distance < min_distance:
                    min_distance = distance
                    nearest_tag = tag
            else:
                self.logger.debug(f"DRONE_Gripper: Cargo {self.cargo_name}/{tag} is not attachable.")
        return nearest_tag

    def _attach_nearest(self):
        nearest_tag = self._nearest()
        if nearest_tag is not None:
            self._attach(cargo_tag=nearest_tag)
            self.attached_tag = nearest_tag
            self.logger.debug(f"DRONE_Gripper: Nearest cargo: {nearest_tag}")
        else:
            self.logger.warning("DRONE_Gripper: No attachable cargo found nearby.")

    def toggle(self):
        self.logger.debug("DRONE_Gripper: toggle cargo")
        if self.attached_tag is not None:
            self._detach(cargo_tag=self.attached_tag)
            self.logger.info(f"DRONE_Gripper: Detached cargo {self.cargo_name}/{self.attached_tag}.")
            self.attached_tag = None 
        else:
            self._attach_nearest()
