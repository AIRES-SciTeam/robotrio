import subprocess
import re
import math
import logging

from utils import DRONE_ModelConfig, DRONE_TagConfig


class DRONE_Gripper:
    def __init__(
        self, 
        model_config: DRONE_ModelConfig,
        tag_config : DRONE_TagConfig,
        logger: logging.Logger,
        grip_distance: int = 0.6,
    ):
        self.drone_name = model_config.model
        self.cargo_name = model_config.cargo
        self.tags_list = [tag_config.family + "-" + tag for tag in tag_config.list]
        self.grip_distance = grip_distance

        self.logger = logger

        self.attached_tag = None

        for tag in self.tags_list:
            self._detach(cargo_tag=tag)

        self.logger.debug("DRONE_Gripper: GripperCTRL initialized.")

    def _attach(self, cargo_tag):
        topic = f"/model/{self.drone_name}/gripper/{self.cargo_name}/{cargo_tag}/attach"
        self._publish(topic)
        self.logger.info(f"DRONE_Gripper: Attached cargo {self.cargo_name}/{cargo_tag}")

    def _detach(self, cargo_tag):
        topic = f"/model/{self.drone_name}/gripper/{self.cargo_name}/{cargo_tag}/detach"
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

    def _parse_pose(self, output):
        match = re.search(
            r"\[?\s*([-+\d.eE]+)[,\s]+([-+\d.eE]+)[,\s]+([-+\d.eE]+)",
            output,
        )
        self.logger.debug(f"DRONE_Gripper: Got match: {match}")
        if not match:
            self.logger.error(f"DRONE_Gripper: Failed: Gazebo returned an unknown pose format: {output!r}")
            raise ValueError(f"Failed: Gazebo returned an unknown pose format: {output!r}")
        return [float(value) for value in match.groups()]

    def _get_pose(self, model_name, tag=None):
        if tag:
            cmd = ["gz", "model", "--model", model_name, "--link", tag, "--pose"]
        else:
            cmd = ["gz", "model", "--model", model_name, "--pose"]
        result = subprocess.run(
            cmd,
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0
        )
        return self._parse_pose(result.stdout)

    def _is_attachable(self, drone_pose, cargo_pose):   
        good_x = abs(drone_pose[0] - cargo_pose[0]) <= 0.3
        good_y = abs(drone_pose[1] - cargo_pose[1]) <= 0.3
        good_z = drone_pose[2] - cargo_pose[2] <= self.grip_distance

        return good_x and good_y and good_z

    def _nearest(self):
        drone_pose = self._get_pose(self.drone_name)
        goods_poses = [
            (tag, self._get_pose(self.cargo_name, tag))
            for tag in self.tags_list
        ]
        min_distance = float("inf")
        nearest_tag = None
        for tag, cargo_pose in goods_poses:
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
            self.attached_tag = nearest_tag
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
