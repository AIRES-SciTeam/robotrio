import subprocess
import re
import math
import logging

from utils import DRONE_ModelConfig


class DRONE_Gripper:
    def __init__(
        self, 
        model_config: DRONE_ModelConfig,
        logger: logging.Logger,
        grip_distance: int = 0.6,
    ):
        self.config = model_config
        self.grip_distance = grip_distance

        self.logger = logger

        self.attached_id = None

        for i in self.config.cargo_ids:
            self._detach(cargo_id=i)

        self.logger.debug("DRONE_Gripper: GripperCTRL initialized.")

    def _attach(self, cargo_id=None):
        topic = f"/model/{self.config.model}/gripper/{self.config.cargo}#{cargo_id}/attach"
        self._publish(topic)
        self.logger.info(f"DRONE_Gripper: Attached cargo {self.config.cargo}#{cargo_id}.")

    def _detach(self, cargo_id):
        topic = f"/model/{self.config.model}/gripper/{self.config.cargo}#{cargo_id}/detach"
        self._publish(topic)
        self.logger.info(f"DRONE_Gripper: Detached cargo {self.config.cargo}#{cargo_id}.")

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
        if not match:
            self.logger.error(f"DRONE_Gripper: Failed: Gazebo returned an unknown pose format: {output!r}")
            raise ValueError(f"Failed: Gazebo returned an unknown pose format: {output!r}")
        return [float(value) for value in match.groups()]

    def _get_pose(self, model_name):
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
        drone_pose = self._get_pose(self.config.model)
        goods_poses = [
            (i, self._get_pose(f"{self.config.cargo}#{i}"))
            for i in self.config.cargo_ids
        ]
        min_distance = float("inf")
        nearest_id = None
        for cargo_id, cargo_pose in goods_poses:
            attachable = self._is_attachable(drone_pose, cargo_pose)
            if attachable:
                distance = math.sqrt(
                    (drone_pose[0] - cargo_pose[0]) ** 2 +
                    (drone_pose[1] - cargo_pose[1]) ** 2 +
                    (drone_pose[2] - cargo_pose[2]) ** 2
                )
                self.logger.debug(f"DRONE_Gripper: Cargo #{cargo_id} is attachable. Pose: {cargo_pose}, distance:{distance}")
                if distance < min_distance:
                    min_distance = distance
                    nearest_id = cargo_id
            else:
                self.logger.debug(f"DRONE_Gripper: Cargo #{cargo_id} is not attachable.")
        return nearest_id

    def _attach_nearest(self):
        nearest_id = self._nearest()
        if nearest_id is not None:
            self._attach(cargo_id=nearest_id)
            self.attached_id = nearest_id
        else:
            self.logger.warning("DRONE_Gripper: No attachable cargo found nearby.")

    def toggle(self):
        self.logger.debug("DRONE_Gripper: toggle cargo")
        if self.attached_id is not None:
            self._detach(cargo_id=self.attached_id)
            self.logger.info(f"DRONE_Gripper: Detached cargo {self.config.cargo}#{self.attached_id}.")
            self.attached_id = None 
        else:
            self._attach_nearest()
