import subprocess
import re
import math


class GripperCTRL:
    def __init__(
        self, 
        world="scene",
        model="x500",
        cargo="goods",
        max_cargo_id=4,
        max_distance=0.6
    ):
        self.world = world
        self.model = model
        self.cargo = cargo

        self.attached_id = None

        self.max_cargo_id = max_cargo_id
        self.max_distance = max_distance

        for i in range(1, self.max_cargo_id + 1):
            self._detach(cargo_id=i)

        print("\033[92mGripperCTRL initialized.\033[0m")

    def _attach(self, cargo_id=None):
        topic = f"/model/{self.model}/gripper/{self.cargo}#{cargo_id}/attach"
        self._publish(topic)
        print(f"\033[92m Attached cargo #{cargo_id}.\033[0m")

    def _detach(self, cargo_id):
        topic = f"/model/{self.model}/gripper/{self.cargo}#{cargo_id}/detach"
        self._publish(topic)
        print(f"\033[92mDetached cargo #{cargo_id}.\033[0m")

    def _publish(self, topic):
        subprocess.run(
            [
                "gz", "topic", "--topic", topic,
                "--msgtype", "gz.msgs.Empty",
                "--pub", "",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )

    def _parse_pose(self, output):
        match = re.search(
            r"\[?\s*([-+\d.eE]+)[,\s]+([-+\d.eE]+)[,\s]+([-+\d.eE]+)",
            output,
        )
        if not match:
            raise ValueError(f"Failed: Gazebo returned an unknown pose format: {output!r}")
        return [float(value) for value in match.groups()]

    def _get_pose(self, model_name):
        result = subprocess.run(
            ["gz", "model", "--model", model_name, "--pose"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        return self._parse_pose(result.stdout)

    def _is_attachable(self, drone_pose, cargo_pose):
        good_x = abs(drone_pose[0] - cargo_pose[0]) <= 0.3
        good_y = abs(drone_pose[1] - cargo_pose[1]) <= 0.3
        good_z = drone_pose[2] - cargo_pose[2] <= self.max_distance

        return good_x and good_y and good_z

    def _nearest(self):
        drone_pose = self._get_pose(self.model)
        goods_poses = [
            (i, self._get_pose(f"{self.cargo}#{i}"))
            for i in range(1, self.max_cargo_id + 1)
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
                if distance < min_distance:
                    min_distance = distance
                    nearest_id = cargo_id

        return nearest_id

    def _attach_nearest(self):
        nearest_id = self._nearest()   
        if nearest_id is not None:
            self._attach(cargo_id=nearest_id)
            self.attached_id = nearest_id
        else:
            print("\033[91mNo attachable cargo found nearby.\033[0m")

    def toggle(self):
        if self.attached_id is not None:
            self._detach(cargo_id=self.attached_id)
            self.attached_id = None
        else:
            self._attach_nearest()
