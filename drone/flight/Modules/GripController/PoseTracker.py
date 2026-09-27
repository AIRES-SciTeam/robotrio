from gz.transport import Node
from functools import partial
from dataclasses import dataclass
import threading 
from gz.msgs.pose_v_pb2 import Pose_V

from Modules.Logger.LoggerFabric import LoggerFabric
from Modules.GripController.Gripper import GZConfig


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]

    @classmethod
    def from_message(cls, msg):
        p, q = msg.position, msg.orientation
        return cls((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def compose(self, child):
        x, y, z, w = self.orientation
        px, py, pz = child.position
        tx, ty, tz = 2 * (y*pz-z*py), 2 * (z*px-x*pz), 2 * (x*py-y*px)
        position = (
            self.position[0] + px + w*tx + y*tz - z*ty,
            self.position[1] + py + w*ty + z*tx - x*tz,
            self.position[2] + pz + w*tz + x*ty - y*tx,
        )
        cx, cy, cz, cw = child.orientation
        orientation = (
            w*cx + x*cw + y*cz - z*cy,
            w*cy - x*cz + y*cw + z*cx,
            w*cz + x*cy - y*cx + z*cw,
            w*cw - x*cx - y*cy - z*cz,
        )
        return Pose(position, orientation)


class PoseTracker:
    def __init__(
        self, 
        node : Node, 
        gz_config : GZConfig, 
        logger_fabric : LoggerFabric
    ):
        self._node = node
        self._logger = logger_fabric.get_logger("PoseTracker")

        self._model = gz_config.model
        self._cargo = gz_config.cargo
        self._links = {tag: f"{gz_config.tag_family}-{tag}" for tag in gz_config.tags}
        self._names = {self._model, self._cargo, *self._links.values()}

        self._lock = threading.Lock()
        self._drone_pose = None
        self._cargo_poses = {}
        self._received = False

        self._topic = f"/world/{gz_config.world}/dynamic_pose/info"

        if not self._node.subscribe(Pose_V, self._topic, self._on_poses):
            self._logger.error("Cannot subscribe topic: %s", self._topic)
            raise RuntimeError(f"Cannot subscribe topic: {self._topic}")
        self._logger.info("Subscribed to topic: %s", self._topic)
        self._logger.info("Initialized")

    def _on_poses(self, msg):
        poses = {
            pose.name: Pose.from_message(pose)
            for pose in msg.pose
            if pose.name in self._names
        }

        cargo = poses.get(self._cargo)
        goods = {
            tag: cargo.compose(poses[link])
            for tag, link in self._links.items()
            if cargo is not None and link in poses
        }
        with self._lock:
            self._drone_pose = poses.get(self._model)
            self._cargo_poses = goods
            if not self._received:
                self._logger.info("Received first pose message")
                self._received = True

    def snapshot(self) -> tuple[Pose | None, dict[str, Pose]]:
        with self._lock:
            return self._drone_pose, dict(self._cargo_poses)

    def get_drone_pose(self) -> Pose | None:
        with self._lock:
            return self._drone_pose

    def get_cargo_pose(self, tag: str) -> Pose | None:
        with self._lock:
            return self._cargo_poses.get(tag)
        
