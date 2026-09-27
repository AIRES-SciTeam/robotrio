#!/opt/python-venv/bin/python3
from gz.transport import Node
from gz.msgs.stringmsg_pb2 import StringMsg
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.empty_pb2 import Empty
from functools import partial
import threading
import time


class TestGripper:
    def __init__(self, node, tags):
        self._node = node
        self._tags = tags
        self._cond = threading.Condition()
        self._attached = {tag: True for tag in tags}
        self._attached_tag = None
        self._callbacks = {}

        topic_base = "/model/x500/gripper/goods/tag36h11-"

        for tag in tags:
            state_topic = topic_base + f"{tag}/state"
            callback = partial(self._on_state, tag)
            self._callbacks[state_topic] = callback
            ok = self._node.subscribe(StringMsg, state_topic, callback)
            print(f"subscribe {state_topic}: {ok}", flush=True)

        self._attach_pub = {}
        self._detach_pub = {}
        for tag in tags:
            self._attach_pub[tag] = self._node.advertise(
                topic_base + f"{tag}/attach", Empty
            )
            self._detach_pub[tag] = self._node.advertise(
                topic_base + f"{tag}/detach", Empty
            )

    def _on_state(self, tag, msg):
        print(f"ON_STATE tag={tag!r} data={msg.data!r}", flush=True)
        with self._cond:
            if msg.data == "detached":
                self._attached[tag] = False
                if tag == self._attached_tag:
                    self._attached_tag = None
            elif msg.data == "attached":
                self._attached[tag] = True
                self._attached_tag = tag
            self._cond.notify_all()

    def attach(self, tag):
        print(f"publish attach for {tag}", flush=True)
        self._attach_pub[tag].publish(Empty())

    def detach(self, tag):
        print(f"publish detach for {tag}", flush=True)
        self._detach_pub[tag].publish(Empty())


def on_pose(msg):
    interesting = {"x500", "goods"}
    names = [p.name for p in msg.pose if p.name in interesting]
    # print(f"POSE: {len(msg.pose)} poses, matching: {names}", flush=True)


node = Node()

# Подписка на позы — как в PoseTracker, чтобы воспроизвести нагрузку
ok = node.subscribe(Pose_V, "/world/scene/dynamic_pose/info", on_pose)
print(f"subscribe pose: {ok}", flush=True)

# Класс с partial-callback, как в Gripper
gripper = TestGripper(node, ["00", "01", "02", "03", "04", "05"])

time.sleep(1)  # дать discovery завершиться

while True:
    # gripper.attach("00")
    # gripper.attach("01")
    # gripper.attach("02")
    # gripper.attach("03")
    # gripper.attach("04")
    # gripper.attach("05")
    # time.sleep(1)
    # gripper.detach("00")
    # gripper.detach("01")
    # gripper.detach("02")
    # gripper.detach("03")
    # gripper.detach("04")
    # gripper.detach("05")
    time.sleep(1)