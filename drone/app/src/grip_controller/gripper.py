from gz.transport import Node
from gz.msgs.stringmsg_pb2 import StringMsg
from gz.msgs.empty_pb2 import Empty
import threading, time
from dataclasses import dataclass
from functools import partial

from logger.logger_fabric import LoggerFabric


MODULE_NAME = "grip_controller"

@dataclass
class GZConfig:
    tags: list[str]
    world: str = "scene"
    model: str = "x500"
    cargo: str = "goods"
    tag_family: str = "tag36h11"
    confirmation_timeout: float = 3.0

    def __post_init__(self):
        from math import isfinite
        if not isfinite(self.confirmation_timeout) or self.confirmation_timeout <= 0:
            raise ValueError("confirmation_timeout must be finite and positive")
        if len(set(self.tags)) != len(self.tags):
            raise ValueError("Tags must be unique")

class Gripper:
    def __init__(
        self,
        node : Node,
        gz_config : GZConfig,
        logger_fabric : LoggerFabric
    ):
        if not logger_fabric.is_alive():
            raise RuntimeError("LoggerFabric is stopped")
        self._logger = logger_fabric.get_logger(f"{type(self).__name__}", module_name=MODULE_NAME)
        self._logger.info("Initializing")
        self.node = node

        self._timeout = gz_config.confirmation_timeout
        self._pending_tag = None
        self._pending_state = None

        topic_base = (
            f"/model/{gz_config.model}/gripper/{gz_config.cargo}"
            f"/{gz_config.tag_family}-"
        )

        self._attached_tag = None
        
        self._cond = threading.Condition()
        self._attached = {tag: False for tag in gz_config.tags}

        self._command_lock = threading.Lock()
        self._attach_publishers = {}
        self._detach_publishers = {}

        with self._command_lock:
            self._callbacks = {}
            for tag in gz_config.tags:
                state_topic = topic_base + f"{tag}/state"
                callback = partial(self._on_state, tag)
                self._callbacks[state_topic] = callback
                if not self.node.subscribe(StringMsg, state_topic, self._callbacks[state_topic]):
                    self._logger.error("Cannot subscribe topic: %s", state_topic)
                    raise RuntimeError(f"Cannot subscribe topic: {state_topic}")
                else:
                    self._logger.info("Subscribed to topic: %s", state_topic)
                self._attach_publishers[tag] = self.node.advertise(
                    topic_base + f"{tag}/attach", Empty
                )
                self._detach_publishers[tag] = self.node.advertise(
                    topic_base + f"{tag}/detach", Empty
                )
                self._logger.debug("Created attach and detach publishers for tag %s", tag)

        self._detach_all(tags=gz_config.tags)

        self._logger.info("Initialized")

    def _on_state(self, tag, msg):
        self._logger.debug("Received state for tag %s: %s", tag, msg.data)
        with self._cond:
            if msg.data == "detached":
                self._attached[tag] = False
                if tag == self._attached_tag:
                    self._attached_tag = None
                self._logger.info("Detached tag %s", tag)
            elif msg.data == "attached":
                self._attached[tag] = True
                self._attached_tag = tag
                self._logger.info("Attached tag %s", tag)
            else:
                self._logger.error("Unknown state for tag: %s: %r", tag, msg.data)    
            if msg.data == self._pending_state and self._pending_tag == tag:
                self._pending_tag = None
            self._cond.notify_all()

    def _detach_all(self, tags):
        time.sleep(0.5)
        try:
            self._logger.info("Detaching tags: %s", tags)
            for tag in tags:
                self._logger.info("Detaching tag: %s", tag)    
                if not self._detach_publishers[tag].publish(Empty()):
                    raise RuntimeError(f"Cannot publish detach command for {tag}")
        except Exception as e:
            self._logger.error("Detaching all error: %s", e)
            raise

    def grip(self, tag, confirm: bool = True) -> bool:
        self._logger.debug("Grip requested for tag %s", tag)
        with self._command_lock:
            with self._cond:
                self._validate_tag(tag)
                if (any(attached and other != tag for other, attached in self._attached.items())
                        or self._pending_tag not in (None, tag)):
                    self._logger.warning(
                        "Cannot grip tag %s: another cargo is attached or awaiting confirmation",
                        tag,
                    )
                    return False
                if self._attached[tag]:
                    self._logger.debug("Tag %s is already confirmed attached", tag)
                    return True
                self._logger.debug("Publishing attach command for tag %s", tag)
                previous_pending = self._pending_tag, self._pending_state
                self._pending_tag = tag
                self._pending_state = "attached"
                if not self._attach_publishers[tag].publish(Empty()):
                    self._pending_tag, self._pending_state = previous_pending
                    self._logger.error("Cannot publish attach command for tag %s", tag)
                    raise RuntimeError(f"Cannot publish attach command for {tag}")
                self._logger.info("Attach command published for tag %s", tag)
                if not confirm:
                    return True
                res = self._cond.wait_for(lambda: self._attached[tag], timeout=self._timeout)
                self._logger.debug("Grip confirmed for tag %s, res : %s", tag, res)
                if not res:
                    return False
                self._attached_tag = tag
            return True

    def release(self, tag : str | None = None, confirm : bool = True):
        self._logger.debug("Release requested for tag %s, need_confirm = %s", tag, confirm)
        with self._command_lock:
            with self._cond:
                tag = (self._pending_tag or next(
                    (t for t, attached in self._attached.items() if attached), None
                )) if tag is None else tag
                if tag is None:
                    return True
                self._validate_tag(tag)
                self._logger.debug("Publishing detach command for tag %s", tag)
                previous_pending = self._pending_tag, self._pending_state
                self._pending_tag = tag
                self._pending_state = "detached"
                if not self._detach_publishers[tag].publish(Empty()):
                    self._pending_tag, self._pending_state = previous_pending
                    self._logger.error("Cannot publish detach command for tag %s", tag)
                    raise RuntimeError(f"Cannot publish detach command for {tag}")
                self._logger.info("Detach command published for tag %s", tag)
                if not confirm:
                    return True
                
                res = self._cond.wait_for(
                    lambda: not self._attached[tag] and self._pending_tag != tag,
                    timeout=self._timeout,
                )
                self._logger.debug("Release confirmed for tag %s, res = %s", tag, res)
                if not res:
                    return False
            return True

    def is_busy(self):
        with self._cond:
            return any(self._attached.values()) or self._pending_tag is not None

    def _validate_tag(self, tag):
        if tag not in self._attached:
            raise ValueError(f"Unknown cargo tag: {tag!r}")

    def is_attached(self, tag):
        with self._cond:
            self._validate_tag(tag)
            return self._attached[tag]
