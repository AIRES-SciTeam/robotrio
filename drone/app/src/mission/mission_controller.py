"""Sequential mission runner. Block callbacks are serialized with operator commands."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
import math
import threading
import time
from typing import Any, Sequence

from mission.mission_env import MissionEnv
from mission.mission_block.mission_block import MBState, MissionBlock


class FailureAction(Enum):
    SKIP = auto()
    RETRY = auto()
    FAIL = auto()

@dataclass
class MissionStep:
    block_class: type[MissionBlock]
    block_params: dict[str, Any] = field(default_factory=dict)
    on_failure: FailureAction = FailureAction.FAIL
    max_retries: int = 0
    retry_delay: float = 0.5

    def __post_init__(self):
        if not issubclass(self.block_class, MissionBlock):
            raise TypeError('block_class must inherit MissionBlock')
        if not isinstance(self.on_failure, FailureAction):
            raise ValueError('Unknown failure action')
        if not isinstance(self.max_retries, int) or self.max_retries < 0:
            raise ValueError('max_retries must be a nonnegative integer')
        if not math.isfinite(self.retry_delay) or self.retry_delay < 0:
            raise ValueError('retry_delay must be finite and nonnegative')
        if 'mission_env' in self.block_params:
            raise ValueError('mission_env is supplied by the controller')

class MissionState(Enum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    FINISHING = auto()
    FAILING = auto()
    COMPLETED = auto()
    FAILED = auto()
    STOPPED = auto()

TERMINAL_STATES = (MissionState.COMPLETED, MissionState.FAILED, MissionState.STOPPED)

class MissionController:
    def __init__(
        self,
        plan: Sequence[MissionStep],
        mission_env: MissionEnv,
        tick_period: float = 0.1,
        *,
        pause_steps: Sequence[MissionStep] = (),
        end_steps: Sequence[MissionStep] = (),
        fail_steps: Sequence[MissionStep] = (),
    ):
        if not math.isfinite(tick_period) or tick_period <= 0:
            raise ValueError('tick_period must be finite and positive')
        def snapshot(steps):
            return tuple(MissionStep(s.block_class, dict(s.block_params), s.on_failure,
                                     s.max_retries, s.retry_delay) for s in steps)
        self.plan = snapshot(plan)
        self.pause_steps = snapshot(pause_steps)
        self.end_steps = snapshot(end_steps)
        self.fail_steps = snapshot(fail_steps)
        self._menv = mission_env
        self._logger = mission_env.logger
        self._tick_period = tick_period
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._ticker_thread = None
        self._state = MissionState.IDLE
        self._index = 0
        self._aux_index = 0
        self._block = None
        self._retries = 0
        self._retry_at = 0.0
        self._last_error = None

    @property
    def state(self):
        with self._lock:
            return self._state

    @property
    def current_index(self):
        """Zero-based main-plan index; len(plan) means no main steps remain."""
        with self._lock:
            return self._index

    @property
    def current_block(self):
        """Current main or auxiliary block, for observation only."""
        with self._lock:
            return self._block

    @property
    def last_error(self):
        with self._lock:
            return self._last_error

    def start(self, *, background: bool = True):
        """Start once. With background=False the caller must drive tick()."""
        with self._lock:
            if self._state != MissionState.IDLE:
                return False
            self._state = MissionState.RUNNING
            if background:
                self._ticker_thread = threading.Thread(target=self._ticker, daemon=True)
                self._ticker_thread.start()
            return True

    def _cancel_block(self):
        block, self._block = self._block, None
        if block is not None:
            try:
                block.cancel()
            except Exception:
                self._logger.exception('Block cancellation failed')

    def _reset_attempt(self):
        self._cancel_block()
        self._retries = 0
        self._retry_at = 0.0

    def _enter(self, state):
        self._reset_attempt()
        self._aux_index = 0
        self._state = state
        self._logger.info('Mission state: %s', state.name)

    def pause(self):
        with self._lock:
            if self._state != MissionState.RUNNING:
                return False
            self._enter(MissionState.PAUSED)
            return True

    def resume(self):
        with self._lock:
            if self._state != MissionState.PAUSED:
                return False
            self._enter(MissionState.RUNNING)
            return True

    def toggle(self):
        with self._lock:
            return self.resume() if self._state == MissionState.PAUSED else self.pause()

    def switch_to(self, index: int):
        """Restart the specified main block, or select it while remaining paused."""
        if not isinstance(index, int) or isinstance(index, bool):
            raise TypeError('index must be an integer')
        if not 0 <= index < len(self.plan):
            raise IndexError('Mission block index out of range')
        with self._lock:
            if self._state not in (MissionState.RUNNING, MissionState.PAUSED):
                return False
            if self._state == MissionState.RUNNING:
                self._reset_attempt()
            self._index = index
            return True

    def skip(self):
        """Skip one main step. On pause, leave its pause chain running."""
        with self._lock:
            if self._state not in (MissionState.RUNNING, MissionState.PAUSED):
                return False
            if self._index >= len(self.plan):
                return False
            if self._state == MissionState.RUNNING:
                self._reset_attempt()
            self._index += 1
            return True

    def next(self):
        return self.skip()

    def prev(self):
        with self._lock:
            if self._index == 0:
                return False
            return self.switch_to(self._index - 1)

    def finish(self):
        """End the main plan early and execute end_steps."""
        with self._lock:
            if self._state not in (MissionState.RUNNING, MissionState.PAUSED):
                return False
            self._index = len(self.plan)
            self._enter(MissionState.FINISHING)
            return True

    def stop(self):
        """Cancel execution without running end/failure steps or stopping shared devices."""
        self._stop_event.set()
        with self._lock:
            if self._state not in TERMINAL_STATES:
                self._enter(MissionState.STOPPED)
            thread = self._ticker_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join()

    def _sequence(self):
        if self._state == MissionState.RUNNING:
            return self.plan, self._index
        if self._state == MissionState.PAUSED:
            return self.pause_steps, self._aux_index
        if self._state == MissionState.FINISHING:
            return self.end_steps, self._aux_index
        return self.fail_steps, self._aux_index

    def _advance(self):
        self._block = None
        self._retries = 0
        self._retry_at = 0.0
        if self._state == MissionState.RUNNING:
            self._index += 1
        else:
            self._aux_index += 1

    def _handle_failure(self, step, error):
        self._last_error = error
        self._logger.error('Mission block failed: %s', error)
        self._cancel_block()
        if step.on_failure == FailureAction.SKIP:
            self._advance()
        elif step.on_failure == FailureAction.RETRY and self._retries < step.max_retries:
            self._retries += 1
            self._retry_at = time.monotonic() + step.retry_delay
        elif self._state == MissionState.FAILING:
            # Never recursively invoke a failed error handler.
            self._enter(MissionState.FAILED)
        else:
            self._enter(MissionState.FAILING)

    def tick(self):
        """Execute at most one block tick. Commands wait for an in-flight tick."""
        with self._lock:
            if self._stop_event.is_set() or self._state in (MissionState.IDLE, *TERMINAL_STATES):
                return self._state
            sequence, index = self._sequence()
            if index >= len(sequence):
                if self._state == MissionState.RUNNING:
                    self._enter(MissionState.FINISHING)
                elif self._state == MissionState.FINISHING:
                    self._enter(MissionState.COMPLETED)
                elif self._state == MissionState.FAILING:
                    self._enter(MissionState.FAILED)
                return self._state
            if time.monotonic() < self._retry_at:
                return self._state
            step = sequence[index]
            state = self._state
            block = None
            try:
                if self._block is None:
                    self._block = step.block_class(mission_env=self._menv, **step.block_params)
                    block = self._block
                    block.start()
                    if self._block is not block or self._state != state:
                        return self._state
                block = self._block
                result = block.tick()
                if self._block is not block or self._state != state:
                    return self._state
                if result == MBState.SUCCEEDED:
                    self._advance()
                elif result in (MBState.FAILED, MBState.CANCELLED):
                    self._handle_failure(step, f'{step.block_class.__name__}: {result.name}')
                elif result != MBState.RUNNING:
                    raise ValueError(f'Unexpected block result: {result!r}')
            except Exception as exc:
                self._logger.exception('Mission block raised an exception')
                if self._state != state or (block is not None and self._block is not block):
                    return self._state
                self._handle_failure(step, f'{step.block_class.__name__}: {exc}')
            return self._state

    def _ticker(self):
        while not self._stop_event.is_set():
            if self.tick() in TERMINAL_STATES:
                return
            self._stop_event.wait(self._tick_period)
