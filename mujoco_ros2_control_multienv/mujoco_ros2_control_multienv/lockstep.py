# Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.
"""Client side of the lockstep shared-memory channel (include/.../lockstep_protocol.hpp).

One LockstepChannel per simulation; LockstepVecEnv steps several together. Nothing here
goes through ROS: the bridge controller blocks the simulation at every step until the
client answers, so an observation is always the state at its step's controller time and an
action is in effect from that step on, independent of how loaded the machine is.
"""

import ctypes
import mmap
import os
import struct
import time

import numpy as np

MAGIC = 0x504554534B434F4C  # "LOCKSTEP"
VERSION = 1
HEADER_BYTES = 4096
OBS_READY = 256
ACT_READY = 320
STATUS_INITIALIZING, STATUS_RUNNING, STATUS_CLOSED = 0, 1, 2

# Header field offsets, as in lockstep::Header.
_MAGIC, _VERSION, _STATUS = 0, 8, 12
_NUM_OBS, _NUM_ACT, _NAMES_OFFSET, _NAMES_BYTES, _OBS_OFFSET, _ACT_OFFSET = 16, 20, 24, 28, 32, 36
_BRIDGE_PID, _CLIENT_PID, _STEP_PERIOD, _UPDATE_RATE, _HANDSHAKE_OFFSET = 40, 44, 48, 56, 64
_OBS_STEP, _ACT_STEP, _OBS_TIME, _SYNC_ERRORS = 72, 80, 88, 96

_libc = ctypes.CDLL("libc.so.6", use_errno=True)


class _Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


_libc.sem_post.argtypes = [ctypes.c_void_p]
_libc.sem_post.restype = ctypes.c_int
_libc.sem_timedwait.argtypes = [ctypes.c_void_p, ctypes.POINTER(_Timespec)]
_libc.sem_timedwait.restype = ctypes.c_int


class LockstepClosed(RuntimeError):
    """The bridge deactivated or its simulation went away."""


def shm_path(name):
    return "/dev/shm/" + name.lstrip("/")


def remove_stale(names):
    """Delete channels a killed simulation left behind, so a client cannot attach to them."""
    for name in names:
        try:
            os.unlink(shm_path(name))
        except FileNotFoundError:
            pass


class LockstepChannel:
    """One simulation's channel. obs / act are live views into the shared memory."""

    def __init__(self, name, timeout=None, alive=None):
        """Wait for the bridge to create and activate the channel, then attach.

        alive: optional callable, polled while waiting; returning False raises LockstepClosed
        (e.g. the simulation process exited).
        """
        self.name = name.lstrip("/")
        self._alive = alive
        deadline = None if timeout is None else time.monotonic() + timeout
        path = shm_path(self.name)
        while True:
            if os.path.exists(path) and os.path.getsize(path) >= HEADER_BYTES:
                fd = os.open(path, os.O_RDWR)
                try:
                    mm = mmap.mmap(fd, os.fstat(fd).st_size)
                finally:
                    os.close(fd)
                magic, = struct.unpack_from("<Q", mm, _MAGIC)
                status, = struct.unpack_from("<I", mm, _STATUS)
                if magic == MAGIC and status == STATUS_RUNNING and self._bridge_alive(mm):
                    self._mm = mm
                    break
                mm.close()
            self._check_alive()
            if deadline is not None and time.monotonic() > deadline:
                raise TimeoutError(f"lockstep channel {path} did not become active")
            time.sleep(0.2)

        version, = struct.unpack_from("<I", self._mm, _VERSION)
        if version != VERSION:
            raise RuntimeError(f"{path}: protocol version {version}, this client speaks {VERSION}")
        num_obs, num_act, names_offset, names_bytes, obs_offset, act_offset = struct.unpack_from(
            "<6I", self._mm, _NUM_OBS)
        names = bytes(self._mm[names_offset:names_offset + names_bytes]).split(b"\0")[:-1]
        names = [n.decode() for n in names]
        self.obs_names, self.act_names = names[:num_obs], names[num_obs:]
        self.step_period, self.update_rate = struct.unpack_from("<2d", self._mm, _STEP_PERIOD)
        self.obs = np.frombuffer(self._mm, dtype=np.float64, count=num_obs, offset=obs_offset)
        self.act = np.frombuffer(self._mm, dtype=np.float64, count=num_act, offset=act_offset)
        self._anchor = ctypes.c_char.from_buffer(self._mm)
        self._base = ctypes.addressof(self._anchor)
        struct.pack_into("<i", self._mm, _CLIENT_PID, os.getpid())

    @staticmethod
    def _bridge_alive(mm):
        pid, = struct.unpack_from("<i", mm, _BRIDGE_PID)
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def _check_alive(self):
        if self._alive is not None and not self._alive():
            raise LockstepClosed(f"{self.name}: the simulation exited")

    def index(self, name):
        return self.obs_names.index(name)

    @property
    def status(self):
        return struct.unpack_from("<I", self._mm, _STATUS)[0]

    @property
    def obs_step(self):
        return struct.unpack_from("<Q", self._mm, _OBS_STEP)[0]

    @property
    def obs_time(self):
        return struct.unpack_from("<d", self._mm, _OBS_TIME)[0]

    @property
    def sync_errors(self):
        return struct.unpack_from("<Q", self._mm, _SYNC_ERRORS)[0]

    @property
    def handshake_tick_offset(self):
        return struct.unpack_from("<i", self._mm, _HANDSHAKE_OFFSET)[0]

    def wait(self, timeout=None):
        """Block until the bridge publishes the next step's observations."""
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            now = time.time()
            slice_end = now + 0.5
            ts = _Timespec(int(slice_end), int((slice_end % 1.0) * 1e9))
            if _libc.sem_timedwait(self._base + OBS_READY, ctypes.byref(ts)) == 0:
                if self.status == STATUS_CLOSED:
                    raise LockstepClosed(f"{self.name}: the bridge closed the channel")
                return
            err = ctypes.get_errno()
            if err not in (110, 4):  # ETIMEDOUT, EINTR
                raise OSError(err, os.strerror(err))
            if self.status == STATUS_CLOSED or not self._bridge_alive(self._mm):
                raise LockstepClosed(f"{self.name}: the bridge closed the channel")
            self._check_alive()
            if deadline is not None and time.monotonic() > deadline:
                raise TimeoutError(f"{self.name}: no observation within {timeout} s")

    def answer(self, actions):
        """Hand the actions for the pending step to the bridge, which resumes the simulation."""
        self.act[:] = actions
        struct.pack_into("<Q", self._mm, _ACT_STEP, self.obs_step)
        if _libc.sem_post(self._base + ACT_READY) != 0:
            err = ctypes.get_errno()
            raise OSError(err, os.strerror(err))

    def close(self):
        if getattr(self, "_mm", None) is None:
            return
        # The views and the ctypes anchor export the buffer; mmap.close() refuses while they live.
        self.obs = self.act = self._anchor = None
        self._mm.close()
        self._mm = None


class LockstepVecEnv:
    """Several channels stepped together, with every step's timing checked.

    first() returns the observations of step 0; each step(actions) answers every
    environment and returns the next step's observations, shape (num_envs, num_obs). A step
    whose index or controller time does not follow the previous one by exactly one step is
    counted in sync_violations - by construction there should be none.
    """

    def __init__(self, names, timeout=None, alive=None):
        """alive: optional callable(env_index) -> bool polled while waiting."""
        self.channels = []
        for i, name in enumerate(names):
            check = (lambda i=i: alive(i)) if alive is not None else None
            self.channels.append(LockstepChannel(name, timeout=timeout, alive=check))
        first = self.channels[0]
        for channel in self.channels[1:]:
            if channel.obs_names != first.obs_names or channel.act_names != first.act_names:
                raise RuntimeError(f"{channel.name} does not expose the same interfaces as {first.name}")
        self.obs_names, self.act_names = first.obs_names, first.act_names
        self.step_period = first.step_period
        self.num_envs = len(self.channels)
        self.sync_violations = 0
        self.violation_examples = []  # (env, previous (step, time), this (step, time)), the first ten
        self._last = [None] * self.num_envs
        self._buffer = np.empty((self.num_envs, len(self.obs_names)))

    def index(self, name):
        return self.obs_names.index(name)

    def _collect(self, timeout):
        for i, channel in enumerate(self.channels):
            channel.wait(timeout)
            step, t = channel.obs_step, channel.obs_time
            if self._last[i] is not None:
                last_step, last_t = self._last[i]
                if step != last_step + 1 or abs(t - last_t - self.step_period) > 1e-6:
                    self.sync_violations += 1
                    if len(self.violation_examples) < 10:
                        self.violation_examples.append((i, (last_step, last_t), (step, t)))
            self._last[i] = (step, t)
            self._buffer[i] = channel.obs
        return self._buffer.copy()

    def first(self, timeout=None):
        return self._collect(timeout)

    def step(self, actions, timeout=None):
        actions = np.asarray(actions, dtype=np.float64).reshape(self.num_envs, len(self.act_names))
        for channel, action in zip(self.channels, actions):
            channel.answer(action)
        return self._collect(timeout)

    @property
    def times(self):
        return np.array([last[1] for last in self._last])

    @property
    def bridge_sync_errors(self):
        return sum(channel.sync_errors for channel in self.channels)

    def close(self):
        for channel in self.channels:
            channel.close()
