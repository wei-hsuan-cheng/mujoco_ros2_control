# Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.
"""Start N copies of a simulation launch, each isolated in its own ROS domain.

Every copy is the unmodified single-simulation launch - same nodes, topic names, TF frames
and controller manager - with ROS_DOMAIN_ID = domain_id_base + index, so the copies cannot
see each other and nothing has to be namespaced. Each runs as its own process group (its
own cores, a crash stays local) and logs to <log_dir>/env_<index>.log. Data the trainer
needs goes through the lockstep shared memory (lockstep.py), never through these domains.
"""

import os
import signal
import subprocess
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def compile_model(xml_path, mjb_path):
    """MJCF -> .mjb with mujoco_ros2_control's compiler, for simulations to load directly.

    A meshed humanoid costs ~280 MB per process to compile from XML and ~55 MB to load
    compiled, which is what decides how many simulations fit in memory.
    """
    tool = Path(get_package_prefix("mujoco_ros2_control")) / "lib/mujoco_ros2_control/mujoco_compile_model"
    Path(mjb_path).parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([str(tool), str(xml_path), str(mjb_path)], check=True)
    return Path(mjb_path)


class MultiEnvLauncher:
    """launch: [package, launch_file, "arg:=value", ...]; "{env}" in an argument becomes the index."""

    def __init__(self, num_envs, launch, log_dir, domain_id_base=100, stagger=3.0, cwd=None, extra_env=None):
        if domain_id_base < 0 or domain_id_base + num_envs - 1 > 232:
            raise ValueError("ROS domain ids must stay within 0..232")
        self.num_envs = num_envs
        self.launch = list(launch)
        self.log_dir = Path(log_dir)
        self.domain_id_base = domain_id_base
        self.stagger = stagger
        self.cwd = cwd
        self.extra_env = dict(extra_env or {})
        self.processes = []

    def domain_id(self, index):
        return self.domain_id_base + index

    def command(self, index):
        return ["ros2", "launch"] + [arg.replace("{env}", str(index)) for arg in self.launch]

    def start(self):
        """Start every copy, `stagger` seconds apart so their start-up (xacro, CppAD loading,
        controller configuration) does not all land at once."""
        self.log_dir.mkdir(parents=True, exist_ok=True)
        for index in range(self.num_envs):
            env = dict(os.environ)
            env.update(self.extra_env)
            env["ROS_DOMAIN_ID"] = str(self.domain_id(index))
            env.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
            with open(self.log_dir / f"env_{index}.log", "w") as log:
                self.processes.append(subprocess.Popen(
                    self.command(index), cwd=self.cwd, env=env, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True))
            if index + 1 < self.num_envs:
                time.sleep(self.stagger)
        return self

    def alive(self, index):
        return self.processes[index].poll() is None

    def stop(self, timeout=30.0):
        """SIGINT every launch (it shuts its nodes down), then SIGKILL whatever is left."""
        for process in self.processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + timeout
        for process in self.processes:
            try:
                process.wait(max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        for process in self.processes:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for process in self.processes:
            process.wait()
        self.processes = []

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
