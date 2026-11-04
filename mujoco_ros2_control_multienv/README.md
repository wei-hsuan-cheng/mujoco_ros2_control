# mujoco_ros2_control_multienv

Many `mujoco_ros2_control` simulations in parallel, on CPU, each stepped in lockstep with an
external process such as an RL trainer. Nothing in that loop goes through a topic.

Every simulation is the unmodified single-simulation launch, with the same nodes, topic names,
TF frames and controllers. Each runs as its own process group, and its own `ROS_DOMAIN_ID`
keeps the copies from seeing each other.

## Pieces

| | what |
|---|---|
| `LockstepBridgeController` | ros2_control controller. Every `step_period` of controller time it copies the state interfaces it claims into shared memory. It then blocks the controller manager, and with it the simulation, until the client writes the values for the command interfaces it claims. |
| `lockstep.py` | The client: `LockstepChannel` (one simulation) and `LockstepVecEnv` (N stepped together, every step's timing checked). |
| `launcher.py`, `multi_env` | Start N copies of a launch, `ROS_DOMAIN_ID = base + i`, logs in `<log_dir>/env_<i>.log`. `compile_model()` turns the scene into a `.mjb` once. |

## Why the latency is fixed

At step `k` the bridge publishes the claimed interfaces at controller time `k * step_period`.
Nothing advances until the client answers, and the answer is in the claimed command interfaces
from that tick on. A busy machine therefore makes the run slower, never staler.

For that to hold end to end, whatever the bridge commands must also run on controller time.
The legged MPC's `mpcExecution.mode: synchronous` solves inside the control tick for exactly
this reason.

Claiming another controller's reference interfaces and also reading its exported state
leaves the controller manager's update order between the two ambiguous. Set `order_probe` to
that controller's latest update time. The bridge then detects the order on the first tick
and, if the other controller runs first, exchanges one tick earlier. Either way the answer
reaches it on the step tick; `handshake_tick_offset` in the header records which case applied.

## Protocol

`/dev/shm/<shm_name>`, laid out in `include/mujoco_ros2_control_multienv/lockstep_protocol.hpp`
and mirrored in `lockstep.py`:
- a 4 KiB header (magic, version, sizes, step index and time, two process-shared semaphores);
- the interface names;
- `double obs[num_obs]`;
- `double act[num_act]`.

Per step:
1. The bridge writes `obs`, `obs_step` and `obs_time`, posts `obs_ready` and waits on `act_ready`.
2. The client waits on `obs_ready`, writes `act` and `act_step = obs_step`, and posts `act_ready`.

## Memory

A simulation's private memory decides how many fit. The main costs found for the H2
whole-body MPC:

- **Loading the MJCF** costs about 280 MB per process: MuJoCo's asset cache plus compiler
  state that is never returned. Loading the compiled `.mjb` costs about 55 MB with the same
  physics. Pass `compile_model()`'s output as the scene.
- **Unused nodes.** RViz-only nodes and a 1 kHz joint state broadcaster feed TF listeners
  nobody reads. The legged MPC's training overlay turns these down.

## Use

See `legged_robot_mpc_controller/tests/lockstep_multi_env_test.py` for a complete example. It
launches N H2 whole-body MPCs with the state estimator in the loop, steps them through bring-up
and a walking sequence, and checks timing and survival.

```python
from mujoco_ros2_control_multienv import LockstepVecEnv, MultiEnvLauncher, compile_model

launcher = MultiEnvLauncher(8, ["legged_robot_mpc_controller", "robot.launch.py", ...,
                                "lockstepShmName:=legged_env_{env}"], "/tmp/envs", cwd=workspace)
launcher.start()
envs = LockstepVecEnv([f"legged_env_{i}" for i in range(8)], alive=launcher.alive)
obs = envs.first()                      # (8, num_obs), step 0
while training:
    obs = envs.step(policy(obs))        # (8, num_act) in, next step's observations out
launcher.stop()
```

To start copies from the shell instead:

```bash
ros2 run mujoco_ros2_control_multienv multi_env --num-envs 8 --shm-name legged_env_{env} -- \
  legged_robot_mpc_controller robot.launch.py lockstepShmName:=legged_env_{env} ...
```

`ROS_DOMAIN_ID=<base + i> ros2 topic list` (or RViz in that domain) inspects copy `i`.

## Status

The substrate is validated for up to 16 H2 whole-body MPC simulations with the state
estimator in the loop. RL itself is not built yet: no adaptation-parameter interfaces in
the MPC, no episode reset without relaunching, no trainer. The handoff, with measurements,
design reasons, traps and the open items in order, is
`legged_robot_mpc_controller/docs/multi_env_handoff.md` (branch `feat/multi_env`).
