// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#ifndef MUJOCO_ROS2_CONTROL_MULTIENV__LOCKSTEP_PROTOCOL_HPP_
#define MUJOCO_ROS2_CONTROL_MULTIENV__LOCKSTEP_PROTOCOL_HPP_

#include <semaphore.h>

#include <cstddef>
#include <cstdint>

namespace mujoco_ros2_control_multienv::lockstep
{

/**
 * Shared-memory layout of one simulation's lockstep channel (POSIX shm, /dev/shm/<name>).
 * mujoco_ros2_control_multienv/lockstep.py mirrors it field by field: change both together
 * and bump kVersion.
 *
 *   [0, kHeaderBytes)            Header
 *   [names_offset, +names_bytes) num_obs then num_act NUL-terminated interface names
 *   [obs_offset, +8 num_obs)     double observations, the claimed state interfaces in order
 *   [act_offset, +8 num_act)     double actions, written to the claimed command interfaces
 *
 * One exchange per step: the bridge writes the observations, stamps obs_step and posts
 * obs_ready, then blocks on act_ready; the client waits on obs_ready, writes the actions and
 * act_step = obs_step, and posts act_ready. The semaphores order the memory, so neither side
 * reads a half-written block, and nothing advances the simulation in between.
 */
constexpr std::uint64_t kMagic = 0x504554534b434f4cULL;  // "LOCKSTEP", little endian
constexpr std::uint32_t kVersion = 1;
constexpr std::size_t kHeaderBytes = 4096;
constexpr std::size_t kSemaphoreBytes = 64;

enum Status : std::uint32_t
{
  kInitializing = 0,  //!< created, the bridge is not active yet
  kRunning = 1,       //!< active: exchanges happen every step
  kClosed = 2,        //!< deactivated or shut down; obs_ready is posted once to release a waiting client
};

struct Header
{
  std::uint64_t magic;               // 0
  std::uint32_t version;             // 8
  std::uint32_t status;              // 12  Status
  std::uint32_t num_obs;             // 16
  std::uint32_t num_act;             // 20
  std::uint32_t names_offset;        // 24
  std::uint32_t names_bytes;         // 28
  std::uint32_t obs_offset;          // 32
  std::uint32_t act_offset;          // 36
  std::int32_t bridge_pid;           // 40
  std::int32_t client_pid;           // 44  written by the client
  double step_period;                // 48  controller time per step [s]
  double update_rate;                // 56  controller-manager rate [Hz]
  std::int32_t handshake_tick_offset;  // 64  0: exchange on the step tick, -1: one tick before it
  std::uint32_t reserved;            // 68
  std::uint64_t obs_step;            // 72  step index of the observations in the block
  std::uint64_t act_step;            // 80  step index the client's actions answer
  double obs_time;                   // 88  controller time of the observations [s]
  std::uint64_t sync_errors;         // 96  exchanges whose act_step did not match obs_step
  unsigned char padding[256 - 104];
  unsigned char obs_ready[kSemaphoreBytes];  // 256  sem_t, process-shared
  unsigned char act_ready[kSemaphoreBytes];  // 320  sem_t, process-shared
};

static_assert(sizeof(sem_t) <= kSemaphoreBytes, "sem_t does not fit its slot");
static_assert(offsetof(Header, handshake_tick_offset) == 64, "layout");
static_assert(offsetof(Header, sync_errors) == 96, "layout");
static_assert(offsetof(Header, obs_ready) == 256, "layout");
static_assert(offsetof(Header, act_ready) == 320, "layout");
static_assert(sizeof(Header) <= kHeaderBytes, "header");

inline sem_t * obs_ready(Header * header) { return reinterpret_cast<sem_t *>(header->obs_ready); }
inline sem_t * act_ready(Header * header) { return reinterpret_cast<sem_t *>(header->act_ready); }

}  // namespace mujoco_ros2_control_multienv::lockstep

#endif  // MUJOCO_ROS2_CONTROL_MULTIENV__LOCKSTEP_PROTOCOL_HPP_
