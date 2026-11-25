// Copyright (c) 2026 Wei-Hsuan Cheng
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef MUJOCO_ROS2_CONTROL__MODEL_ERROR_HPP_
#define MUJOCO_ROS2_CONTROL__MODEL_ERROR_HPP_

#include <cstdint>
#include <string>

#include "mujoco/mujoco.h"

namespace mujoco_ros2_control
{
// Plant-side modeling error for robustness studies: a YAML `model_error` map perturbs the compiled model in memory, so
// one nominal MJCF serves every error setting and the controller keeps its own (nominal) model. Each value is a number
// (a fixed relative change: x -> x (1 + value)), {uniform: [lo, hi]} (a relative change drawn per element from the
// seed) or {value: v} (an absolute value):
//
//   model_error:
//     seed: 0                                 # overridden by the node parameter model_error_seed when >= 0
//     inertia:                                # every body but the world; optional bodies: [...] or exclude: [...]
//       mass: {uniform: [-0.2, 0.2]}
//       com: {uniform: [-0.2, 0.2]}           # CoM shift per axis, relative to the body's radius of gyration
//       inertia: {uniform: [-0.2, 0.2]}       # each principal moment, kept physically valid (A + B >= C)
//     joint_friction:                         # every non-free joint; optional joints: [...]
//       frictionloss: 1.0
//       damping: {value: 0.05}
//       armature: 0.0
//     contact_friction: -0.5                  # sliding friction of every geom (a contact uses the larger of two)
//     ground_regions:                         # the flat floor as a grid of patches, each with its own friction
//       size: 2.0                             # [m] patch edge
//       extent: 20.0                          # [m] the grid covers [-extent, extent]^2; the floor plane stops colliding
//       friction: {uniform: [-0.5, 0.5]}      # per patch, relative to the floor's; the patch's friction governs its
//                                             # contacts (higher geom priority than the robot's)
//     delay: {sensing: 0.005, actuation: 0.005}   # [s] applied by the hardware plugin, see Delays below
//     onset: request                          # load (default): from the start; request: see Onset below
//     ramp: 1.0                               # [s] with onset: request, blend the error in over this time
//
// load_model() loads the MJCF, adding the ground_regions patches through the model-editing API when the file has them;
// apply_model_error() then perturbs the compiled model: call it before mj_makeData, then mj_setConst. It returns the
// realized values as JSON. Both throw std::runtime_error on an invalid file or model.
mjModel * load_model(const std::string & model_path, const std::string & file);
std::string apply_model_error(mjModel * model, const std::string & file, int64_t seed, std::string & summary);

// The `delay` entry: what the controller reads (joints, IMU, force-torque, body states) is `sensing` seconds old, and
// its commands reach the joints `actuation` seconds late. Each is seconds or {uniform: [lo, hi]} drawn once per run:
//   delay: {sensing: 0.005, actuation: {uniform: [0.0, 0.01]}}
struct Delays
{
  double sensing{0.0};
  double actuation{0.0};
};
Delays read_delays(const std::string & file, int64_t seed);

// The `onset` entry: with `onset: request` the simulation starts on the nominal plant (ground patches at the floor's
// friction, no delays) and the node's ~/apply_model_error service switches the error on, blended in over `ramp`
// seconds, e.g. once the controller has taken over, so a start-up phase is not part of the study.
struct Onset
{
  bool on_request{false};
  double ramp{0.0};
};
Onset read_onset(const std::string & file);

// Sets every array apply_model_error() perturbs (inertia, joint friction, geom friction and colour) of `model` to
// nominal + alpha (perturbed - nominal), alpha in [0, 1], and recomputes the derived constants (mj_setConst) with
// `scratch`, an mjData of the model that is not the simulation's own. The three models must have the same structure.
// A blend of two physically valid inertias is valid.
void blend_model(mjModel * model, const mjModel * nominal, const mjModel * perturbed, double alpha, mjData * scratch);
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MODEL_ERROR_HPP_
