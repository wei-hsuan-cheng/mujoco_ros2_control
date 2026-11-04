// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.
//
// Compiles an MJCF scene to a binary .mjb model, which mujoco_ros2_control loads
// directly (mujoco_model_path ending in .mjb).
//
// Loading the MJCF in every simulator process costs ~280 MB of private memory for a
// meshed humanoid (MuJoCo's asset cache plus compiler state that is never returned),
// against ~55 MB for the compiled model: what decides how many simulations fit in a
// machine when several run in parallel. The .mjb is the same compiled model, so the
// physics is identical; it is tied to the MuJoCo version that wrote it.

#include <cstdio>

#include "mujoco/mujoco.h"

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::fprintf(stderr, "usage: %s <model.xml> <model.mjb>\n", argv[0]);
    return 2;
  }
  char error[1000] = "";
  mjModel * model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model)
  {
    std::fprintf(stderr, "failed to load %s: %s\n", argv[1], error);
    return 1;
  }
  if (error[0] != '\0')
  {
    std::fprintf(stderr, "warning while loading %s: %s\n", argv[1], error);
  }
  mj_saveModel(model, argv[2], nullptr, 0);
  mjModel * check = mj_loadModel(argv[2], nullptr);
  if (!check)
  {
    std::fprintf(stderr, "failed to read back %s\n", argv[2]);
    mj_deleteModel(model);
    return 1;
  }
  std::printf("%s -> %s (MuJoCo %s, %.1f MB)\n", argv[1], argv[2], mj_versionString(), model->nbuffer / 1e6);
  mj_deleteModel(check);
  mj_deleteModel(model);
  return 0;
}
