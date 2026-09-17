# Lidar scan patterns

Each `<name>.npy` is an `(N, 2)` float array of `(azimuth, elevation)` in radians,
in firing order, in the sensor frame (azimuth counter-clockwise from +x about +z,
elevation up from the x-y plane). The lidar sensor walks it sequentially, so
consecutive frames continue the pattern instead of repeating it — which is what
makes a Livox scan fill in over time. Select one with the sensor's `scan_pattern`
param, or point `scan_pattern_file` at your own.

| file | sensor | points | azimuth | elevation |
|---|---|---|---|---|
| `mid360.npy` | Livox Mid-360 | 800,000 | 0 to 2π | −0.126 to +0.910 rad (−7.2° to +52.2°) |

At the Mid-360's 200,000 points/s, 800,000 points is a 4 s cycle before the
pattern repeats.

## Provenance

`mid360.npy` is copied unmodified from
[discoverse-dev/MuJoCo-LiDAR](https://github.com/discoverse-dev/MuJoCo-LiDAR)
(`src/mujoco_lidar/scan_mode/mid360.npy`, commit `e6b879d`), MIT licensed —
see `LICENSE-MuJoCo-LiDAR`.
