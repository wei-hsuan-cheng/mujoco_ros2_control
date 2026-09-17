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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_LIDAR_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_LIDAR_HPP_

#include <condition_variable>
#include <cstdint>
#include <limits>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mujoco_ros2_control
{

// One return, laid out exactly like livox_ros_driver2's PointCloud2 point
// (LivoxPointXyzrtlt, packed): consumers written for a real Livox accept it as-is.
#pragma pack(push, 1)
struct LidarPoint
{
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;  // absolute time of the return [ns], as the Livox driver writes it
};
#pragma pack(pop)

struct LidarConfig
{
  std::string name;          // ros2_control sensor name
  std::string site_name;     // MuJoCo site the sensor is mounted on; rays leave its origin
  std::string pattern_file;  // (N, 2) float .npy of (azimuth, elevation) [rad], in firing order
  std::string frame_id;
  std::string topic;
  double frame_rate{10.0};
  int points_per_frame{20000};
  double min_range{0.1};
  double max_range{40.0};
  double range_noise_stddev{0.0};
  unsigned int seed{0};
  bool filter_robot_hits{true};
  float intensity{100.0F};
  int state_sectors{0};
};

// Scanning 3D lidar driven by a recorded scan pattern (e.g. the Livox Mid-360's
// non-repetitive rosette).
//
// Rays are cast progressively as simulation time advances rather than all at the
// frame boundary: each call casts the points whose firing time has come, from the
// sensor pose at that moment. A walking robot therefore gets the same motion
// distortion a real 100 ms sweep has, each point carries the time it was fired, and
// the cost is spread evenly over the control loop instead of spiking once a frame.
//
// update() runs on the simulation thread between mj_step1 and mj_step2, where
// mjData is consistent. Completed frames are handed to a publisher thread so
// serialisation and DDS never stall physics.
class MujocoLidar
{
public:
  // Scalars about the latest COMPLETED frame, exported as ros2_control state
  // interfaces. The full cloud is published as PointCloud2; it cannot be a set of
  // double interfaces, for the same reason camera images are not.
  struct State
  {
    double frame_count{0.0};
    double stamp{0.0};       // simulation time the frame started [s]
    double num_points{0.0};  // returns kept in the frame
    double min_range{std::numeric_limits<double>::quiet_NaN()};  // NaN: no return
    // Nearest return per azimuth sector about the sensor z axis, sector i covering
    // [2*pi*i/N, 2*pi*(i+1)/N) counter-clockwise from +x. NaN: no return. A coarse
    // proximity ring a controller can consume through the state interfaces.
    std::vector<double> sector_min_range;
  };

  explicit MujocoLidar(const LidarConfig &config);
  ~MujocoLidar();

  MujocoLidar(const MujocoLidar &) = delete;
  MujocoLidar &operator=(const MujocoLidar &) = delete;

  bool init(const mjModel *mujoco_model, std::string &error);
  void update(const mjModel *mujoco_model, mjData *mujoco_data);

  State &state() { return state_; }
  const LidarConfig &config() const { return config_; }

private:
  void begin_frame(double start_time);
  void cast_until(const mjModel *mujoco_model, mjData *mujoco_data, size_t due, double now);
  void finish_frame();
  void publish_loop();

  LidarConfig config_;
  State state_;

  // Pattern, flattened (azimuth, elevation) pairs.
  std::vector<float> pattern_;
  size_t pattern_size_{0};
  size_t pattern_cursor_{0};

  int site_id_{-1};
  int mount_body_id_{-1};
  int robot_root_id_{-1};

  double frame_period_{0.1};
  double point_rate_{200000.0};
  double frame_start_{0.0};
  double last_time_{0.0};
  size_t cast_count_{0};
  bool started_{false};

  // Per-slice scratch, sized once for the worst case (a whole frame in one call).
  std::vector<mjtNum> local_dirs_;
  std::vector<mjtNum> world_dirs_;
  std::vector<mjtNum> dist_;
  std::vector<int> geom_id_;

  // Frame accumulation on the simulation thread.
  std::vector<LidarPoint> frame_points_;
  double frame_min_range_{std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> frame_sector_min_;

  std::mt19937 rng_;
  std::normal_distribution<double> noise_{0.0, 1.0};

  // Hand-off to the publisher thread. Three buffers, all reserved to a full frame,
  // are rotated by swap so the simulation thread never allocates.
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::thread publish_thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<LidarPoint> pending_points_;
  double pending_stamp_{0.0};
  bool has_pending_{false};
  bool stop_{false};
  uint64_t dropped_frames_{0};
};

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_LIDAR_HPP_
