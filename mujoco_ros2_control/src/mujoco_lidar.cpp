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

#include "mujoco_ros2_control/mujoco_lidar.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <utility>

namespace mujoco_ros2_control
{
namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

// Reads an (N, 2) little-endian float32/float64 .npy into flattened float pairs.
// Only what a scan pattern needs; anything else is rejected with a message.
bool load_npy_pattern(
  const std::string &path, std::vector<float> &out, size_t &rows, std::string &error)
{
  std::ifstream file(path, std::ios::binary);
  if (!file)
  {
    error = "cannot open scan pattern file '" + path + "'";
    return false;
  }
  std::vector<char> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  static const char kMagic[] = "\x93NUMPY";
  if (bytes.size() < 10 || std::memcmp(bytes.data(), kMagic, 6) != 0)
  {
    error = "'" + path + "' is not a .npy file";
    return false;
  }
  const uint8_t major = static_cast<uint8_t>(bytes[6]);
  size_t header_len = 0;
  size_t data_offset = 0;
  if (major == 1)
  {
    header_len = static_cast<uint8_t>(bytes[8]) | (static_cast<uint8_t>(bytes[9]) << 8);
    data_offset = 10 + header_len;
  }
  else if ((major == 2 || major == 3) && bytes.size() >= 12)
  {
    header_len = static_cast<uint8_t>(bytes[8]) | (static_cast<uint8_t>(bytes[9]) << 8) |
                 (static_cast<uint8_t>(bytes[10]) << 16) | (static_cast<uint8_t>(bytes[11]) << 24);
    data_offset = 12 + header_len;
  }
  else
  {
    error = "'" + path + "': unsupported .npy version " + std::to_string(major);
    return false;
  }
  if (data_offset > bytes.size())
  {
    error = "'" + path + "': truncated .npy header";
    return false;
  }
  const std::string header(bytes.data() + (data_offset - header_len), header_len);

  size_t item_size = 0;
  if (header.find("'<f4'") != std::string::npos)
  {
    item_size = 4;
  }
  else if (header.find("'<f8'") != std::string::npos)
  {
    item_size = 8;
  }
  else
  {
    error = "'" + path + "': dtype must be little-endian float32 or float64";
    return false;
  }
  if (header.find("'fortran_order': False") == std::string::npos)
  {
    error = "'" + path + "': Fortran-ordered arrays are not supported";
    return false;
  }
  const size_t shape_pos = header.find("'shape': (");
  if (shape_pos == std::string::npos)
  {
    error = "'" + path + "': no shape in .npy header";
    return false;
  }
  unsigned long n_rows = 0;
  unsigned long n_cols = 0;
  if (std::sscanf(header.c_str() + shape_pos, "'shape': (%lu, %lu)", &n_rows, &n_cols) != 2 ||
      n_cols != 2 || n_rows == 0)
  {
    error = "'" + path + "': scan pattern must have shape (N, 2) of (azimuth, elevation)";
    return false;
  }
  if (bytes.size() - data_offset < n_rows * 2 * item_size)
  {
    error = "'" + path + "': file shorter than its declared shape";
    return false;
  }

  out.resize(n_rows * 2);
  const char *data = bytes.data() + data_offset;
  for (size_t i = 0; i < out.size(); ++i)
  {
    if (item_size == 4)
    {
      float v;
      std::memcpy(&v, data + i * 4, 4);
      out[i] = v;
    }
    else
    {
      double v;
      std::memcpy(&v, data + i * 8, 8);
      out[i] = static_cast<float>(v);
    }
  }
  rows = n_rows;
  return true;
}

std::string sanitize_node_name(const std::string &name)
{
  std::string out = name;
  for (auto &c : out)
  {
    if (!std::isalnum(static_cast<unsigned char>(c)))
    {
      c = '_';
    }
  }
  return out;
}

// True when `candidate` is a new minimum; NaN stands for "no return yet".
bool is_new_min(double current, double candidate)
{
  return std::isnan(current) || candidate < current;
}
}  // namespace

MujocoLidar::MujocoLidar(const LidarConfig &config) : config_(config), rng_(config.seed) {}

MujocoLidar::~MujocoLidar()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  cv_.notify_one();
  if (publish_thread_.joinable())
  {
    publish_thread_.join();
  }
  if (node_ && dropped_frames_ > 0)
  {
    RCLCPP_WARN(
      node_->get_logger(), "lidar '%s' dropped %lu frames: the publisher could not keep up",
      config_.name.c_str(), static_cast<unsigned long>(dropped_frames_));
  }
}

bool MujocoLidar::init(const mjModel *mujoco_model, std::string &error)
{
  if (!(config_.frame_rate > 0.0) || config_.points_per_frame <= 0)
  {
    error = "frame_rate and points_per_frame must be positive";
    return false;
  }
  if (!(config_.min_range >= 0.0) || !(config_.max_range > config_.min_range))
  {
    error = "ranges must satisfy 0 <= min_range < max_range";
    return false;
  }
  if (config_.state_sectors < 0 || !(config_.range_noise_stddev >= 0.0))
  {
    error = "state_sectors and range_noise_stddev must not be negative";
    return false;
  }

  site_id_ = mj_name2id(mujoco_model, mjOBJ_SITE, config_.site_name.c_str());
  if (site_id_ == -1)
  {
    error = "no MuJoCo site named '" + config_.site_name + "'";
    return false;
  }
  // The sensor housing sits inside the geometry of the body it is mounted on (on the
  // G1 the Mid-360 origin is inside the torso mesh, so every ray would otherwise hit
  // it from the inside at a few centimetres). That body is excluded from the cast.
  mount_body_id_ = mujoco_model->site_bodyid[site_id_];
  robot_root_id_ = mujoco_model->body_rootid[mount_body_id_];

  if (!load_npy_pattern(config_.pattern_file, pattern_, pattern_size_, error))
  {
    return false;
  }

  frame_period_ = 1.0 / config_.frame_rate;
  point_rate_ = static_cast<double>(config_.points_per_frame) * config_.frame_rate;

  const size_t n = static_cast<size_t>(config_.points_per_frame);
  local_dirs_.resize(3 * n);
  world_dirs_.resize(3 * n);
  dist_.resize(n);
  geom_id_.resize(n);
  frame_points_.reserve(n);
  pending_points_.reserve(n);
  frame_sector_min_.assign(static_cast<size_t>(config_.state_sectors),
                           std::numeric_limits<double>::quiet_NaN());
  state_.sector_min_range.assign(static_cast<size_t>(config_.state_sectors),
                                 std::numeric_limits<double>::quiet_NaN());

  // Publishers need no executor, so this node is never spun.
  node_ = rclcpp::Node::make_shared("mujoco_lidar_" + sanitize_node_name(config_.name));
  // Reliable by default because a reliable publisher can feed both reliable and
  // best-effort subscribers, while a best-effort publisher is INVISIBLE to a
  // reliable one. Set best_effort for sensor-style delivery: a late frame is
  // dropped rather than retransmitted, which keeps a slow consumer from
  // back-pressuring a 10 Hz cloud.
  rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(config_.qos_depth)));
  config_.best_effort ? qos.best_effort() : qos.reliable();
  publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(config_.topic, qos);
  publish_thread_ = std::thread(&MujocoLidar::publish_loop, this);

  RCLCPP_INFO(
    node_->get_logger(),
    "lidar '%s' on site '%s': %d points/frame at %.1f Hz, range [%.2f, %.2f] m, %zu-point "
    "pattern, publishing %s (%s, depth %d) in frame '%s'",
    config_.name.c_str(), config_.site_name.c_str(), config_.points_per_frame, config_.frame_rate,
    config_.min_range, config_.max_range, pattern_size_, publisher_->get_topic_name(),
    config_.best_effort ? "best effort" : "reliable", config_.qos_depth,
    config_.frame_id.c_str());
  return true;
}

void MujocoLidar::begin_frame(double start_time)
{
  frame_start_ = start_time;
  cast_count_ = 0;
  frame_points_.clear();
  frame_min_range_ = std::numeric_limits<double>::quiet_NaN();
  std::fill(
    frame_sector_min_.begin(), frame_sector_min_.end(), std::numeric_limits<double>::quiet_NaN());
}

void MujocoLidar::update(const mjModel *mujoco_model, mjData *mujoco_data)
{
  const double now = mujoco_data->time;
  // Time going backwards means the simulation was reset: start over.
  if (!started_ || now < last_time_)
  {
    begin_frame(now);
    started_ = true;
  }
  last_time_ = now;

  const size_t n = static_cast<size_t>(config_.points_per_frame);
  while (now - frame_start_ >= frame_period_)
  {
    cast_until(mujoco_model, mujoco_data, n, now);
    finish_frame();
    double next_start = frame_start_ + frame_period_;
    // More than a whole frame behind (a long pause between calls): resynchronise
    // instead of emitting a burst of frames all cast from the same pose.
    if (now - next_start >= frame_period_)
    {
      next_start = now;
    }
    begin_frame(next_start);
  }

  const size_t due =
    std::min(n, static_cast<size_t>((now - frame_start_) * point_rate_) + 1);
  cast_until(mujoco_model, mujoco_data, due, now);
}

void MujocoLidar::cast_until(
  const mjModel *mujoco_model, mjData *mujoco_data, size_t due, double now)
{
  if (due <= cast_count_)
  {
    return;
  }
  const size_t count = due - cast_count_;
  const mjtNum *origin = mujoco_data->site_xpos + 3 * site_id_;
  const mjtNum *rot = mujoco_data->site_xmat + 9 * site_id_;

  for (size_t k = 0; k < count; ++k)
  {
    const size_t idx = (pattern_cursor_ + k) % pattern_size_;
    const double azimuth = pattern_[2 * idx];
    const double elevation = pattern_[2 * idx + 1];
    const double ce = std::cos(elevation);
    mjtNum *local = local_dirs_.data() + 3 * k;
    local[0] = ce * std::cos(azimuth);
    local[1] = ce * std::sin(azimuth);
    local[2] = std::sin(elevation);
    mju_mulMatVec(world_dirs_.data() + 3 * k, rot, local, 3, 3);
  }

  // flg_static = 1: world geoms (floor, stairs, walls) are what a lidar is for.
  mj_multiRay(
    mujoco_model, mujoco_data, origin, world_dirs_.data(), nullptr, 1, mount_body_id_,
    geom_id_.data(), dist_.data(), nullptr, static_cast<int>(count), config_.max_range);

  const double stamp_ns = now * 1e9;
  const double sector_scale = static_cast<double>(config_.state_sectors) / kTwoPi;
  for (size_t k = 0; k < count; ++k)
  {
    const int geom = geom_id_[k];
    if (geom < 0 || dist_[k] < 0.0)
    {
      continue;
    }
    // A ray that hits the robot itself is still blocked by it - the occlusion is
    // real - but the return is dropped, as a real sensor's self-filter would.
    if (config_.filter_robot_hits &&
        mujoco_model->body_rootid[mujoco_model->geom_bodyid[geom]] == robot_root_id_)
    {
      continue;
    }
    double range = dist_[k];
    if (config_.range_noise_stddev > 0.0)
    {
      range += config_.range_noise_stddev * noise_(rng_);
    }
    if (range < config_.min_range || range > config_.max_range)
    {
      continue;
    }

    const mjtNum *local = local_dirs_.data() + 3 * k;
    LidarPoint point;
    point.x = static_cast<float>(local[0] * range);
    point.y = static_cast<float>(local[1] * range);
    point.z = static_cast<float>(local[2] * range);
    point.intensity = config_.intensity;
    point.tag = 0;
    point.line = 0;
    point.timestamp = stamp_ns;
    frame_points_.push_back(point);

    if (is_new_min(frame_min_range_, range))
    {
      frame_min_range_ = range;
    }
    if (config_.state_sectors > 0)
    {
      const size_t idx = (pattern_cursor_ + k) % pattern_size_;
      double azimuth = std::fmod(static_cast<double>(pattern_[2 * idx]), kTwoPi);
      if (azimuth < 0.0)
      {
        azimuth += kTwoPi;
      }
      const size_t sector = std::min(
        static_cast<size_t>(azimuth * sector_scale), frame_sector_min_.size() - 1);
      if (is_new_min(frame_sector_min_[sector], range))
      {
        frame_sector_min_[sector] = range;
      }
    }
  }

  pattern_cursor_ = (pattern_cursor_ + count) % pattern_size_;
  cast_count_ = due;
}

void MujocoLidar::finish_frame()
{
  state_.frame_count += 1.0;
  state_.stamp = frame_start_;
  state_.num_points = static_cast<double>(frame_points_.size());
  state_.min_range = frame_min_range_;
  std::copy(frame_sector_min_.begin(), frame_sector_min_.end(), state_.sector_min_range.begin());

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_pending_)
    {
      ++dropped_frames_;
    }
    std::swap(frame_points_, pending_points_);
    pending_stamp_ = frame_start_;
    has_pending_ = true;
  }
  cv_.notify_one();
}

void MujocoLidar::publish_loop()
{
  std::vector<LidarPoint> points;
  points.reserve(static_cast<size_t>(config_.points_per_frame));

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = config_.frame_id;
  msg.height = 1;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = sizeof(LidarPoint);
  const auto add_field = [&msg](const char *name, uint32_t offset, uint8_t datatype)
  {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    msg.fields.push_back(field);
  };
  add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
  add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
  add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
  add_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
  add_field("tag", 16, sensor_msgs::msg::PointField::UINT8);
  add_field("line", 17, sensor_msgs::msg::PointField::UINT8);
  add_field("timestamp", 18, sensor_msgs::msg::PointField::FLOAT64);

  while (true)
  {
    double stamp = 0.0;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return stop_ || has_pending_; });
      if (stop_)
      {
        return;
      }
      std::swap(points, pending_points_);
      stamp = pending_stamp_;
      has_pending_ = false;
    }

    const auto sec = static_cast<int32_t>(std::floor(stamp));
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = static_cast<uint32_t>((stamp - sec) * 1e9);
    msg.width = static_cast<uint32_t>(points.size());
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);
    if (!points.empty())
    {
      std::memcpy(msg.data.data(), points.data(), msg.row_step);
    }
    publisher_->publish(msg);
  }
}

}  // namespace mujoco_ros2_control
