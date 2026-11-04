// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#include "mujoco_ros2_control_multienv/lockstep_bridge_controller.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

namespace mujoco_ros2_control_multienv
{

namespace
{

constexpr std::size_t align64(std::size_t n) { return (n + 63) & ~static_cast<std::size_t>(63); }

}  // namespace

LockstepBridgeController::~LockstepBridgeController() { unmap_shared_memory(); }

controller_interface::CallbackReturn LockstepBridgeController::on_init()
{
  auto_declare<std::string>("shm_name", "");
  auto_declare<double>("step_period", 0.01);
  auto_declare<std::vector<std::string>>("state_interfaces", {});
  auto_declare<std::vector<std::string>>("command_interfaces", {});
  auto_declare<std::vector<double>>("command_defaults", {});
  auto_declare<std::string>("order_probe", "");
  auto_declare<double>("wait_log_period", 5.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LockstepBridgeController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, commanded_interfaces_};
}

controller_interface::InterfaceConfiguration LockstepBridgeController::state_interface_configuration() const
{
  auto names = observed_interfaces_;
  if (!order_probe_.empty() && probe_index_ == observed_interfaces_.size()) {
    names.push_back(order_probe_);  // claimed after the observed ones unless it is one of them
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn LockstepBridgeController::on_configure(const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  shm_name_ = node->get_parameter("shm_name").as_string();
  step_period_ = node->get_parameter("step_period").as_double();
  observed_interfaces_ = node->get_parameter("state_interfaces").as_string_array();
  commanded_interfaces_ = node->get_parameter("command_interfaces").as_string_array();
  command_defaults_ = node->get_parameter("command_defaults").as_double_array();
  order_probe_ = node->get_parameter("order_probe").as_string();
  wait_log_period_ = node->get_parameter("wait_log_period").as_double();

  if (shm_name_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "shm_name is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (shm_name_.front() != '/') {
    shm_name_ = "/" + shm_name_;
  }
  const double rate = static_cast<double>(get_update_rate());
  step_ticks_ = std::llround(step_period_ * rate);
  if (rate <= 0.0 || step_ticks_ < 1 || std::abs(static_cast<double>(step_ticks_) / rate - step_period_) > 1e-9) {
    RCLCPP_ERROR(node->get_logger(), "step_period %.6f s must be a whole number of control ticks at %.1f Hz",
                 step_period_, rate);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (command_defaults_.empty()) {
    command_defaults_.assign(commanded_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  } else if (command_defaults_.size() != commanded_interfaces_.size()) {
    RCLCPP_ERROR(node->get_logger(), "command_defaults needs one value per command_interfaces entry (or none)");
    return controller_interface::CallbackReturn::ERROR;
  }
  // The controller manager refuses duplicate claims: reuse an observed probe.
  probe_index_ = static_cast<std::size_t>(
    std::find(observed_interfaces_.begin(), observed_interfaces_.end(), order_probe_) - observed_interfaces_.begin());
  if (!map_shared_memory()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(node->get_logger(),
              "lockstep channel /dev/shm%s: %zu observations, %zu actions, one exchange every %ld ticks (%.4f s)",
              shm_name_.c_str(), observed_interfaces_.size(), commanded_interfaces_.size(), step_ticks_, step_period_);
  return controller_interface::CallbackReturn::SUCCESS;
}

bool LockstepBridgeController::map_shared_memory()
{
  unmap_shared_memory();
  std::size_t names_bytes = 0;
  for (const auto * list : {&observed_interfaces_, &commanded_interfaces_}) {
    for (const auto & name : *list) {
      names_bytes += name.size() + 1;
    }
  }
  const std::size_t names_offset = lockstep::kHeaderBytes;
  const std::size_t obs_offset = align64(names_offset + names_bytes);
  const std::size_t act_offset = align64(obs_offset + sizeof(double) * observed_interfaces_.size());
  shm_bytes_ = align64(act_offset + sizeof(double) * commanded_interfaces_.size());

  shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0600);
  if (shm_fd_ < 0 || ftruncate(shm_fd_, static_cast<off_t>(shm_bytes_)) != 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "cannot create /dev/shm%s: %s", shm_name_.c_str(), std::strerror(errno));
    unmap_shared_memory();
    return false;
  }
  shm_ = mmap(nullptr, shm_bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
  if (shm_ == MAP_FAILED) {
    shm_ = nullptr;
    RCLCPP_ERROR(get_node()->get_logger(), "cannot map /dev/shm%s: %s", shm_name_.c_str(), std::strerror(errno));
    unmap_shared_memory();
    return false;
  }
  std::memset(shm_, 0, shm_bytes_);
  auto * base = static_cast<unsigned char *>(shm_);
  header_ = reinterpret_cast<lockstep::Header *>(base);
  obs_ = reinterpret_cast<double *>(base + obs_offset);
  act_ = reinterpret_cast<double *>(base + act_offset);
  char * names = reinterpret_cast<char *>(base + names_offset);
  for (const auto * list : {&observed_interfaces_, &commanded_interfaces_}) {
    for (const auto & name : *list) {
      std::memcpy(names, name.c_str(), name.size() + 1);
      names += name.size() + 1;
    }
  }
  std::fill(obs_, obs_ + observed_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  std::copy(command_defaults_.begin(), command_defaults_.end(), act_);

  if (sem_init(lockstep::obs_ready(header_), 1, 0) != 0 || sem_init(lockstep::act_ready(header_), 1, 0) != 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "sem_init failed: %s", std::strerror(errno));
    unmap_shared_memory();
    return false;
  }
  semaphores_initialized_ = true;
  header_->version = lockstep::kVersion;
  header_->status = lockstep::kInitializing;
  header_->num_obs = static_cast<std::uint32_t>(observed_interfaces_.size());
  header_->num_act = static_cast<std::uint32_t>(commanded_interfaces_.size());
  header_->names_offset = static_cast<std::uint32_t>(names_offset);
  header_->names_bytes = static_cast<std::uint32_t>(names_bytes);
  header_->obs_offset = static_cast<std::uint32_t>(obs_offset);
  header_->act_offset = static_cast<std::uint32_t>(act_offset);
  header_->bridge_pid = static_cast<std::int32_t>(getpid());
  header_->step_period = step_period_;
  header_->update_rate = static_cast<double>(get_update_rate());
  // Written last: a client treats the segment as valid only once the magic is there.
  __atomic_store_n(&header_->magic, lockstep::kMagic, __ATOMIC_RELEASE);
  return true;
}

void LockstepBridgeController::unmap_shared_memory()
{
  if (header_ && semaphores_initialized_) {
    __atomic_store_n(&header_->status, lockstep::kClosed, __ATOMIC_RELEASE);
    sem_post(lockstep::obs_ready(header_));  // release a client waiting for this step
    sem_destroy(lockstep::obs_ready(header_));
    sem_destroy(lockstep::act_ready(header_));
  }
  semaphores_initialized_ = false;
  if (shm_) {
    munmap(shm_, shm_bytes_);
    shm_ = nullptr;
  }
  if (shm_fd_ >= 0) {
    close(shm_fd_);
    shm_fd_ = -1;
    shm_unlink(shm_name_.c_str());
  }
  header_ = nullptr;
  obs_ = nullptr;
  act_ = nullptr;
}

controller_interface::CallbackReturn LockstepBridgeController::on_activate(const rclcpp_lifecycle::State &)
{
  if (!header_) {
    RCLCPP_ERROR(get_node()->get_logger(), "activated without a shared-memory channel");
    return controller_interface::CallbackReturn::ERROR;
  }
  const bool probe_claimed_extra = !order_probe_.empty() && probe_index_ == observed_interfaces_.size();
  if (command_interfaces_.size() != commanded_interfaces_.size() ||
      state_interfaces_.size() != observed_interfaces_.size() + (probe_claimed_extra ? 1 : 0)) {
    RCLCPP_ERROR(get_node()->get_logger(), "claimed %zu state / %zu command interfaces, configured %zu / %zu",
                 state_interfaces_.size(), command_interfaces_.size(), observed_interfaces_.size(),
                 commanded_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  // A previous activation may have left a post behind; start from a clean exchange.
  while (sem_trywait(lockstep::obs_ready(header_)) == 0) {
  }
  while (sem_trywait(lockstep::act_ready(header_)) == 0) {
  }
  commands_ = command_defaults_;
  write_commands();
  step_ = 0;
  order_known_ = order_probe_.empty();
  handshake_offset_ = 0;
  header_->handshake_tick_offset = 0;
  header_->sync_errors = 0;
  __atomic_store_n(&header_->status, lockstep::kRunning, __ATOMIC_RELEASE);
  RCLCPP_INFO(get_node()->get_logger(),
              "lockstep bridge active: the controller manager blocks at every step until a client answers on "
              "/dev/shm%s", shm_name_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LockstepBridgeController::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (header_) {
    __atomic_store_n(&header_->status, lockstep::kClosed, __ATOMIC_RELEASE);
    sem_post(lockstep::obs_ready(header_));
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LockstepBridgeController::on_cleanup(const rclcpp_lifecycle::State &)
{
  unmap_shared_memory();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LockstepBridgeController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // A zero period is the controller manager stepping without time passing (MuJoCo waiting to
  // start): nothing to exchange yet.
  if (period.seconds() <= 0.0) {
    write_commands();
    return controller_interface::return_type::OK;
  }
  const double t = time.seconds();
  const double rate = static_cast<double>(get_update_rate());
  const int64_t tick = std::llround(t * rate);
  if (!order_known_) {
    const auto probe = state_interfaces_[probe_index_].get_optional();
    const bool probe_ran_first = probe && std::isfinite(*probe) && std::abs(*probe - t) < 0.5 / rate;
    handshake_offset_ = probe_ran_first ? -1 : 0;
    header_->handshake_tick_offset = static_cast<std::int32_t>(handshake_offset_);
    order_known_ = true;
    RCLCPP_INFO(get_node()->get_logger(),
                "%s updates %s this bridge: exchanges happen %s, so the answer is in effect on the step tick",
                order_probe_.c_str(), probe_ran_first ? "before" : "after",
                probe_ran_first ? "one tick before each step tick" : "on each step tick");
  }
  const int64_t phase = ((tick - handshake_offset_) % step_ticks_ + step_ticks_) % step_ticks_;
  if (phase == 0 && !exchange(t)) {
    return controller_interface::return_type::OK;  // shutting down
  }
  write_commands();
  return controller_interface::return_type::OK;
}

bool LockstepBridgeController::exchange(double time)
{
  for (std::size_t i = 0; i < observed_interfaces_.size(); ++i) {
    const auto value = state_interfaces_[i].get_optional();
    obs_[i] = value ? *value : std::numeric_limits<double>::quiet_NaN();
  }
  header_->obs_time = time;
  header_->obs_step = step_;
  sem_post(lockstep::obs_ready(header_));

  using Clock = std::chrono::steady_clock;
  const auto wait_start = Clock::now();
  auto next_log = wait_start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(wait_log_period_));
  while (true) {
    timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_nsec += 100000000;  // 100 ms, to notice a shutdown
    if (deadline.tv_nsec >= 1000000000) {
      deadline.tv_nsec -= 1000000000;
      ++deadline.tv_sec;
    }
    if (sem_timedwait(lockstep::act_ready(header_), &deadline) == 0) {
      break;
    }
    if (errno != ETIMEDOUT && errno != EINTR) {
      RCLCPP_ERROR(get_node()->get_logger(), "sem_timedwait: %s", std::strerror(errno));
      return false;
    }
    if (!rclcpp::ok()) {
      return false;
    }
    if (Clock::now() >= next_log) {
      // The node clock is controller time, frozen here, so this cannot use the throttled macros.
      RCLCPP_WARN(get_node()->get_logger(), "step %lu at t=%.3f s: waiting %.0f s for the client on /dev/shm%s",
                  static_cast<unsigned long>(step_), time,
                  std::chrono::duration<double>(Clock::now() - wait_start).count(), shm_name_.c_str());
      next_log += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(wait_log_period_));
    }
  }
  if (header_->act_step != step_) {
    ++header_->sync_errors;
    RCLCPP_ERROR(get_node()->get_logger(), "client answered step %lu while step %lu was pending",
                 static_cast<unsigned long>(header_->act_step), static_cast<unsigned long>(step_));
  }
  std::copy(act_, act_ + commanded_interfaces_.size(), commands_.begin());
  ++step_;
  return true;
}

void LockstepBridgeController::write_commands()
{
  for (std::size_t i = 0; i < command_interfaces_.size() && i < commands_.size(); ++i) {
    (void)command_interfaces_[i].set_value(commands_[i]);
  }
}

}  // namespace mujoco_ros2_control_multienv

PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_control_multienv::LockstepBridgeController, controller_interface::ControllerInterface)
