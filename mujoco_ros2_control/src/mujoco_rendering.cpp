// Copyright (c) 2025 Sangtaek Lee
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

#include <cstdio>
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace mujoco_ros2_control
{
MujocoRendering *MujocoRendering::instance_ = nullptr;

MujocoRendering *MujocoRendering::get_instance()
{
  if (instance_ == nullptr)
  {
    instance_ = new MujocoRendering();
  }

  return instance_;
}

MujocoRendering::MujocoRendering()
    : mj_model_(nullptr),
      mj_data_(nullptr),
      button_left_(false),
      button_middle_(false),
      button_right_(false),
      lastx_(0.0),
      lasty_(0.0)
{
}

void MujocoRendering::set_camera_by_id(int cam_id)
{
  current_cam_id_ = cam_id;

  if (cam_id < 0) {
    mjv_cam_.type = mjCAMERA_FREE;
    return;
  }

  mjv_cam_.type = mjCAMERA_FIXED;
  mjv_cam_.fixedcamid = cam_id;

  // Optional: print which camera you switched to
  const char* nm = mj_id2name(mj_model_, mjOBJ_CAMERA, cam_id);
  if (nm) {
    std::printf("[mujoco_rendering] switched to camera: %s (id=%d)\n", nm, cam_id);
  }
}


void MujocoRendering::set_tracking(bool on)
{
  if (!on) {
    // Keep the view where the tracking camera left it (mjv_updateScene writes the tracked lookat back).
    current_cam_id_ = -1;
    mjv_cam_.type = mjCAMERA_FREE;
    std::printf("[mujoco_rendering] free camera\n");
    return;
  }
  int body = track_body_name_.empty() ? -1 : mj_name2id(mj_model_, mjOBJ_BODY, track_body_name_.c_str());
  for (int b = 1; body < 0 && b < mj_model_->nbody; ++b) {
    // default: the first floating body (the robot root); its subtree centre of mass is the robot's CoM
    const int j = mj_model_->body_jntadr[b];
    if (j >= 0 && mj_model_->jnt_type[j] == mjJNT_FREE) body = b;
  }
  if (body < 0) {
    std::printf("[mujoco_rendering] no body to track ('%s' not found, no floating body)\n", track_body_name_.c_str());
    return;
  }
  current_cam_id_ = -1;
  mjv_cam_.type = mjCAMERA_TRACKING;
  mjv_cam_.trackbodyid = body;
  mjv_cam_.distance = track_distance_;
  std::printf("[mujoco_rendering] tracking the centre of mass of body '%s' (T toggles)\n",
              mj_id2name(mj_model_, mjOBJ_BODY, body));
}

void MujocoRendering::init(mjModel *mujoco_model, mjData *mujoco_data)
{
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  // create window, make OpenGL context current, request v-sync
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
  window_ = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window_);

  // initialize visualization data structures
  mjv_defaultCamera(&mjv_cam_);
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);

  // Show body frames (including world and end-effector frames)
  mjv_opt_.frame = mjFRAME_NONE; // default: mjFRAME_NONE or mjFRAME_BODY
  // Make frame axes smaller than MuJoCo defaults
  mj_model_->vis.scale.framelength *= 0.5f;
  mj_model_->vis.scale.framewidth *= 0.3f;

  // Initialize viewer camera from model defaults (XML <visual><global/> and <statistic/>)
  // This keeps the camera draggable (free camera), but with a deterministic start pose.
  mjv_cam_.type = mjCAMERA_FREE;
  mjv_cam_.azimuth = mj_model_->vis.global.azimuth;
  mjv_cam_.elevation = mj_model_->vis.global.elevation;
  mjv_cam_.lookat[0] = mj_model_->stat.center[0];
  mjv_cam_.lookat[1] = mj_model_->stat.center[1];
  mjv_cam_.lookat[2] = mj_model_->stat.center[2];
  mjv_cam_.distance = (mj_model_->stat.extent > 1e-9) ? (2.0 * mj_model_->stat.extent) : 5.0;

  // create scene and context
  mjv_makeScene(mj_model_, &mjv_scn_, 2000);
  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window_, &MujocoRendering::keyboard_callback);
  glfwSetCursorPosCallback(window_, &MujocoRendering::mouse_move_callback);
  glfwSetMouseButtonCallback(window_, &MujocoRendering::mouse_button_callback);
  glfwSetScrollCallback(window_, &MujocoRendering::scroll_callback);

  // This might cause tearing, but having RViz and the renderer both open can
  // wreak havoc on the rendering process.
  glfwSwapInterval(0);
}

void MujocoRendering::configure_camera_from_ros_params(const rclcpp::Node::SharedPtr &node)
{
  if (!node || !mj_model_) return;

  // Declare parameters with defaults based on the loaded model.
  // If launch overrides them, those overridden values will be visible via get_parameter().
  if (!node->has_parameter("viewer_camera.mode")) {
    // free | fixed | track; empty: the scene XML's <custom><text name="viewer_camera.mode" data="..."/>, else free
    node->declare_parameter<std::string>("viewer_camera.mode", "");
  }
  if (!node->has_parameter("viewer_camera.body")) {
    // track: body whose subtree CoM is followed; empty: the XML's viewer_camera.body, else the first floating body
    node->declare_parameter<std::string>("viewer_camera.body", "");
  }
  if (!node->has_parameter("viewer_camera.track_distance")) {
    // track: start distance [m]; <= 0: the XML's <numeric name="viewer_camera.track_distance">, else 2 m
    node->declare_parameter<double>("viewer_camera.track_distance", -1.0);
  }
  if (!node->has_parameter("viewer_camera.id")) {
    node->declare_parameter<int>("viewer_camera.id", -1);  // fixed camera id
  }
  if (!node->has_parameter("viewer_camera.name")) {
    node->declare_parameter<std::string>("viewer_camera.name", "");  // fixed camera name
  }
  if (!node->has_parameter("viewer_camera.azimuth")) {
    node->declare_parameter<double>("viewer_camera.azimuth", static_cast<double>(mj_model_->vis.global.azimuth));
  }
  if (!node->has_parameter("viewer_camera.elevation")) {
    node->declare_parameter<double>("viewer_camera.elevation", static_cast<double>(mj_model_->vis.global.elevation));
  }
  if (!node->has_parameter("viewer_camera.distance")) {
    const double dflt = (mj_model_->stat.extent > 1e-9) ? (2.0 * mj_model_->stat.extent) : 5.0;
    node->declare_parameter<double>("viewer_camera.distance", dflt);
  }
  if (!node->has_parameter("viewer_camera.lookat")) {
    node->declare_parameter<std::vector<double>>(
      "viewer_camera.lookat",
      {static_cast<double>(mj_model_->stat.center[0]),
       static_cast<double>(mj_model_->stat.center[1]),
       static_cast<double>(mj_model_->stat.center[2])});
  }

  // A ROS parameter that is set wins; otherwise the scene XML's <custom> entry of the same name; otherwise the default.
  const auto xml_text = [this](const char *name, const std::string &fallback) {
    const int id = mj_name2id(mj_model_, mjOBJ_TEXT, name);
    return id < 0 ? fallback : std::string(mj_model_->text_data + mj_model_->text_adr[id]);
  };
  const auto xml_numeric = [this](const char *name, double fallback) {
    const int id = mj_name2id(mj_model_, mjOBJ_NUMERIC, name);
    return id < 0 || mj_model_->numeric_size[id] < 1 ? fallback : mj_model_->numeric_data[mj_model_->numeric_adr[id]];
  };
  auto mode = node->get_parameter("viewer_camera.mode").as_string();
  if (mode.empty()) mode = xml_text("viewer_camera.mode", "free");
  track_body_name_ = node->get_parameter("viewer_camera.body").as_string();
  if (track_body_name_.empty()) track_body_name_ = xml_text("viewer_camera.body", "");
  track_distance_ = node->get_parameter("viewer_camera.track_distance").as_double();
  if (track_distance_ <= 0.0) track_distance_ = xml_numeric("viewer_camera.track_distance", 2.0);
  const auto cam_name = node->get_parameter("viewer_camera.name").as_string();
  int cam_id = node->get_parameter("viewer_camera.id").as_int();

  if (mode == "fixed") {
    if (!cam_name.empty()) {
      const int id = mj_name2id(mj_model_, mjOBJ_CAMERA, cam_name.c_str());
      if (id >= 0) cam_id = id;
    }
    set_camera_by_id(cam_id);
    return;
  }

  // Free camera: draggable. Apply initial pose.
  mjv_cam_.type = mjCAMERA_FREE;
  mjv_cam_.azimuth = node->get_parameter("viewer_camera.azimuth").as_double();
  mjv_cam_.elevation = node->get_parameter("viewer_camera.elevation").as_double();
  mjv_cam_.distance = node->get_parameter("viewer_camera.distance").as_double();

  const auto lookat = node->get_parameter("viewer_camera.lookat").as_double_array();
  if (lookat.size() == 3) {
    mjv_cam_.lookat[0] = lookat[0];
    mjv_cam_.lookat[1] = lookat[1];
    mjv_cam_.lookat[2] = lookat[2];
  }
  if (mode == "track") {
    set_tracking(true);  // the camera follows the robot; the mouse still orbits and zooms around it
  }
}

bool MujocoRendering::is_close_flag_raised() { return glfwWindowShouldClose(window_); }

void MujocoRendering::update()
{
  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  glfwMakeContextCurrent(window_);

  // Reset the buffer
  mjr_setBuffer(mjFB_WINDOW, &mjr_con_);

  // update scene and render
  mjv_updateScene(mj_model_, mj_data_, &mjv_opt_, NULL, &mjv_cam_, mjCAT_ALL, &mjv_scn_);
  mjr_render(viewport, &mjv_scn_, &mjr_con_);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window_);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

void MujocoRendering::close()
{
  // free visualization storage
  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);
  glfwDestroyWindow(window_);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}

void MujocoRendering::keyboard_callback(
  GLFWwindow *window, int key, int scancode, int act, int mods)
{
  get_instance()->keyboard_callback_impl(window, key, scancode, act, mods);
}

void MujocoRendering::mouse_button_callback(GLFWwindow *window, int button, int act, int mods)
{
  get_instance()->mouse_button_callback_impl(window, button, act, mods);
}

void MujocoRendering::mouse_move_callback(GLFWwindow *window, double xpos, double ypos)
{
  get_instance()->mouse_move_callback_impl(window, xpos, ypos);
}

void MujocoRendering::scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
  get_instance()->scroll_callback_impl(window, xoffset, yoffset);
}

void MujocoRendering::keyboard_callback_impl(
  GLFWwindow * /* window */, int key, int /* scancode */, int act, int /* mods */)
{
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
  {
    mj_resetData(mj_model_, mj_data_);
    mj_forward(mj_model_, mj_data_);
  }

  // 'F' key: toggle body frames on/off
  if (act == GLFW_PRESS && key == GLFW_KEY_F)
  {
    mjv_opt_.frame = (mjv_opt_.frame == mjFRAME_BODY) ? mjFRAME_NONE : mjFRAME_BODY;
  }

  // 'T' key: toggle a camera that follows the robot's centre of mass
  if (act == GLFW_PRESS && key == GLFW_KEY_T) {
    set_tracking(mjv_cam_.type != mjCAMERA_TRACKING);
  }

  // 'C' key: Set camera (cyclic)
  if (act == GLFW_PRESS && key == GLFW_KEY_C) {
    // cycle: free(-1) -> 0 -> 1 -> ... -> ncam-1 -> free(-1)
    if (mj_model_->ncam == 0) return;
    int next = current_cam_id_ + 1;
    if (next >= mj_model_->ncam) next = -1;
    set_camera_by_id(next);
  }

}

void MujocoRendering::mouse_button_callback_impl(
  GLFWwindow *window, int /* button */, int /* act */, int /* mods */)
{
  // update button state
  button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx_, &lasty_);
}

void MujocoRendering::mouse_move_callback_impl(GLFWwindow *window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left_ && !button_middle_ && !button_right_)
  {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx_;
  double dy = ypos - lasty_;
  lastx_ = xpos;
  lasty_ = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift =
    (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right_)
  {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  }
  else if (button_left_)
  {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  else
  {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(mj_model_, action, dx / height, dy / height, &mjv_cam_);
}

void MujocoRendering::scroll_callback_impl(
  GLFWwindow * /* window */, double /* xoffset */, double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &mjv_cam_);
}

}  // namespace mujoco_ros2_control
