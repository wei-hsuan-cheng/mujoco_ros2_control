cmake_minimum_required(VERSION 3.5)
project(mujoco_ros2_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MujocoExternalWrench.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)


set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  hardware_interface
  pluginlib
  controller_manager
  joint_limits
  urdf
  glfw3
  control_toolbox
  std_srvs
  tf2_ros
  geometry_msgs
  std_msgs
  nav_msgs
)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# MuJoCo resolution order:
#   1. a source-installed mujoco (find_package)
#   2. a prebuilt tree pointed to by the MUJOCO_DIR environment variable
#   3. automatic download of the prebuilt release into <pkg>/lib/ (Linux only);
#      the download is cached there, so it only happens once per version.
set(MUJOCO_VERSION "3.3.7" CACHE STRING
  "MuJoCo prebuilt release downloaded automatically when no other install is found")

# find_library caches its result: drop a cached path whose file no longer exists
# (e.g. a removed MUJOCO_DIR tree) so resolution below runs again.
if(MUJOCO_LIB AND NOT EXISTS ${MUJOCO_LIB})
  message(STATUS "Cached mujoco library ${MUJOCO_LIB} no longer exists; re-resolving.")
  unset(MUJOCO_LIB CACHE)
endif()

find_package(mujoco QUIET)
if(mujoco_FOUND)
  message(STATUS "Mujoco build from source has been found")
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
elseif(DEFINED ENV{MUJOCO_DIR})
  message(STATUS "Mujoco build from source has not been found. Attempting to find the binary in $ENV{MUJOCO_DIR} instead.")
  find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "Failed to find binary in $ENV{MUJOCO_DIR}")
  endif()
  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
else()
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(MUJOCO_ARCH "aarch64")
  else()
    set(MUJOCO_ARCH "x86_64")
  endif()
  # Arch-specific cache folder: the source tree may be shared across machines
  # (e.g. a macOS folder mounted into Linux containers of different arch).
  set(MUJOCO_DOWNLOAD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux-${MUJOCO_ARCH})
  set(MUJOCO_LOCAL_DIR ${MUJOCO_DOWNLOAD_ROOT}/mujoco-${MUJOCO_VERSION})

  if(NOT EXISTS ${MUJOCO_LOCAL_DIR}/lib)
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
      message(FATAL_ERROR "No mujoco install found and the automatic prebuilt download only supports Linux.
  Either build and install mujoco from source or set the MUJOCO_DIR environment variable to tell CMake where to find the binary install. ")
    endif()
    set(MUJOCO_TARBALL mujoco-${MUJOCO_VERSION}-linux-${MUJOCO_ARCH}.tar.gz)
    set(MUJOCO_URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_TARBALL})

    message(STATUS "Mujoco not found; downloading prebuilt ${MUJOCO_URL} into ${MUJOCO_DOWNLOAD_ROOT}")
    file(MAKE_DIRECTORY ${MUJOCO_DOWNLOAD_ROOT})
    file(DOWNLOAD ${MUJOCO_URL} ${MUJOCO_DOWNLOAD_ROOT}/${MUJOCO_TARBALL}
      SHOW_PROGRESS STATUS MUJOCO_DOWNLOAD_STATUS)
    list(GET MUJOCO_DOWNLOAD_STATUS 0 MUJOCO_DOWNLOAD_CODE)
    if(NOT MUJOCO_DOWNLOAD_CODE EQUAL 0)
      list(GET MUJOCO_DOWNLOAD_STATUS 1 MUJOCO_DOWNLOAD_ERROR)
      file(REMOVE ${MUJOCO_DOWNLOAD_ROOT}/${MUJOCO_TARBALL})
      message(FATAL_ERROR "Mujoco download failed: ${MUJOCO_DOWNLOAD_ERROR}
  Download it manually and set the MUJOCO_DIR environment variable instead.")
    endif()

    # The tarball extracts to mujoco-${MUJOCO_VERSION}/; cmake -E tar works on all CMake versions.
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar xzf ${MUJOCO_DOWNLOAD_ROOT}/${MUJOCO_TARBALL}
      WORKING_DIRECTORY ${MUJOCO_DOWNLOAD_ROOT}
      RESULT_VARIABLE MUJOCO_EXTRACT_CODE)
    file(REMOVE ${MUJOCO_DOWNLOAD_ROOT}/${MUJOCO_TARBALL})
    if(NOT MUJOCO_EXTRACT_CODE EQUAL 0)
      message(FATAL_ERROR "Failed to extract ${MUJOCO_TARBALL}")
    endif()
  else()
    message(STATUS "Using cached mujoco prebuilt at ${MUJOCO_LOCAL_DIR}")
  endif()

  find_library(MUJOCO_LIB mujoco HINTS ${MUJOCO_LOCAL_DIR}/lib)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "Downloaded mujoco at ${MUJOCO_LOCAL_DIR} is missing its library.
  Delete that folder to retry the download, or set the MUJOCO_DIR environment variable.")
  endif()
  set(MUJOCO_INCLUDE_DIR ${MUJOCO_LOCAL_DIR}/include)
endif()

add_library(mujoco_system_plugins SHARED src/mujoco_system.cpp)
ament_target_dependencies(mujoco_system_plugins ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_system_plugins ${MUJOCO_LIB})
target_include_directories(mujoco_system_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  mujoco_system_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# TODO: make it simple
add_executable(mujoco_ros2_control_node
  src/mujoco_ros2_control_node.cpp
  src/mujoco_rendering.cpp
  src/mujoco_ros2_control.cpp
  src/mujoco_cameras.cpp
)
set_target_properties(mujoco_ros2_control_node
  PROPERTIES OUTPUT_NAME mujoco_ros2_control)
ament_target_dependencies(mujoco_ros2_control_node ${THIS_PACKAGE_DEPENDS})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mujoco_ros2_control_node ${MUJOCO_LIB} glfw "${cpp_typesupport_target}")
target_include_directories(mujoco_ros2_control_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  mujoco_ros2_control_node
  DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_system_plugins.xml)
ament_package()
