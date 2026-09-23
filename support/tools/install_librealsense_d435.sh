#!/usr/bin/env bash
set -euo pipefail

# Install the librealsense SDK used by mono_camera_capture.  Prefer the ROS
# binary so headers, libraries and CMake metadata live under the active ROS
# prefix and cannot conflict with a second /usr/local SDK installation.

TASK_ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${TASK_ROS_DISTRO}/setup.bash"
SDK_PACKAGE="ros-${TASK_ROS_DISTRO}-librealsense2"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS ${TASK_ROS_DISTRO} was not found at ${ROS_SETUP}." >&2
  echo "Source the intended ROS environment or run ROS_DISTRO=<name> $0." >&2
  exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "This installer currently supports Debian/Ubuntu systems with apt." >&2
  exit 1
fi

if [[ "${EUID}" -eq 0 ]]; then
  TASK_SUDO=()
elif command -v sudo >/dev/null 2>&1; then
  TASK_SUDO=(sudo)
  "${TASK_SUDO[@]}" -v
else
  echo "Root privileges are required, but sudo is unavailable." >&2
  exit 1
fi

echo "Installing ${SDK_PACKAGE} for ROS ${TASK_ROS_DISTRO}..."
"${TASK_SUDO[@]}" apt-get update

if ! apt-cache show "${SDK_PACKAGE}" >/dev/null 2>&1; then
  echo "Package ${SDK_PACKAGE} is unavailable in the configured apt sources." >&2
  echo "Make sure the official ROS 2 apt repository is enabled." >&2
  exit 1
fi

TASK_PACKAGES=("${SDK_PACKAGE}")
if apt-cache show librealsense2-udev-rules >/dev/null 2>&1; then
  TASK_PACKAGES+=(librealsense2-udev-rules)
fi
"${TASK_SUDO[@]}" apt-get install -y "${TASK_PACKAGES[@]}"

if command -v udevadm >/dev/null 2>&1; then
  "${TASK_SUDO[@]}" udevadm control --reload-rules
  "${TASK_SUDO[@]}" udevadm trigger
fi

# shellcheck disable=SC1090
source "${ROS_SETUP}"

SDK_CMAKE_CONFIG="$(find "/opt/ros/${TASK_ROS_DISTRO}" \
  -type f -name realsense2Config.cmake -print -quit)"
SDK_HEADER="/opt/ros/${TASK_ROS_DISTRO}/include/librealsense2/rs.hpp"
SDK_LIBRARY="$(find "/opt/ros/${TASK_ROS_DISTRO}" \
  -type f -name 'librealsense2.so.*' -print -quit)"

if [[ -z "${SDK_CMAKE_CONFIG}" || ! -f "${SDK_HEADER}" || -z "${SDK_LIBRARY}" ]]; then
  echo "librealsense2 installed, but its ROS library, CMake config or headers were not found." >&2
  exit 1
fi

echo "librealsense2 installation verified:"
echo "  header: ${SDK_HEADER}"
echo "  cmake:  ${SDK_CMAKE_CONFIG}"
echo "  library: ${SDK_LIBRARY}"

# Some upstream librealsense 2.58 packages export their private Fast DDS copy.
# That conflicts with ROS 2 Humble's rmw_fastrtps in the same process.  The ROS
# repository build is expected not to expose this symbol; fail early instead of
# leaving a camera node that aborts during ROS node creation.
if command -v nm >/dev/null 2>&1 && \
  nm -D -C "${SDK_LIBRARY}" 2>/dev/null | \
    grep 'DomainParticipantFactory::get_instance' >/dev/null; then
  echo "The installed ROS librealsense library exports Fast DDS symbols." >&2
  echo "Refusing this build because it can conflict with ROS 2 rmw_fastrtps." >&2
  exit 1
fi

if command -v rs-enumerate-devices >/dev/null 2>&1; then
  echo ""
  echo "Connected RealSense devices:"
  rs-enumerate-devices --compact || true
else
  echo "rs-enumerate-devices is not on PATH; source ${ROS_SETUP} before using it."
fi

echo ""
echo "If the D435 was connected during installation, reconnect its USB cable once."
echo "Always source ${ROS_SETUP} before configuring, building or launching this workspace."
