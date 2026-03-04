ARG ROS_DISTRO=humble

# Optional proxy settings (pass via: --build-arg http_proxy=... --build-arg https_proxy=...)
# NOTE: ARG declared before FROM is global, but must be re-declared in each stage to be used.
ARG http_proxy=
ARG https_proxy=
ARG no_proxy=
ARG HTTP_PROXY=
ARG HTTPS_PROXY=
ARG NO_PROXY=

# -------------------------
# Builder
# -------------------------
FROM ros:${ROS_DISTRO}-ros-base AS builder

ARG ROS_DISTRO=humble

ARG http_proxy=
ARG https_proxy=
ARG no_proxy=
ARG HTTP_PROXY=
ARG HTTPS_PROXY=
ARG NO_PROXY=

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ENV http_proxy=${http_proxy} \
  https_proxy=${https_proxy} \
  no_proxy=${no_proxy} \
  HTTP_PROXY=${HTTP_PROXY} \
  HTTPS_PROXY=${HTTPS_PROXY} \
  NO_PROXY=${NO_PROXY} \
  MAKEFLAGS="-j1" \
  CMAKE_BUILD_PARALLEL_LEVEL=1

WORKDIR /ws

# Optional: switch Ubuntu apt mirror (useful in CN). Example:
#   --build-arg APT_MIRROR=mirrors.tuna.tsinghua.edu.cn
ARG APT_MIRROR=
RUN if [[ -n "${APT_MIRROR}" ]]; then \
    sed -i \
      -e "s|http://archive.ubuntu.com/ubuntu|http://${APT_MIRROR}/ubuntu|g" \
      -e "s|http://security.ubuntu.com/ubuntu|http://${APT_MIRROR}/ubuntu|g" \
      /etc/apt/sources.list; \
  fi

# Build tools + rosdep + python venv toolchain
RUN if [[ -n "${APT_MIRROR}" ]]; then \
    if [[ -n "${no_proxy}" ]]; then export no_proxy="${no_proxy},${APT_MIRROR}"; else export no_proxy="${APT_MIRROR}"; fi; \
    if [[ -n "${NO_PROXY}" ]]; then export NO_PROXY="${NO_PROXY},${APT_MIRROR}"; else export NO_PROXY="${APT_MIRROR}"; fi; \
  fi \
  && apt-get update && apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  curl \
  wget \
  python3-pip \
  python3-venv \
  python3-dev \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-${ROS_DISTRO}-example-interfaces \
  libpcl-dev \
  libeigen3-dev \
  libomp-dev \
  python3-tk \
  libusb-1.0-0-dev \
  usbutils \
  vim \
  nano \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && rosdep update

# Workspace dependency snapshot (package.xml only)
# 说明：docker/rosdep_src 由仓库内生成（只含 package.xml），用于让 rosdep 这一步长期命中缓存。
# 注意：不要把快照放进 /ws/src，否则 colcon 会看到同名包两份（docker/rosdep_src 与 src）。
COPY docker/rosdep_src/ /ws/rosdep_src/

# Install ROS package deps
# auto_aim_interfaces：你明确说“自瞄接口不要了”，因此这里跳过该 rosdep key。
RUN apt-get update \
  && if [[ -n "${APT_MIRROR}" ]]; then \
      if [[ -n "${no_proxy}" ]]; then export no_proxy="${no_proxy},${APT_MIRROR}"; else export no_proxy="${APT_MIRROR}"; fi; \
      if [[ -n "${NO_PROXY}" ]]; then export NO_PROXY="${NO_PROXY},${APT_MIRROR}"; else export NO_PROXY="${APT_MIRROR}"; fi; \
    fi \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && rosdep install -r --from-paths rosdep_src --ignore-src --rosdistro ${ROS_DISTRO} -y \
  --skip-keys auto_aim_interfaces \
  && rm -rf /var/lib/apt/lists/*

# Workspace full sources (this invalidates cache when code changes)
COPY . /ws

# Safety net: ensure deps match current sources (even if docker/rosdep_src snapshot is stale)
RUN apt-get update \
  && if [[ -n "${APT_MIRROR}" ]]; then \
      if [[ -n "${no_proxy}" ]]; then export no_proxy="${no_proxy},${APT_MIRROR}"; else export no_proxy="${APT_MIRROR}"; fi; \
      if [[ -n "${NO_PROXY}" ]]; then export NO_PROXY="${NO_PROXY},${APT_MIRROR}"; else export NO_PROXY="${APT_MIRROR}"; fi; \
    fi \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
  --skip-keys auto_aim_interfaces \
  && rm -rf /var/lib/apt/lists/*

# Pre-install small_gicp (required by small_gicp_relocalization headers/libs)
RUN cmake -S /ws/src/small_gicp -B /tmp/small_gicp_build \
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE:-Release} \
    -DBUILD_HELPER=ON \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARKS=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF \
  && cmake --build /tmp/small_gicp_build \
  && cmake --install /tmp/small_gicp_build

# NeuPAN python deps must be installed in a venv
RUN ./scripts/setup_neupan_env.sh

ARG CMAKE_BUILD_TYPE=Release
# Optional extra args (default: single-thread to reduce peak RAM)
ARG COLCON_EXTRA_ARGS="--executor sequential --parallel-workers 1"
# 你要求“先不要自瞄相关”，默认跳过 rm_behavior_tree/rm_serial_driver（它们依赖 auto_aim_interfaces）。
ARG COLCON_SKIP_PACKAGES="rm_behavior_tree rm_serial_driver small_gicp behaviortree_ros2 btcpp_ros2_samples"

# 1) Build everything except neupan_nav2_controller (and skip auto-aim related pkgs)
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && if [[ -n "${COLCON_SKIP_PACKAGES}" ]]; then \
  colcon build \
  --base-paths src \
  --packages-skip neupan_nav2_controller ${COLCON_SKIP_PACKAGES} \
  --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_BUILD_PARALLEL_LEVEL=1 \
  ${COLCON_EXTRA_ARGS}; \
  else \
  colcon build \
  --base-paths src \
  --packages-skip neupan_nav2_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_BUILD_PARALLEL_LEVEL=1 \
  ${COLCON_EXTRA_ARGS}; \
  fi

# 2) Build neupan_nav2_controller inside the venv (PYTHONPATH points to venv site-packages)
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && source /ws/install/setup.bash \
  && source /ws/neupan_env/bin/activate \
  && export PYTHONPATH="${PYTHONPATH}:/ws/neupan_env/lib/python3.10/site-packages" \
  && colcon build \
  --base-paths src \
  --packages-select neupan_nav2_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_BUILD_PARALLEL_LEVEL=1 \
  ${COLCON_EXTRA_ARGS} \
  && deactivate 2>/dev/null || true

# -------------------------
# Runtime (experimental): reuse builder image
# -------------------------
# 说明：实验阶段先“全部打包”，最终镜像直接复用 builder，避免重复跑 rosdep/apt。
FROM builder AS runtime

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
