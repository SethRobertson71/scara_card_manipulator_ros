# =============================================================
# ROS2 Jazzy — Dobot M1 Pro Vision & Control Container
# Base: Ubuntu 24.04 (Noble) — no CUDA, no BuildKit dependency
# Inference: OpenVINO (Intel iGPU / CPU auto-select at runtime)
#
# !! IMPORTANT — ALWAYS BUILD WITH BUILDKIT DISABLED !!
# BuildKit's network isolation breaks apt access to
# packages.ros.org inside WSL2, causing silent empty installs.
#
#   export DOCKER_BUILDKIT=0
#   docker compose -f docker/docker-compose.yml build --no-cache
#
# =============================================================

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV ROS_VERSION=2
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# AUTO = OpenVINO picks best device at runtime (Intel iGPU if
# present, CPU otherwise). Override at launch:
#   docker compose up -e OPENVINO_DEVICE=CPU
ENV OPENVINO_DEVICE=AUTO

# ── System deps ──────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    wget \
    git \
    nano \
    vim \
    gnupg2 \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    pkg-config \
    libopencv-dev \
    python3-opencv \
    v4l-utils \
    libv4l-dev \
    net-tools \
    iputils-ping \
    usbutils \
    libgl1 \
    libglib2.0-0 \
    libxext6 \
    libxrender1 \
    ocl-icd-libopencl1 \
    clinfo \
    && rm -rf /var/lib/apt/lists/*

# ── ROS2 Jazzy apt source ────────────────────────────────────
# GPG key in its own layer — failure is explicit, not silent
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Hardcode 'noble' — never use shell substitution here,
# it evaluates unreliably inside Docker build contexts
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list

# ── ROS2 Jazzy base install ──────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    && rm -rf /var/lib/apt/lists/*

# Hard stop — build fails here with a clear message if
# ros-base didn't install correctly
RUN test -f /opt/ros/jazzy/setup.bash || \
    (echo "FATAL: /opt/ros/jazzy/setup.bash missing — ROS did not install" && exit 1)

# ── ROS2 project-specific packages ──────────────────────────
# Separate layer so a rebuild after adding packages here
# does not re-download ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-pipeline \
    ros-jazzy-usb-cam \
    ros-jazzy-vision-msgs \
    ros-jazzy-moveit \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ── Python ML deps ───────────────────────────────────────────
# numpy is installed by apt as a dependency (no RECORD file),
# so pip cannot uninstall it the normal way.
# We forcibly remove the dist-info directory so pip can take
# full ownership, then install our own version cleanly.
RUN find /usr/lib/python3 -maxdepth 4 \
        -name "numpy*" -o -name "scipy*" | \
    grep -E "(dist-info|egg-info)" | \
    xargs rm -rf && \
    apt-get remove -y python3-numpy python3-scipy 2>/dev/null || true && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*

# OpenVINO — hardware-agnostic inference (Intel iGPU / CPU)
RUN pip3 install --no-cache-dir --break-system-packages \
    openvino

# Install numpy first standalone so every subsequent package
# sees it already present and does not pull a conflicting version
RUN pip3 install --no-cache-dir --break-system-packages \
    "numpy>=1.26,<2.0"

# scipy and OpenCV after numpy is settled
RUN pip3 install --no-cache-dir --break-system-packages \
    scipy \
    "opencv-python==4.10.0.84" \
    "opencv-python-headless==4.10.0.84"

# YOLO and robot utility packages
RUN pip3 install --no-cache-dir --break-system-packages \
    "ultralytics==8.3.145" \
    transforms3d \
    pyserial

# Re-assert NumPy/OpenCV compatibility in case a dependency
# resolver attempts to upgrade to NumPy 2.x-compatible wheels.
RUN pip3 install --no-cache-dir --break-system-packages \
    "numpy>=1.26,<2.0" \
    "opencv-python==4.10.0.84" \
    "opencv-python-headless==4.10.0.84"

# ── ROS2 workspace ───────────────────────────────────────────
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# ── rosdep init ──────────────────────────────────────────────
RUN rosdep init && rosdep update

# ── Shell environment ────────────────────────────────────────
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc

# ── Entrypoint ───────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
