# T3 Foundation Gemstone Project [t3gemstone.org]
# SPDX-License-Identifier: Apache-2.0
#
# Ubuntu 24.04 + ROS 2 Jazzy (arm64 T3 Gemstone O1 and amd64 PC). Three stages:
#   base   ROS 2 Jazzy and the dependencies of the UGV packages
#   box    development environment: 'task box' opens it with Distrobox, CI runs its jobs here
#   robot  built workspace: 'docker compose build' builds this stage (default)

FROM ros:jazzy-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
WORKDIR /ugv_ws

# Extra options for rosdep. CI installs only the build and test dependencies:
#   --build-arg ROSDEP_ARGS="-t build -t buildtool -t build_export -t buildtool_export -t test"
ARG ROSDEP_ARGS=""

# Dependency manifests only, first: this layer is not rebuilt when the source changes.
COPY repos.yml .
RUN mkdir -p src && vcs import src < repos.yml
COPY src/ugv_interfaces/package.xml src/ugv/ugv_interfaces/
COPY src/ugv_description/package.xml src/ugv/ugv_description/
COPY src/ugv_hal/package.xml src/ugv/ugv_hal/
COPY src/ugv_base/package.xml src/ugv/ugv_base/
COPY src/ugv_imu/package.xml src/ugv/ugv_imu/
COPY src/ugv_line/package.xml src/ugv/ugv_line/
COPY src/ugv_control/package.xml src/ugv/ugv_control/
COPY src/ugv_navigation/package.xml src/ugv/ugv_navigation/
COPY src/ugv_gcs/package.xml src/ugv/ugv_gcs/
COPY src/ugv_bringup/package.xml src/ugv/ugv_bringup/
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y ${ROSDEP_ARGS} \
    && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/*

# libgpiod v2 Python bindings (Ubuntu 24.04 apt only has v1.6).
# v2: a 1024-event kernel buffer and a single ordered queue for encoder events;
# v1, with 16 events per line, misses counts at high RPM. Takes precedence over v1 from apt.
RUN pip install --no-cache-dir --break-system-packages "gpiod>=2.2,<3"


FROM base AS box

# Tools for linting (flake8, shellcheck), the web UI tests (nodejs) and
# Distrobox's interactive shell
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        bash-completion \
        ca-certificates \
        curl \
        file \
        git \
        locales \
        nano \
        nodejs \
        openssh-client \
        python3-flake8 \
        shellcheck \
        sudo \
        vim \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Distrobox copies /etc/os-release from the image; a symlink is lost in the copy
RUN rm -f /etc/os-release && cp /usr/lib/os-release /etc/os-release

# The 'ubuntu' user (UID 1000) in the Ubuntu 24.04 image clashes with the
# account Distrobox creates for the host user
RUN userdel --remove ubuntu 2>/dev/null || true

# Allow minimum password length of image in Distrobox to be 1 character
RUN sed -i 's/pam_unix\.so obscure/pam_unix.so minlen=1 obscure/' /etc/pam.d/common-password
RUN echo gemstone > /etc/hostname

# Taskfile Installation
ARG TASK_VERSION=3.50.0
RUN arch="$(dpkg --print-architecture)" \
    && curl -fsSL "https://github.com/go-task/task/releases/download/v${TASK_VERSION}/task_${TASK_VERSION}_linux_${arch}.deb" \
        --output /tmp/task.deb \
    && apt-get install -y /tmp/task.deb \
    && rm /tmp/task.deb \
    && task --completion bash > /etc/bash_completion.d/task

CMD ["bash"]


FROM base AS robot

COPY src src/ugv
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --event-handlers console_cohesion-

COPY scripts/docker-entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ugv_bringup", "robot.launch.py"]
