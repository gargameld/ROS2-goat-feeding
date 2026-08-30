# Webtop desktop with ROS 2 and the goat-feeding workspace.
#
# Pinned to the last webtop tag built on Ubuntu 24.04 (noble). Do not bump it
# without also changing ROS_DISTRO: ubuntu-xfce-version-6af183f9 (2026-04-09)
# and every tag after it rebase onto Ubuntu 26.04 "resolute", and
# packages.ros.org publishes only ros-lyrical-* for resolute -- no ros-jazzy-*
# -- so the apt-get install below dies with "Unable to locate package
# ros-jazzy-desktop".
ARG WEBTOP_IMAGE=ubuntu-xfce-version-d5ad760c
ARG ROS_DISTRO=jazzy
ARG DEFAULT_USERNAME=abc
ARG DEFAULT_PASSWORD=abc

FROM linuxserver/webtop:${WEBTOP_IMAGE}

ARG ROS_DISTRO
ARG DEFAULT_USERNAME
ARG DEFAULT_PASSWORD
ARG TARGETARCH

SHELL ["/bin/bash", "-c"]

RUN if [ "$TARGETARCH" = "amd64" ] || [ "$TARGETARCH" = "arm64" ]; then \
        echo "Architecture $TARGETARCH is supported."; \
    else \
        echo "Unsupported architecture: $TARGETARCH"; exit 1; \
    fi

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake curl git locales nano openssh-server \
        python3 python3-pip python3-venv software-properties-common \
        libboost-all-dev libeigen3-dev libopencv-dev libopenni2-dev libpcl-dev \
        mesa-utils && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-moveit-ros-perception && \
    mkdir -p /config/workspace /config/.XDG /config/.ros/log && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    echo "${DEFAULT_USERNAME}:${DEFAULT_PASSWORD}" | chpasswd && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO}

# Build and install GPD, then put its bundled neural-network models where the
# gpd_ros2 configuration expects them: /opt/gpd/models/.
RUN git clone https://github.com/rohitmenon86/gpd.git /opt/tmp/gpd && \
    cmake -S /opt/tmp/gpd -B /opt/tmp/gpd/build && \
    cmake --build /opt/tmp/gpd/build --parallel "$(nproc)" && \
    cmake --install /opt/tmp/gpd/build && \
    mkdir -p /opt/gpd && \
    mv /opt/tmp/gpd/models /opt/gpd/models && \
    ldconfig && \
    rm -rf /opt/tmp/gpd

# MuJoCo and Open3D have no apt packages, so they come from PyPI. Install them
# as abc -- the user the ROS nodes run as -- so they land in that user's
# /config/.local rather than the system tree. Four details here are easy to get
# wrong, and each one fails in its own way:
#   - /config is still root-owned at build time (LinuxServer's init only
#     chowns it at container start), so abc cannot create /config/.local and
#     pip aborts with "[Errno 13] Permission denied: '/config/.local'".
#     install -d fixes the ownership first.
#   - HOME. runuser does not switch HOME, so without setting it here --user
#     would resolve to /root/.local and abc would never see the packages.
#   - /usr/bin/python3 by absolute path. The base image sets VIRTUAL_ENV=/lsiopy
#     and puts /lsiopy/bin first on PATH; this webtop tag has no python3 there,
#     but newer ones do, and a bare `python3` that resolves into that venv is
#     invisible to the ROS nodes, which run under /usr/bin/python3.
#   - --break-system-packages. Ubuntu marks its python3 EXTERNALLY-MANAGED and
#     pip refuses to touch it, user site included, without this flag.
# numpy is deliberately left out: pip sees ROS's system numpy 1.26 through
# dist-packages and leaves it alone, which is what we want -- a numpy 2.x in
# /config/.local would shadow it and break ROS's Python extension modules.
RUN install -d -o abc -g abc /config /config/.local && \
    runuser -u abc -- env HOME=/config \
        /usr/bin/python3 -m pip install --user --break-system-packages --no-cache-dir \
        mujoco open3d

# Ownership of /config is normally fixed at runtime by LinuxServer's init, but we
# seed these dirs so the ROS/XDG paths exist and ROS can create log files. abc is
# the default LSIO user (911).
RUN chown -R abc:abc /config/workspace /config/.XDG /config/.ros

# /config may be backed by an existing volume whose ownership is not changed by
# the image build. Repair ROS and XDG runtime directories during container init,
# before desktop applications or ROS controller spawners run as abc.
RUN mkdir -p /custom-cont-init.d && \
    { \
        echo '#!/usr/bin/with-contenv bash'; \
        echo 'mkdir -p /config/.ros/log /config/.XDG'; \
        echo 'chown -R abc:abc /config/.ros /config/.XDG'; \
        echo 'chmod 700 /config/.XDG'; \
    } > /custom-cont-init.d/10-fix-config-permissions && \
    chmod +x /custom-cont-init.d/10-fix-config-permissions

EXPOSE 3000 3001 22

ENTRYPOINT ["/init"]
