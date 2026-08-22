# Webtop desktop with ROS 2 and the goat-feeding workspace.
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
        libboost-all-dev libeigen3-dev libopencv-dev libopenni2-dev libpcl-dev && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup && \
    python3 -m pip install open3d --break-system-packages && \
    mkdir -p /var/run/sshd /config/workspace /config/.XDG && \
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

# The build context must contain the workspace directory beside this Dockerfile.
# COPY --chown=${DEFAULT_USERNAME}:${DEFAULT_USERNAME} workspace/ /config/workspace/
RUN mkdir -p /config/.ros && \
    sudo chown -R abc:abc /config/.ros && \
    rosdep init && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths /config/workspace/src --ignore-src \
        --rosdistro ${ROS_DISTRO} -r -y \
        --skip-keys="pytest warehouse_ros_mongo" && \
    rm -rf /var/lib/apt/lists/* && \
    chmod -R a+rwX /config/workspace /opt/ros/${ROS_DISTRO}

EXPOSE 3000 3001 22

ENTRYPOINT ["/bin/bash", "-c", "mkdir -p /config/.XDG /config/.ros; chown -R abc:abc /config/.ros; chown -R ${PUID:-911}:${PGID:-911} /config/.XDG /config/workspace; /usr/sbin/sshd; exec /init"]
