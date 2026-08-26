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
    python3 -m pip install open3d --break-system-packages && \
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

# Select the OpenGL driver at runtime instead of baking one host's choice into
# the image. MuJoCo's viewer and its offscreen camera rendering go through
# OpenGL, and the driver that reaches hardware differs per host: D3D12 on WSL,
# libGLX_nvidia with the nvidia container runtime, plain Mesa on a native
# /dev/dri. gl-autodetect.sh probes glxinfo (hence mesa-utils above) and exports
# whatever it takes to avoid the llvmpipe software fallback.
COPY docker/gl-autodetect.sh /usr/local/lib/gl-autodetect.sh
RUN chmod 0644 /usr/local/lib/gl-autodetect.sh && \
    echo 'source /usr/local/lib/gl-autodetect.sh' >> /etc/bash.bashrc

# Run sshd under s6 instead of starting it during build. LinuxServer's baseimage
# picks up any executable in /custom-services.d and supervises it, so /init stays
# PID 1 (required by s6-overlay) while sshd runs as a managed service.
RUN mkdir -p /custom-services.d && \
    { \
        echo '#!/usr/bin/with-contenv bash'; \
        echo 'mkdir -p /run/sshd'; \
        echo '[ -f /etc/ssh/ssh_host_ed25519_key ] || ssh-keygen -A'; \
        echo 'exec /usr/sbin/sshd -D -e'; \
    } > /custom-services.d/sshd && \
    chmod +x /custom-services.d/sshd

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
