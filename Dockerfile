FROM osrf/ros:jazzy-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo \
    locales \
    bash-completion \
    less \
    vim \
    git \
    openssh-client \
    ca-certificates \
    curl \
    wget \
    iputils-ping \
    net-tools \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    ros-dev-tools \
    udev \
    usbutils \
    libglu1-mesa \
    mesa-utils && \
    rm -rf /var/lib/apt/lists/*

# velodyneドライバ + RMDモータ用CANツール
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-jazzy-velodyne \
    ros-jazzy-velodyne-description \
    can-utils \
    iproute2 && \
    rm -rf /var/lib/apt/lists/*

# Livox SDK2 (Mid-360はSDK2系。SDK1やlivox_ros_driverでは動かない)
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/Livox-SDK2/build -j"$(nproc)" && \
    cmake --install /tmp/Livox-SDK2/build && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2

# livox_ros_driver2 (Mid-360用ROS 2ドライバ)。build.shを使わず直接colcon:
# package_ROS2.xml→package.xmlのコピーがbuild.shの実質的な仕事。
RUN mkdir -p /opt/livox_ws/src && \
    git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git \
        /opt/livox_ws/src/livox_ros_driver2 && \
    cp /opt/livox_ws/src/livox_ros_driver2/package_ROS2.xml \
       /opt/livox_ws/src/livox_ros_driver2/package.xml && \
    bash -c "source /opt/ros/jazzy/setup.bash && \
        cd /opt/livox_ws && \
        colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy \
            -DCMAKE_BUILD_TYPE=Release"

RUN locale-gen en_US.UTF-8 ja_JP.UTF-8 && \
    update-locale LANG=ja_JP.UTF-8

ENV LANG=ja_JP.UTF-8 \
    LANGUAGE=ja_JP:ja \
    LC_ALL=ja_JP.UTF-8

# bag再生→rviz2→動画化 (scripts/bag2video.sh) 用: 仮想Xサーバと画面キャプチャ
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ffmpeg \
    xvfb \
    x11-utils && \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    existing_gid_group="$(getent group "${USER_GID}" | cut -d: -f1 || true)"; \
    if [ -n "${existing_gid_group}" ] && [ "${existing_gid_group}" != "${USERNAME}" ]; then \
        groupmod -n "${USERNAME}" "${existing_gid_group}"; \
    fi; \
    if ! getent group "${USERNAME}" >/dev/null 2>&1; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    else \
        groupmod -g "${USER_GID}" "${USERNAME}"; \
    fi; \
    existing_uid_user="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)"; \
    if [ -n "${existing_uid_user}" ] && [ "${existing_uid_user}" != "${USERNAME}" ]; then \
        usermod --login "${USERNAME}" --home "/home/${USERNAME}" --move-home "${existing_uid_user}"; \
    fi; \
    if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
        useradd --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}"; \
    else \
        usermod --uid "${USER_UID}" --gid "${USER_GID}" "${USERNAME}"; \
    fi; \
    usermod -aG sudo "${USERNAME}" && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/90-${USERNAME}" && \
    chmod 0440 "/etc/sudoers.d/90-${USERNAME}"

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/skel/.bashrc && \
    echo "source /opt/livox_ws/install/setup.bash" >> /etc/skel/.bashrc && \
    echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /opt/livox_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "alias ll='ls -alF'" >> /home/${USERNAME}/.bashrc && \
    mkdir -p /workspaces && \
    chown -R ${USER_UID}:${USER_GID} /home/${USERNAME} /workspaces

WORKDIR /workspaces

USER ${USERNAME}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
