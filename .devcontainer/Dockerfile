FROM ros:humble-ros-base

# 同步容器内用户到宿主UID/GID，避免文件权限错乱
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# 基础工具
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo git curl vim nano locales tzdata \
    python3-pip python3-colcon-common-extensions \
    ros-dev-tools \
    bash-completion \
    # 可选：头less GUI 支持（xvfb + ros2 turtlesim）
    xvfb x11-apps ros-humble-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# 创建同UID用户并给sudo免密
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# 确保工作目录存在并归属 ubuntu 用户
RUN mkdir -p /home/${USERNAME}/ros2_ws && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/ros2_ws

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ROS 环境 & 开发便捷项
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc \
    && echo 'export ROS_DOMAIN_ID=1' >> ~/.bashrc \
    && echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc \
    && echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >> ~/.bashrc \
    && echo 'export _colcon_cd_root=/opt/ros/humble/' >> ~/.bashrc \
    && echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc \
    && echo "alias ck_dep='cd ~/ros2_ws && rosdep install -i --from-path src --rosdistro humble -y'" >> ~/.bashrc \
    && echo 'col_mk() { cd ~/ros2_ws && ck_dep && colcon build --packages-select "$1" && source ~/ros2_ws/install/setup.bash; }' >> ~/.bashrc

# rosdep 首次初始化（非root用户下）
RUN sudo rosdep init || true && rosdep update

# 进入容器时自动 source 工作区（存在才 source）
RUN echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
