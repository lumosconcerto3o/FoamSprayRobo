# 使用官方的 ROS 2 Humble 镜像
# Use official ROS 2 Humble image
FROM osrf/ros:humble-desktop-full

# 设置非 root 用户
# Set up non-root user
ARG USERNAME=pp  # 将用户名改为 pp
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 创建用户 pp
# Create the user pp
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 添加 Gazebo 仓库
# Add Gazebo repository
RUN apt-get update && apt-get install -y wget \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# 安装额外的依赖
# Install additional dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    gazebo \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# 设置工作区
# Set up workspace
RUN mkdir -p /home/lumos/pp_ws/src  # 将工作目录改为 /home/lumos/pp_ws
WORKDIR /home/lumos/pp_ws

# 将工作目录的所有权转移给 pp 用户
RUN chown -R $USERNAME:$USERNAME /home/lumos/pp_ws

# 设置环境
# Set Env
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "if [ -f /home/lumos/pp_ws/install/setup.bash ]; then source /home/lumos/pp_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# 切换到非 root 用户
# Switch to non-root user
USER $USERNAME

# 初始化 rosdep（如果已经初始化则跳过）
# Initialize rosdep (skip if already initialized)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi \
    && rosdep update

# 设置 ROS domain ID
# Set ROS domain ID
ENV ROS_DOMAIN_ID=10

CMD ["bash"]
