# 使用指定的镜像作为基础
FROM 5d63i5idjq9rby.xuanyuan.run/osrf/ros:humble-desktop-full

# 避免交互式提示
ENV DEBIAN_FRONTEND=noninteractive

# 1. 配置 bashrc
RUN cp /etc/skel/.bashrc ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 2. 配置 Ubuntu 中科大镜像源
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list

# 3. 配置 ROS2 中科大镜像源
RUN curl -sSL https://mirrors.ustc.edu.cn/rosdistro/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.ustc.edu.cn/ros2/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    rm -rf /etc/apt/sources.list.d/ros2.sources || true

# 4. 更新软件源并安装依赖
RUN apt update && apt install -y \
    cmake \
    clang \
    clangd \
    clang-format \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-diagnostic-updater \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    x11-xserver-utils \
    unzip \
    curl \
    iputils-ping \
    ros-humble-octomap* \
    libomp-dev \
    libdw-dev \
    libasio-dev\
    ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-visualization-msgs ros-humble-rosbag2-storage-mcap ros-humble-rqt-tf-tree libceres-dev \
    clang-tidy \
    python3-pip \
    python3-dev \
    python3-matplotlib \
    python3-tk

# 5. 初始化 rosdep
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    echo "yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/osx-homebrew.yaml osx" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    echo "yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/base.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list && \
    echo "yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/python.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list && \
    echo "yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/ruby.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list && \
    # 修改 rosdep Python 源码中的地址
    find /usr -type f -name "*.py" | xargs sed -i 's|raw.githubusercontent.com/ros/rosdistro/master|mirrors.ustc.edu.cn/rosdistro|g' && \
    rosdep update

# 6. 创建工作空间目录
RUN mkdir -p /home/ros2_ws/src /home/ros2_ws/build /home/ros2_ws/install /home/ros2_ws/log

# 7. 离线点云工具（pcd_trans / 可视化）。Humble 是 Ubuntu 22.04，不需要 LD_PRELOAD。
RUN python3 -m pip install --no-cache-dir \
      -i https://mirrors.ustc.edu.cn/pypi/simple \
      "open3d>=0.15.0" \
      "numpy<=2"

# 设置工作目录
WORKDIR /home/ros2_ws

# 设置环境变量
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
