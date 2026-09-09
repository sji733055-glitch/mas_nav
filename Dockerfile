# 环境镜像：只装系统依赖。compose 把仓库挂到 /home/ros2_ws/src。
# 改 C++ 不要 --build，在容器里 colcon。只有改了本文件才 docker compose build。
#
# 本地默认走中科大镜像。不要在文件头加 # syntax=docker/dockerfile:1，Hub 会超时；
# 引擎自带的 BuildKit 仍然认 --mount=type=cache。
ARG BASE_IMAGE=5d63i5idjq9rby.xuanyuan.run/osrf/ros:humble-desktop-full
FROM ${BASE_IMAGE} AS dev

ENV DEBIAN_FRONTEND=noninteractive

RUN cp /etc/skel/.bashrc ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list

RUN curl -sSL https://mirrors.ustc.edu.cn/rosdistro/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.ustc.edu.cn/ros2/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    rm -rf /etc/apt/sources.list.d/ros2.sources || true

# apt 缓存挂载要留下 .deb；Ubuntu 镜像默认的 docker-clean 会删掉它们。
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# 只装 desktop-full 没有、编译/运行又用到的。
# libdw-dev：minco 自带 backward.hpp 默认 BACKWARD_HAS_DW=1，没有 dwarf.h 编不过。
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-navigation2 \
        ros-humble-rosbag2-storage-mcap \
        libomp-dev \
        libasio-dev \
        libdw-dev \
        python3-pip

ENV ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    printf '%s\n' \
      'yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/osx-homebrew.yaml osx' \
      'yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/base.yaml' \
      'yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/python.yaml' \
      'yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/ruby.yaml' \
      > /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update --rosdistro="$ROS_DISTRO"

RUN mkdir -p /home/ros2_ws/src /home/ros2_ws/build /home/ros2_ws/install /home/ros2_ws/log

RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install \
      -i https://mirrors.ustc.edu.cn/pypi/simple \
      "open3d>=0.15.0" \
      "numpy<=2"

WORKDIR /home/ros2_ws

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
