ARG ROS_IMAGE=docker.1ms.run/library/ros:humble-ros-base-jammy
FROM ${ROS_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_DISABLE_PIP_VERSION_CHECK=1
ENV PYTHONDONTWRITEBYTECODE=1

RUN sed -i \
    -e 's|http://archive.ubuntu.com/ubuntu|http://mirrors.pku.edu.cn/ubuntu|g' \
    -e 's|http://security.ubuntu.com/ubuntu|http://mirrors.pku.edu.cn/ubuntu|g' \
    /etc/apt/sources.list \
    && rm -f /etc/apt/sources.list.d/ros2.sources \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        libbackward-cpp-dev \
        libdw-dev \
        libegl1 \
        libegl1-mesa \
        libgl1-mesa-dri \
        libglfw3 \
        libglfw3-dev \
        libgles2 \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/lite3-simulation-requirements.txt
RUN python3 -m pip install \
    --no-cache-dir \
    --index-url https://mirrors.pku.edu.cn/pypi/web/simple \
    -r /tmp/lite3-simulation-requirements.txt

COPY fastrtps_no_shm.xml /etc/fastrtps_no_shm.xml
WORKDIR /workspace/Lite3_rl_deploy
