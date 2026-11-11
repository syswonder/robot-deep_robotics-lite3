ARG ROS_BASE_IMAGE=robonix-ros:humble-ros-base
FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple

RUN set -eux; \
    find /etc/apt/sources.list.d/ -maxdepth 1 \( -name '*.list' -o -name '*.sources' \) -print0 \
      | xargs -0 -r sed -i --follow-symlinks \
          -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
          -e 's|^Types: deb deb-src$|Types: deb|'; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      python3-pip python3-numpy python3-scipy python3-yaml python3-pil \
      python3-cv-bridge ros-humble-tf2-ros ros-humble-tf-transformations \
      ros-humble-rmw-fastrtps-cpp; \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
      'grpcio==1.80.0' 'grpcio-tools==1.76.0' 'protobuf==6.33.6' \
      'mcp>=1.27,<2' 'fastmcp>=3,<5' uvicorn httpx Pillow PyYAML

COPY entrypoint.sh /scene-lite-entrypoint.sh
RUN chmod +x /scene-lite-entrypoint.sh
ENTRYPOINT ["/scene-lite-entrypoint.sh"]
