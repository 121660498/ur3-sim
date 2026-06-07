FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

RUN arch="$(dpkg --print-architecture)" \
    && if [ "$arch" = "arm64" ] || [ "$arch" = "armhf" ] || [ "$arch" = "ppc64el" ] || [ "$arch" = "s390x" ] || [ "$arch" = "riscv64" ]; then \
        ubuntu_mirror_path="ubuntu-ports"; \
    else \
        ubuntu_mirror_path="ubuntu"; \
    fi \
    && printf '%s\n' \
        "deb http://mirrors.tuna.tsinghua.edu.cn/${ubuntu_mirror_path}/ focal main restricted universe multiverse" \
        "deb http://mirrors.tuna.tsinghua.edu.cn/${ubuntu_mirror_path}/ focal-updates main restricted universe multiverse" \
        "deb http://mirrors.tuna.tsinghua.edu.cn/${ubuntu_mirror_path}/ focal-backports main restricted universe multiverse" \
        "deb http://mirrors.tuna.tsinghua.edu.cn/${ubuntu_mirror_path}/ focal-security main restricted universe multiverse" \
        > /etc/apt/sources.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    locales \
    tzdata \
    ca-certificates \
    curl \
    git \
    gnupg2 \
    lsb-release \
    iproute2 \
    net-tools \
    iputils-ping \
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-venv \
    build-essential \
    cmake \
    libssl-dev \
    && locale-gen en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros1.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-teb-local-planner \
    ros-noetic-global-planner \
    ros-noetic-costmap-2d \
    ros-noetic-map-server \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    # 配置 pip 使用清华源
    && python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel \
    && python3 -m pip install --no-cache-dir numpy opencv-python typing_extensions \
    && (rosdep init || true) \
    && printf "\nsource /opt/ros/noetic/setup.bash\n" >> /root/.bashrc \
    && printf "if [ -f /workspaces/NAV/HongTu/G1Nav2D/devel/setup.bash ]; then source /workspaces/NAV/HongTu/G1Nav2D/devel/setup.bash; fi\n" >> /root/.bashrc \
    && printf "export PYTHONPATH=/workspaces/NAV/HongTu/unitree_sdk2_python:\$PYTHONPATH\n" >> /root/.bashrc \
    && rm -rf /var/lib/apt/lists/*
