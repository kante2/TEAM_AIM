# syntax=docker/dockerfile:1
# ===============================
# ROS2 Foxy Dev Image (Ubuntu 20.04)
# - C++17(g++9), CMake>=3.16
# - ROS2 dev tools (colcon/rosdep/vcstool/flake8/pytest...)
# - extra libs (bullet/asio/tinyxml2/cunit)
# - simulator build deps (eigen/glm/yaml-cpp + X11/GL dev headers 등)
# - bashrc "Run commands" 세팅(오타/공백/따옴표 문제 없이)
# ===============================
FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm-256color \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=Asia/Seoul

# X11 + NVIDIA(OpenGL) friendly defaults
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_INDIRECT=0 \
    ROS_LOCALHOST_ONLY=0

# Workspace 경로 (네 compose에 맞춰서 /root/kaist_contest_ws)
ENV WS=/root/kaist_contest_ws

# -------------------------------
# Ubuntu packages (요구사항 묶음)
# -------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    g++ \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    nano \
    vim \
    tmux \
    sudo \
    locales \
    tzdata \
    ca-certificates \
    gnupg2 \
    lsb-release \
    iproute2 \
    net-tools \
    iputils-ping \
    netcat \
    less \
    htop \
    udev \
    usbutils \
    openssh-client \
    openssh-server \
    dos2unix \
    mesa-utils \
    libgl1-mesa-glx \
    libglu1-mesa \
    \
    # ---- Simulator / GUI build deps (2nd Dockerfile에서 추가된 것들) ----
    libeigen3-dev \
    libglm-dev \
    nlohmann-json3-dev \
    libyaml-cpp-dev \
    libx11-dev \
    libxext-dev \
    libxrandr-dev \
    libxcursor-dev \
    libxfixes-dev \
    libxi-dev \
    libudev-dev \
    libgl-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    \
    # ROS2 dev tools (문서 2.4)
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-argcomplete \
    python3-colcon-cd \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-flake8 \
    python3-pytest-cov \
    python3-aiohttp \
    \
    # 추가 의존성 (문서 2.4 마지막)
    libbullet-dev \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
 && rm -rf /var/lib/apt/lists/*

# test_msgs 패키지를 설치하여 테스트용 메시지 타입 사용 가능하게 함
RUN apt-get update && apt-get install -y ros-foxy-test-msgs

# -------------------------------
# pip packages (문서 2.4 pip install -U ...)
# -------------------------------
RUN python3 -m pip install --no-cache-dir -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest

# rosdep init/update (이미 되어있으면 무시)
RUN rosdep init 2>/dev/null || true && rosdep update

# -------------------------------
# (선택) SDL3 필요하면 여기 주석 해제
# - Ubuntu 20.04는 공식 패키지로 SDL3가 없는 경우가 많아 소스 빌드
# -------------------------------
RUN git clone --branch release-3.2.2 --depth 1 https://github.com/libsdl-org/SDL.git /tmp/SDL \
  && cmake -S /tmp/SDL -B /tmp/SDL/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DSDL_STATIC=OFF \
      -DSDL_SHARED=ON \
      -DSDL_TESTS=OFF \
  && cmake --build /tmp/SDL/build -j"$(nproc)" \
  && cmake --install /tmp/SDL/build \
  && ldconfig \
  && rm -rf /tmp/SDL

# -------------------------------
# Workspace 디렉토리 생성
# (호스트 마운트하면 이건 덮어써짐)
# -------------------------------
RUN mkdir -p ${WS}/src
WORKDIR ${WS}

# -------------------------------
# .bashrc "Run commands" 세팅
# - 공백 없는 export 문법
# - fancy quote(’ ‘) 금지, ASCII quote 사용
# - install/local_setup.bash는 존재할 때만 source
# -------------------------------
RUN cat << 'EOF' >> /root/.bashrc

# ===== ROS2 Foxy =====
if [ -f /opt/ros/foxy/setup.bash ]; then
  source /opt/ros/foxy/setup.bash
fi

# ===== Workspace local_setup (build 후 생김) =====
if [ -f /root/TEAM_AIM/install/local_setup.bash ]; then
  source /root/TEAM_AIM/install/local_setup.bash
fi

# ===== colcon/vcs completion =====
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi
if [ -f /usr/share/vcstool-completion/vcs.bash ]; then
  source /usr/share/vcstool-completion/vcs.bash
fi
if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
  source /usr/share/colcon_cd/function/colcon_cd.sh
fi

# ===== Simulator/WS env =====
export _colcon_cd_root=/root/TEAM_AIM
export ROS_DOMAIN_ID=100
export ROS_NAMESPACE=central
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 로그 포맷(문서 예시 중 에러나던 줄은 제거하고 안전한 형태로)
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# ===== aliases =====
alias cw='cd /root/TEAM_AIM'
alias cs='cd /root/TEAM_AIM/src'
alias ccd='colcon_cd'
alias cb='cd /root/TEAM_AIM && colcon build --symlink-install'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias cbu='colcon build --symlink-install --packages-up-to'
alias ct='colcon test'
alias ctp='colcon test --packages-select'
alias ctr='colcon test-result'

alias rt='ros2 topic list'
alias re='ros2 topic echo'
alias rn='ros2 node list'

alias killgazebo='killall -9 gazebo gzserver gzclient 2>/dev/null || true'
EOF

CMD ["bash"]