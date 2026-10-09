FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=harmonic

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    wget \
    curl \
    git \
    cmake \
    build-essential \
    clangd \
    clang-format \
    ccache \
    gawk \
    pkg-config \
    gnupg \
    lsb-release \
    ca-certificates \
    python3-pip \
    python3-venv \
    python3-numpy \
    python3-pexpect \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-lxml \
    python3-colcon-common-extensions \
    libxml2-dev \
    libxslt1-dev \
    libtool \
    rapidjson-dev \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    libxext6 \
    libx11-6 \
    mesa-utils \
    x11-xserver-utils \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | \
        gpg --dearmor --yes -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    libgz-sim8-dev \
    libgz-cmake3-dev \
    libgz-plugin2-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-mavros-msgs \
    ros-jazzy-launch-ros \
    ros-jazzy-ros2launch \
    ros-jazzy-rviz2 \
    geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages --no-cache-dir \
    future \
    empy==3.3.4 \
    pexpect \
    MAVProxy \
    pymavlink \
    intelhex

RUN wget -qO /tmp/install_geographiclib_datasets.sh \
        https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && bash /tmp/install_geographiclib_datasets.sh \
    && rm /tmp/install_geographiclib_datasets.sh

ARG ARDUPILOT_GAZEBO_REF=082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git /opt/ardupilot_gazebo \
    && cd /opt/ardupilot_gazebo \
    && git checkout ${ARDUPILOT_GAZEBO_REF} \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j"$(nproc)" \
    && chmod -R a+rX /opt/ardupilot_gazebo

ARG ARDUPILOT_REF=Rover-4.6.2
RUN git clone --depth 1 --branch ${ARDUPILOT_REF} --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
    && cd /opt/ardupilot \
    && ./waf configure --board sitl \
    && ./waf rover \
    && chmod -R a+rX /opt/ardupilot /opt/ardupilot_gazebo

RUN userdel -r ubuntu 2>/dev/null || true

ARG USERNAME=developer
ARG WORKDIR=/home/developer/ardurover_navigation

RUN useradd -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f ${WORKDIR}/install/setup.bash ] && source ${WORKDIR}/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "export ARDUROVER_NAV_ROOT=${WORKDIR}" >> /home/${USERNAME}/.bashrc

ENV ARDUROVER_NAV_ROOT=${WORKDIR}
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

USER ${USERNAME}
WORKDIR ${WORKDIR}
