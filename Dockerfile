# ==========================================================
# Stage: clean
# Base image: ros:jazzy-ros-base-noble
# Contains: Ubuntu 24.04 + ROS Jazzy + base libs
# Purpose: to build the base enviroment that prepared for 
#          installing necessary apps
# ==========================================================
FROM ros:jazzy-ros-base-noble AS clean

ENV DEBIAN_FRONTEND=nointeractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    nano \
    git \
    wget \
    curl \
    terminator \
    tmux \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-dev \
    libglew-dev \
    libopencv-dev \
    build-essential \
    cmake \
    pkg-config \
&& rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /robotrio

CMD ["/bin/bash"]


# ==========================================================
# Stage: base
# Base image: clean
# Contains: Python 3.14 + Gazebo Jetty
# Purpose: main simulation
# ==========================================================
FROM clean AS base

RUN apt update && apt install -y --no-install-recommends \ 
    python3 \
    python3-pip \
    python3-venv \
    python3-full \
&& rm -rf /var/lib/apt/lists/*

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y gz-jetty \
&& rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/python-venv

ENV PATH="/opt/python-venv/bin:$PATH"

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /robotrio

CMD ["/bin/bash/"]

# ==========================================================
# Stage: humanoid
# Base image: base
# Contains: humanoid requirements
# Purpose: humanoid simulation
# ==========================================================
FROM base AS humanoid

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-testing-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-testing-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg --remove ros2-apt-source && \
    dpkg -i /tmp/ros2-testing-apt-source.deb

COPY humanoid/apt-packages.txt /tmp/apt-packagers.txt
RUN apt update && \
    xargs -a /tmp/apt-packagers.txt apt install -y \
&& rm -rf /var/lib/apt/lists/*

WORKDIR /robotrio

CMD ["/bin/bash/"]

# ==========================================================
# Stage: drone
# Base image: base
# Contains: drone requirements
# Purpose: drone simulation
# ==========================================================
FROM base AS drone

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-testing-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-testing-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg --remove ros2-apt-source && \
    dpkg -i /tmp/ros2-testing-apt-source.deb

COPY drone/apt-packages.txt /tmp/apt-packagers.txt
RUN apt update && \
    xargs -a /tmp/apt-packagers.txt apt install -y \
&& rm -rf /var/lib/apt/lists/*

RUN chmod drone/run.sh

WORKDIR /tmp

RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    make px4_sitl gz_all -j$(nproc) && \
    mkdir -p /robotrio/drone/bin && \
    cp /tmp/PX4-Autopilot/build/px4_sitl_default/bin/px4 /robotrio/drone/bin/ && \
    chmod +x /robotrio/drone/bin/px4 && \
    mkdir -p /robotrio/drone/lib && \
    cp -r /tmp/PX4-Autopilot/build/px4_sitl_default/lib/libgz* /robotrio/drone/lib/ || true && \
    cp -r /tmp/PX4-Autopilot/build/px4_sitl_default/lib/libFlight* /robotrio/drone/lib/ || true && \
    rm -rf /tmp/PX4-Autopilot

RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    cmake .. -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_P2P_PROFILE=OFF && \
    make && \
    cp MicroXRCEAgent /robotrio/drone/bin/ && \
    chmod +x /robotrio/drone/bin/MicroXRCEAgent && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

WORKDIR /robotrio

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD []
