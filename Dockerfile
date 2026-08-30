# ==========================================================
# Stage: clean
# Base image: ros:rolling-ros-base-noble
# Contains: Ubuntu 26.04 Resolute + ROS Rolling + base libs
# Purpose: to build the base enviroment that prepared for 
#          installing necessary apps
# ==========================================================
FROM ros:rolling-ros-base-resolute AS clean

ENV DEBIAN_FRONTEND=nointeractive

COPY requirements.apt.txt /tmp/requirements.apt.txt
RUN apt-get update && \
    xargs -a /tmp/requirements.apt.txt apt-get install -y --no-install-recommends \
&& rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/rolling/setup.bash" >> /root/.bashrc

WORKDIR /robotrio

CMD ["/bin/bash"]

# ==========================================================
# Stage: base
# Base image: clean
# Contains: Python 3.14 + Gazebo Rotary
# Purpose: main simulation
# ==========================================================
FROM clean AS base

RUN apt-get update && apt-get install -y --no-install-recommends \ 
    python3 \
    python3-pip \
    python3-venv \
    python3-full \
&& rm -rf /var/lib/apt/lists/*

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y \
    ros-rolling-ros-gz \
&& rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/python-venv

ENV PATH="/opt/python-venv/bin:$PATH"

COPY requirements.py.txt /tmp/requirements.py.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.py.txt

ENV GZ_SIM_RESOURCE_PATH="/robotrio/scene:$GZ_SIM_RESOURCE_PATH"

WORKDIR /robotrio

CMD ["/bin/bash/"]

# ==========================================================
# Stage: humanoid
# Base image: base
# Contains: humanoid requirements
# Purpose: humanoid simulation
# ==========================================================
FROM base AS humanoid

# RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
#     curl -L -o /tmp/ros2-testing-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-testing-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
#     dpkg --remove ros2-apt-source && \
#     dpkg -i /tmp/ros2-testing-apt-source.deb

# COPY humanoid/requirements.apt.txt /tmp/requirements.apt.txt
# RUN apt-get update && \
#     xargs -a /tmp/requirements.apt.txt apt-get install -y \
# && rm -rf /var/lib/apt/lists/*

# COPY humanoid/requirements.py.txt /tmp/requirements.py.txt
# RUN pip3 install --no-cache-dir -r /tmp/requirements.py.txt

# WORKDIR /robotrio

CMD ["/bin/bash/"]

# ==========================================================
# Stage: drone
# Base image: base
# Contains: drone requirements
# Purpose: drone simulation
# ==========================================================
FROM base AS drone

COPY drone/requirements.apt.txt /tmp/requirements.apt.txt
RUN apt-get update && \
    xargs -a /tmp/requirements.apt.txt apt-get install -y \
&& rm -rf /var/lib/apt/lists/*

COPY drone/requirements.py.txt /tmp/requirements.py.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.py.txt

WORKDIR /robotrio

COPY drone/install/ /robotrio/drone/install/
RUN cd drone/install && \
    chmod +x *.sh && \
    ./install.sh 

ENV PATH="/MicroXRCEAgent/bin:/PX4-Autopilot/bin:$PATH"
ENV LD_LIBRARY_PATH="/MicroXRCEAgent/lib:$LD_LIBRARY_PATH"
ENV GZ_SIM_RESOURCE_PATH="/robotrio:/robotrio/drone/models:/robotrio/scene:$GZ_SIM_RESOURCE_PATH"
ENV GZ_SIM_SERVER_CONFIG_PATH="/robotrio/scene/server.config"

CMD ["/bin/bash"]
