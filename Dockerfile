# ==========================================================
# Stage: clean
# Base image: ros:jazzy-ros-base-noble
# Contains: Ubuntu 24.04 + ROS Jazzy + base libs
# Purpose: to build the base enviroment that prepared for 
#          installing necessary apps
# ==========================================================
FROM ros:jazzy-ros-base-noble
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
CMD ["/bin/bash"]
