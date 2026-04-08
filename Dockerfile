FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    libboost-system-dev \
    build-essential \
    libudev-dev \
    udev \
    git \
    nano \
    vim \
    ros-jazzy-ros2-control \
    # --- CANopen ---
    can-utils \
    ros-jazzy-canopen \
    ros-jazzy-canopen-fake-slaves \
    ros-jazzy-canopen-proxy-driver \
    ros-jazzy-canopen-master-driver \
    ros-jazzy-canopen-402-driver \
    # -----------------------------------
    && rm -rf /var/lib/apt/lists/*

# Create can group and add root user to it
RUN groupadd -f can && usermod -a -G can root

WORKDIR /workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=150" >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc

CMD ["bash"]
