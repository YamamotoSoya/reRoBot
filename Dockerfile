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
    vim

WORKDIR /root

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=150" >> ~/.bashrc \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc \
    echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc

CMD ["bash"]
