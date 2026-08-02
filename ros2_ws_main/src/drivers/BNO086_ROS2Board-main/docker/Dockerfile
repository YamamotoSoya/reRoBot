# ROS 2 development environment for the BNO086 IMU board.
# Built for macOS hosts, where ROS 2 cannot be installed natively.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial \
        python3-pytest \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-diagnostic-msgs \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-std-srvs \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-imu-tools \
        ros-${ROS_DISTRO}-rqt-plot \
    && rm -rf /var/lib/apt/lists/*

# Source the overlay automatically for interactive shells.
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["/bin/bash"]
