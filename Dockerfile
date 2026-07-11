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
    # --- Visualization ---
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-tf-tree \
    ros-jazzy-tf2-tools \
    # --- CANopen ---
    can-utils \
    ros-jazzy-canopen \
    ros-jazzy-canopen-fake-slaves \
    ros-jazzy-canopen-proxy-driver \
    ros-jazzy-canopen-master-driver \
    ros-jazzy-canopen-402-driver \
    # --- HOKUYO urg_node ---
    ros-jazzy-urg-node \
    # --- Xbox / joystick teleop --- # claude
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    # --- slam toolbox ---
    ros-jazzy-slam-toolbox \
    # --- navigation2 ---
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb* \
    # --- sensor integration (EKF,UKF) ---
    ros-jazzy-robot-localization \
    # --- LiDAR (Sure-Star RFans/CFans rfans_driver) ---
    libpcap-dev \
    # --- LIO-SAM (LiDAR odometry, src/external/LIO-SAM) --- # claude
    # 公式 (TixiaoShan/LIO-SAM) の依存を ROS 2 Jazzy 用に読み替えたもの:
    #   PCL / cv_bridge(OpenCV) / tf2 / Boost::timer + GTSAM。
    #   navigation・robot_localization・robot_state_publisher は
    #   上の navigation2 / sensor integration / ros-base で導入済み。
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-tf2-geometry-msgs \
    libpcl-dev \
    libboost-timer-dev \
    # 公式は ppa:borglab/gtsam-release-4.0 を使うが noble(24.04) 非対応。
    # universe の libgtsam-dev は gtsam_unstable ヘッダ
    # (imuPreintegration.cpp が include する IncrementalFixedLagSmoother.h)
    # を含まず、libgtsam-unstable-dev は noble に存在しない。
    # → gtsam_unstable 同梱の ros-jazzy-gtsam (GTSAM 4.2.0) を使う。 # claude
    ros-jazzy-gtsam \
    # 6軸IMU(RealSense) → orientation付与 (LIO-SAM入力用) # claude
    ros-jazzy-imu-filter-madgwick \
    # -----------------------------------
    && rm -rf /var/lib/apt/lists/*

# Create can group and add root user to it
RUN groupadd -f can && usermod -a -G can root

# --- librealsense (Intel RealSense SDK) ---
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-librealsense2* \
     && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=150" >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc

CMD ["bash"]
