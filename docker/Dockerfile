FROM arm64v8/ros:humble-perception
#FROM osrf/ros:humble-desktop-full

# キーボード設定の選択を省略
ARG DEBIAN_FRONTEND=noninteractive

# Install apt packages
RUN apt-get update && apt-get install -y \
  build-essential \
  gdb \
  cmake \
  git \
  vim \
  wget \
  tmux \
  x11-apps \
  terminator \
  libfreetype6-dev \
  libpng-dev \
  iputils-ping \
  net-tools \
  libmodbus-dev \
  tree \
  libpcl-dev

# Install python packages
RUN apt-get install -y \
  python3-pykdl \
  python3-pip \
  python3-libgpiod

# Install ROS packages
RUN apt-get update && apt-get install -y \
  ros-humble-urg-node \
  ros-humble-sensor-msgs \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-tf2 \
  ros-humble-joy \ 
  ros-humble-teleop-twist-joy \
  ros-humble-joint-state-publisher-gui \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-rclcpp && \
  apt-get clean && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# Install pip packages
# pip install numpy==1.24.4
RUN pip install --no-cache-dir \
  pyserial==3.5 \
  pykalman==0.9.7 \
  spidev==3.6 \
  rpi-lgpio==0.6 \
  gpiozero==2.0.1 \
  transforms3d==0.4.2 \
  matplotlib==3.9.0

# entrypoint.sh スクリプトをコンテナにコピー
COPY entrypoint.sh /entrypoint.sh

# スクリプトを実行可能にする
RUN chmod +x /entrypoint.sh

# スクリプトをエントリーポイントとして設定
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Create ROS workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN mkdir -p /root/icart_ws/src
WORKDIR /root/icart_ws/src

RUN echo 'source /root/icart_ws/src/icart_mini_ros2/docker/before_bringup.sh' >> ~/.bashrc
RUN echo 'alias cm="cd ~/icart_ws && colcon build --symlink-install --packages-select icart_mini_ros2"' >> ~/.bashrc
