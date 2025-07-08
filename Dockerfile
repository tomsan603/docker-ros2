# ベースイメージ: ROS 2 jazzy
FROM ros:jazzy

# 環境変数: インタラクティブでないインストール用
ENV DEBIAN_FRONTEND=noninteractive

# 基本開発ツール
RUN apt-get update -q && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    vim \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz\
    ros-jazzy-rviz2\
    ros-jazzy-moveit\
    && rm -rf /var/lib/apt/lists/*

# ワークスペース作成
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# UR関連リポジトリをクローン
RUN cd /ros2_ws/src && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git -b ros2 --depth 1 && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git -b jazzy --depth 1 && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git -b humble --depth 1 && \
    git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git --depth 1 && \
    git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git && \
    git clone https://github.com/ian-chuang/serial-ros2.git && \
    git clone https://github.com/panagelak/rq_fts_ros2_driver.git


# 依存関係をすべて解決
RUN apt-get update -q && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# ワークスペースビルド
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 環境設定をbashrcに
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# エントリーポイントとコマンド
ENTRYPOINT []
CMD ["/bin/bash"]