# Base Image: ROS2 Jazzy
FROM ros:jazzy

# Environment variable for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install basic development tools
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
    ros-jazzy-ros-gz \
    ros-jazzy-rviz2 \
    ros-jazzy-moveit \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# For ROS board
RUN pip install --break-system-packages tornado simplejpeg

# Create a ROS 2 workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Clone UR && Gripper && FTsensor && ROS board related repositories
RUN cd /ros2_ws/src && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git -b ros2 --depth 1 && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git -b jazzy --depth 1 && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git -b jazzy --depth 1 && \
    git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git --depth 1 && \
    git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git && \
    git clone https://github.com/ian-chuang/serial-ros2.git && \
    git clone https://github.com/panagelak/rq_fts_ros2_driver.git &&\
    git clone https://github.com/dheera/rosboard.git

# Install dependencies
RUN apt-get update -q && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the environment setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Entrypoint and Command
ENTRYPOINT []
CMD ["/bin/bash"]