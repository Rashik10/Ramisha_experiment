FROM ros:humble-ros-base-jammy

# ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# apt packages
RUN apt-get update && apt-get install -y python3-rosdep usbutils vim

# forcedimension package
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN git clone https://github.com/ICube-Robotics/forcedimension_ros2.git
RUN vcs import . < forcedimension_ros2/forcedimension_ros2.repos
RUN apt-get update && rosdep install --ignore-src --from-paths . -y -r
RUN . /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install && . install/setup.sh

# pip packages
RUN apt-get install -y python3-pip
RUN pip3 install setuptools cvxopt paho-mqtt

WORKDIR /