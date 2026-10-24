# The onboard base image for a fresh Jetson (JetPack 6 / L4T r36): what scripts/dev/pils_jetson_setup.sh expects of
# its PILS_BASE_IMAGE (orin-nano-01's ros2-px4-jetson:humble-a4vai-hybrid, which was built by hand there):
#   ROS 2 Humble (ros-base) + colcon, MicroXRCEAgent v2.4.3 (v3 cannot discover Humble), onnxruntime (CPU),
#   numpy < 2, scipy, pycuda (PathFollowing's MPPI kernel), foxglove_bridge, PCL + pcl_conversions,
#   the ROS packages the apps and the AirSim bridge use, and JetPack's own OpenCV 4.8 (cv_bridge is built
#   against it by pils_jetson_setup.sh)
# Built by scripts/dev/jetson_base_setup.sh on the Jetson.
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

ENV DEBIAN_FRONTEND=noninteractive LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 ROS_DISTRO=humble \
    PATH=/usr/local/cuda/bin:${PATH} LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

RUN apt-get update && apt-get install -y --no-install-recommends \
        locales curl gnupg2 lsb-release ca-certificates git build-essential cmake pkg-config \
        python3-pip python3-dev libpcl-dev libeigen3-dev libyaml-cpp-dev \
    && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions \
        ros-humble-foxglove-bridge ros-humble-pcl-conversions ros-humble-pcl-ros \
        ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs ros-humble-tf2-eigen \
        ros-humble-sensor-msgs-py ros-humble-nav-msgs ros-humble-geometry-msgs \
        ros-humble-image-transport ros-humble-geographic-msgs ros-humble-mavros-msgs \
        ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# numpy < 2 (Humble's rclpy / cv_bridge), the policies' runtime (CPU onnxruntime), PathFollowing's pycuda
RUN pip3 install --no-cache-dir "numpy>=1.24,<2" scipy pyyaml onnxruntime pycuda

# Micro-XRCE-DDS-Agent v2.4.3 (PX4's uXRCE-DDS, serial to the Pixhawk / UDP to SITL)
RUN git clone -q -b v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce-agent \
    && cmake -S /tmp/xrce-agent -B /tmp/xrce-agent/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/xrce-agent/build -j"$(nproc)" && cmake --install /tmp/xrce-agent/build \
    && ldconfig /usr/local/lib && rm -rf /tmp/xrce-agent

WORKDIR /home/user/workspace
CMD ["bash"]
