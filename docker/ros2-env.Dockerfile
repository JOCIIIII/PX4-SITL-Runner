# Derived humble ROS2 SILS image that adds the uXRCE-DDS agent.
#
# The base humble image already has foxglove_bridge, mavros_msgs, pycuda, and runs
# the prebuilt airsim_ros_pkgs binary natively. It is MISSING MicroXRCEAgent (which
# the jazzy image had as a system binary, and which the original humble setup pulled
# from the broken px4_ros tarball). We build Micro-XRCE-DDS-Agent from source here so
# /usr/local/bin/MicroXRCEAgent is available (matching the jazzy image's v2.4.3).
#
# Build:
#   docker build -f docker/ros2-env.Dockerfile \
#     -t reg.iacsl.org/a4-vai/sils-container:humble-cuda12.6-tensorrt10.7-cudnn9.7-uxrce .

ARG BASE=reg.iacsl.org/a4-vai/sils-container:humble-cuda12.6-tensorrt10.7-cudnn9.7
FROM ${BASE}

# The base image runs as the non-root "user"; build steps need root.
USER root

# Build + install Micro-XRCE-DDS-Agent (superbuild fetches Fast-DDS/Fast-CDR).
# Build tools (git/cmake/g++/make/libssl-dev) already exist in the image, so no apt
# is needed - its third-party repos are broken anyway (Isaac ROS Origin change, yarn key).
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent /tmp/uagent \
 && cmake -S /tmp/uagent -B /tmp/uagent/build \
      -DUAGENT_BUILD_EXECUTABLE=ON -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /tmp/uagent/build -j"$(nproc)" \
 && install -m 0755 "$(find /tmp/uagent -type f -perm -u+x -name MicroXRCEAgent | head -1)" /usr/local/bin/MicroXRCEAgent \
 && find /tmp/uagent -type f -name '*.so*' -exec cp -an {} /usr/local/lib/ \; \
 && ldconfig \
 && rm -rf /tmp/uagent

# Run as root (like the jazzy image did) so the container can write build artifacts
# into the host-mounted ros2_ws (whose files are owned by the host user, not uid 1001).
USER root
