# ======================
# Base build stage
# ======================
FROM ros:jazzy-ros-base

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
WORKDIR /ros2_ws
COPY ./src ./

# Resolve dependencies and build
RUN apt-get update && \
    . /opt/ros/jazzy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build

# Source ROS and workspace on container start
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && exec bash"]


