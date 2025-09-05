# ======================
# Base build stage
# ======================
FROM ros:jazzy-ros-base AS base

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
WORKDIR /ros2_ws
COPY ./src ./


# ======================
# Runtime image for Robot
# ======================
FROM ros:jazzy-ros-base AS robot

WORKDIR /ros2_ws
COPY --from=base /ros2_ws /ros2_ws
# COPY --from=base /opt/ros/jazzy /opt/ros/jazzy

# Resolve dependencies and build
RUN apt-get update && \
    . /opt/ros/jazzy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Source ROS and workspace on container start
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && exec bash"]

# ======================
# Runtime image for Dev (with GUI tools)
# ======================
FROM osrf/ros:jazzy-desktop AS dev

RUN apt update && apt install -y \
    git vim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY --from=base /ros2_ws /ros2_ws
# COPY --from=base /opt/ros/jazzy /opt/ros/jazzy

# Resolve dependencies and build
RUN apt-get update && \
    . /opt/ros/jazzy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# GUI env for RViz/Gazebo
ENV QT_X11_NO_MITSHM=1

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && exec bash"]
