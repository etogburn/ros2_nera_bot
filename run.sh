docker run -it --network=host --ipc=host -v /dev:/dev --device-cgroup-rule='c 189:* rmw' --privileged ghcr.io/etogburn/ros2_nera_bot:robot-latest
