FROM ghcr.io/aica-technology/ros-ws:noetic

RUN sudo apt update && sudo apt install -y ros-noetic-turtlesim && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USER}/ros_ws
COPY --chown=${USER} ./turtle_example ./src/
