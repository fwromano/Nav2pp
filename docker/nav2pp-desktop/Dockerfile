FROM ros:jazzy

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    dbus-x11 \
    fluxbox \
    mesa-utils \
    novnc \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-desktop \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim \
    ros-jazzy-navigation2 \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    websockify \
    x11vnc \
    xterm \
    xvfb \
 && rm -rf /var/lib/apt/lists/*

COPY docker/nav2pp-desktop/entrypoint.sh /usr/local/bin/nav2pp-entrypoint

RUN chmod +x /usr/local/bin/nav2pp-entrypoint

EXPOSE 6080

ENTRYPOINT ["/usr/local/bin/nav2pp-entrypoint"]
