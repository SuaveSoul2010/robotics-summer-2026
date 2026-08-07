FROM ros:foxy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    build-essential \
    bison \
    flex \
    clang-tidy \
    libasio-dev \
    usbutils \
    wireless-tools \
    gazebo11 \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-gazebo-ros2-control \
    ros-foxy-imu-filter-madgwick \
    ros-foxy-interactive-marker-twist-server \
    ros-foxy-nav2-bringup \
    ros-foxy-navigation2 \
    ros-foxy-robot-localization \
    ros-foxy-ros2-control \
    ros-foxy-ros2-controllers \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-twist-mux \
    ros-foxy-velodyne-description \
    ros-foxy-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

WORKDIR /jackal_ws

COPY . .

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install"

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
RUN echo "source /jackal_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]