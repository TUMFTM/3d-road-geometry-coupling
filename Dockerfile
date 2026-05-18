FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-pip \
        libeigen3-dev \
        libboost-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src/3d-road-geometry-coupling

COPY . .

RUN git submodule update --init --recursive || true

WORKDIR /ros2_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
        --packages-up-to \
            tum_road_geometry_coupling_nodes_cpp \
            tum_road_geometry_coupling_py \
            tum_road_plane_follower_cpp \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
