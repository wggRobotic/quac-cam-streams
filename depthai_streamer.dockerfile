FROM ros:humble
SHELL ["/bin/bash", "-c"]

# depthai sdk

WORKDIR /workspace
RUN apt update && apt install git cmake -y
RUN git clone https://github.com/luxonis/depthai-core.git

WORKDIR /workspace/depthai-core
RUN git submodule update --init --recursive
RUN cmake -S . -B build -D'BUILD_SHARED_LIBS=ON' -D'CMAKE_INSTALL_PREFIX=/workspace/depthai-core/install'
RUN cmake --build build --target install

# ros2 package dependencies

RUN apt install -y ros-humble-rmw-cyclonedds-cpp

# quac

WORKDIR /quac

COPY ./quac-interfaces /quac/src/quac-interfaces
RUN . /opt/ros/humble/setup.bash && colcon build

COPY ./quac_cam_streams /quac/src/quac_cam_streams
RUN . /opt/ros/humble/setup.bash && . /quac/install/setup.bash && colcon build --cmake-args -DBUILD_DEPTHAI_STREAMER=ON