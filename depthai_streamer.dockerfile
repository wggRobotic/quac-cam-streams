FROM ros:humble
SHELL ["/bin/bash", "-c"]

# depthai sdk

WORKDIR /workspace
RUN apt update && apt install git cmake libopencv-dev -y
RUN git clone https://github.com/luxonis/depthai-core.git

WORKDIR /workspace/depthai-core
RUN git submodule update --init --recursive
RUN cmake -S . -B build -D'BUILD_SHARED_LIBS=ON'
RUN cmake --build build --target install

# ros2 package dependencies

RUN apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgstrtspserver-1.0-dev \
    libyaml-cpp-dev \
    libssl-dev \
    wget \
    build-essential \
    pkg-config

RUN apt install -y libusb-1.0-0 libusb-1.0-0-dev

RUN apt install -y ros-humble-rmw-cyclonedds-cpp

# quac

WORKDIR /quac

COPY ./quac-interfaces /quac/src/quac-interfaces
RUN . /opt/ros/humble/setup.bash && colcon build

COPY ./quac_cam_streams /quac/src/quac_cam_streams
RUN . /opt/ros/humble/setup.bash && . /quac/install/setup.bash && colcon build --cmake-args -DBUILD_DEPTHAI_STREAMER=ON