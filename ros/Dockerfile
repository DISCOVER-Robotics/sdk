FROM --platform=$TARGETPLATFORM registry.qiuzhi.tech/airbot-play/arm-control:main as dep

FROM --platform=$TARGETPLATFORM registry.qiuzhi.tech/library/ros:noetic AS build-dep
COPY --from=dep /usr/local /usr/local
ADD ros /opt/rosif_ws/src/ros_interface
WORKDIR /opt/rosif_ws/src/ros_interface
RUN apt-key add ros.asc

WORKDIR /opt

RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirrors.tuna.tsinghua.edu.cn/ubuntu/|' /etc/apt/sources.list
RUN sed -i 's|http://ports.ubuntu.com/|http://mirrors.tuna.tsinghua.edu.cn/|' /etc/apt/sources.list
RUN echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros1-latest.list

RUN apt update \
        && DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai apt install -y \
            build-essential cmake ros-noetic-kdl-parser ros-noetic-tf ros-noetic-joy \
            iproute2 python3 python3-pip libkdl-parser-dev libompl-dev libassimp-dev libspdlog-dev libfmt* \
        && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/rosif_ws
RUN . /opt/ros/noetic/setup.sh && catkin_make
RUN echo "source /opt/rosif_ws/devel/setup.bash" >> ~/.bashrc
CMD . ./devel/setup.sh && roslaunch ros_interface airbot_arm.launch
