ARG REF=main
ARG NPROC=4
FROM --platform=$TARGETPLATFORM registry.qiuzhi.tech/airbot-play/arm-control:${REF}

WORKDIR /opt

ADD include/ ./arm-control-daemon/include/
ADD src/ ./arm-control-daemon/src/
ADD thirdparty/ ./arm-control-daemon/thirdparty/
ADD models/ ./arm-control-daemon/models/
ADD CMakeLists.txt ./arm-control-daemon/

RUN sed -i 's|http://archive.ubuntu.com/|http://mirrors.tuna.tsinghua.edu.cn/|' /etc/apt/sources.list
RUN sed -i 's|http://ports.ubuntu.com/|http://mirrors.tuna.tsinghua.edu.cn/|' /etc/apt/sources.list

RUN apt-get update && apt-get install -y iproute2

WORKDIR /opt/arm-control-daemon/thirdparty/httplib
RUN mkdir build && cd build && cmake .. && make -j${NPROC} && make install

WORKDIR /opt/arm-control-daemon/thirdparty/websocketpp
RUN mkdir build && cd build && cmake .. && make -j${NPROC} && make install

WORKDIR /opt/arm-control-daemon/thirdparty/eigen
RUN mkdir build && cd build && cmake .. && make -j${NPROC} && make install

WORKDIR /opt/arm-control-daemon
RUN mkdir build && cd build && cmake .. && make -j${NPROC}

RUN ldconfig
