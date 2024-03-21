# arm-control-daemon

# 1. Introduction

This is the web interface for [`arm-control`](https://git.qiuzhi.tech/airbot-play/control/arm-control). In other words, with `arm-control-daemon`, HTTP clients are able to control or query the airbot play bot via HTTP requests.

# 2. Install

## 2.1 Build from sources

### 2.1.1 Install `arm-control` from sources

See [`arm-control`](https://git.qiuzhi.tech/airbot-play/control/arm-control).

### 2.1.2 Clone this repository
```bash
git clone --recursive https://git.qiuzhi.tech/airbot-play/control/arm-control-daemon.git
```

### 2.1.3 Install the thirdparty dependencies 
Run following commands with root privileges:
```bash
pushd arm-control-daemon/thirdparty/eigen
# eigen version 3.2.x is needed. Please check before proceed
mkdir build && cd build && cmake .. && make -j8 && make install
popd

pushd arm-control-daemon/thirdparty/httplib
mkdir build && cd build && cmake .. && make -j8 && make install
popd

pushd arm-control-daemon/thirdparty/websocketpp
mkdir build && cd build && cmake .. && make -j8 && make install
popd
```

### 2.1.4 Install `arm-control-daemon`

```bash
mkdir build && cd build && cmake .. && make -j8 && make install
```

## 2.2 Install via Docker

```bash
docker pull registry.qiuzhi.tech/airbot-play/arm-control-daemon:main
```

# 3. Usage

After starting, the program will listen on `8888`
(HTTP) and `9999` (Websocket) port for incoming queries or control commands. Detailed HTTP APIs will be updated soon.

To use `arm-control-daemon` control an arm to follow the movement of another arm, please refer to the remote arguments in [`arm-control`](https://git.qiuzhi.tech/airbot-play/control/arm-control#311-run-with-local-built-binaries). The listening ports of `arm-control-daemon` and the arguments passed to an `arm-control` with `-r` should be identical. The IP address of the host running `arm-control-daemon` should also be included in the `-r` argument. E.g., `-r 192.168.112.215:8888`.

## 3.1 Built from sources

In the `build/` subdirectory:
```bash
# To be updated
```

## 3.2 Docker

```shell
docker run -it --rm --name arm-control-daemon --network host --privileged registry.qiuzhi.tech/airbot-play/arm-control-daemon:main build/arm-control-daemon models/airbot_play_v2_1/urdf/airbot_play_v2_1.urdf can0 8888 9999

docker run -it --rm --name remote-sync --network host --privileged registry.qiuzhi.tech/airbot-play/arm-control-daemon:main build/remotesync models/airbot_play_v2_1/urdf/airbot_play_v2_1.urdf can0 http://192.168.112.217:8888 ws://192.168.112.217:9999

```