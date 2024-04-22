import can
import struct
import os
import time
import rich.progress


def send_can_data(bus, can_id, data):
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)


def receive_can_data(bus):
    msg = bus.recv(0.001)
    return msg


def send_request_data(bus, can_id):
    request_data = struct.pack("B", 0x00)
    send_can_data(bus, can_id, request_data)


def send_end_request_data(bus, can_id):
    end_request_data = struct.pack("B", 0x00)
    send_can_data(bus, can_id, end_request_data)


def send_version_request_data(bus, can_id):
    version_request_data = struct.pack("B", 0x02)
    send_can_data(bus, can_id, version_request_data)


def main(can_channel: str, device_id: int, firmware_path: str, device_name: str):
    # 开始计时
    start_time = time.time()
    # 输入设备id
    assert device_id >= 0, "Device ID must be greater than or equal to 0"
    # 初始化CAN总线，根据实际情况设置接口和波特率
    bus = can.interface.Bus(channel=can_channel, bustype="socketcan", bitrate=1000000)
    if device_name == "":
        start_ask_name_time = time.time()
        frame_index = 0
        send_can_data(bus, device_id, struct.pack("B", 0x07))
        while time.time() - start_ask_name_time < 2:
            time.sleep(0.1)
            response_msg = receive_can_data(bus)
            if (
                response_msg is not None
                and response_msg.arbitration_id == 0x100 | device_id
            ):
                response_data = struct.unpack(">BB4s", response_msg.data)
                response_index1, response_index2, response_status = response_data
                if response_index1 == 0x07:
                    if frame_index + 1 == response_index2:
                        device_name += response_status.decode("utf-8")
                        frame_index += 1
        device_name = device_name.strip("\x00")

    if device_name == "":
        print("No device name detected. Please retry and input device name.")
        return

    print("Device name:", device_name)

    version = 0
    version_ask_flag = False
    while not version_ask_flag:
        send_version_request_data(bus, device_id)
        time.sleep(0.1)
        response_msg = receive_can_data(bus)
        if (
            response_msg is not None
            and response_msg.arbitration_id == 0x100 | device_id
        ):
            if len(response_msg.data) == 6:
                response_data = struct.unpack(">BB4s", response_msg.data)
                response_index1, response_index2, response_status = response_data
            elif len(response_msg.data) == 8:
                response_data = struct.unpack(">BB4s2s", response_msg.data)
                response_index1, response_index2, response_status, crc = response_data
            if response_index1 == 0x02 and response_index2 == 0x01:
                print("Device detected.")
                print(
                    "Version: ",
                    response_status[1],
                    ".",
                    response_status[2],
                    ".",
                    response_status[3],
                    sep="",
                )
                version = ".".join(
                    [
                        str(response_status[1]),
                        str(response_status[2]),
                        str(response_status[3]),
                    ]
                )
                version_ask_flag = True

    # 发送请求数据
    send_request_data(bus, 0x600 | device_id)

    # 等待接收返回数据，确保请求成功
    while True:
        response_msg = receive_can_data(bus)
        if (
            response_msg is not None
            and response_msg.arbitration_id == 0x780 | device_id
        ):
            response_data = struct.unpack("B", response_msg.data)
            if response_data[0] == 0x01:
                print("Request successful. Starting data transmission.")
                break
        time.sleep(0.1)

    # 读取bin文件
    assert os.path.exists(
        firmware_path
    ), f"Firmware file {firmware_path} does not exist"
    with open(firmware_path, "rb") as file:
        bin_data = file.read()

    # 设置CAN ID和起始索引
    can_id = 0x680 | device_id
    index1 = 0
    index2 = 0

    if device_name == "arm-interface-board-end":
        time.sleep(3)

    with rich.progress.Progress() as progress:
        task = progress.add_task("[cyan]Transmitting data...", total=len(bin_data) / 6)
        # 发送CAN数据
        while index1 * 256 + index2 < len(bin_data) / 6:  # 当所有数据都成功发送完时结束
            # 构建CAN数据帧
            can_data = (
                struct.pack(">BB", index1 % 256, index2 % 256)
                + bin_data[
                    index1 * 256 * 6 + index2 * 6 : index1 * 256 * 6 + index2 * 6 + 6
                ]
            )

            # 发送CAN数据
            send_can_data(bus, can_id, can_data)
            if version >= "2.5.0" and device_name != "arm-interface-board-end":
                index2 += 1
                time.sleep(0.0001)
                if index2 == 256:
                    # 等待接收返回数据
                    response_msg = receive_can_data(bus)

                    while (
                        response_msg == None
                        or response_msg.arbitration_id != 0x780 | device_id
                    ):
                        # print(response_msg)
                        response_msg = receive_can_data(bus)

                    # print(response_msg)

                    # 处理返回数据
                    response_data = struct.unpack(">BB6s", response_msg.data)
                    response_index1, response_index2, response_status = response_data
                    # print (response_index1, response_index2, response_status, response_crc)
                    # 检查返回数据是否成功
                    index2 = 0
                    if response_status[0] == 0x01:
                        progress.update(task, advance=256)
                        # print(f"Packet {index1} sent successfully.")
                        # time.sleep(0.05)
                    else:
                        print(f"Packet {index1} failed. Retrying...")
                        continue
            else:
                response_msg = receive_can_data(bus)
                # 处理返回数据
                if (
                    response_msg is not None
                    and response_msg.arbitration_id == 0x780 | device_id
                ):
                    response_data = struct.unpack(">BB6s", response_msg.data)
                    response_index1, response_index2, response_status = response_data
                    # print (response_index1, response_index2, response_status, response_crc)
                    # 检查返回数据是否成功
                    if response_status[0] == 0x01:
                        progress.update(task, advance=1)
                        # print(f"Frame {index1}-{index2} sent successfully.")
                        index2 += 1
                        if index2 == 256:
                            index2 = 0
                        # time.sleep(0.01)
                    else:
                        print(f"Frame {index1}-{index2} failed. Retrying...")
                        continue
                else:
                    print(
                        f"Error receiving response for Frame {index1}-{index2}. Retrying..."
                    )
                    continue

            # 更新索引
            if index2 == 0:
                index1 += 1

    # 发送结束请求数据
    send_end_request_data(bus, 0x700 | device_id)

    # 等待接收结束请求的返回数据，确保请求成功
    while True:
        response_msg = receive_can_data(bus)
        if (
            response_msg is not None
            and response_msg.arbitration_id == 0x780 | device_id
        ):
            response_data = struct.unpack("B", response_msg.data)
            if response_data[0] == 0x02:
                # print(f"Packet {index1} sent successfully.")
                print("End request successful. Data transmit finished.")
                print(
                    "\033[0;31m",
                    "Please wait for the program auto close and don't shutdown the power supply now.",
                    "\033[0m",
                    sep="",
                )
                break
        time.sleep(0.1)

    burned_flag = False
    with rich.progress.Progress() as burn_progress:
        burn_task = burn_progress.add_task("[green]Burning firmware...", total=100)
        progress_cnt = 0
        while not burned_flag:
            time.sleep(0.1)
            if progress_cnt < 40:
                progress_cnt += 0.75
                burn_progress.update(burn_task, completed=progress_cnt)
                continue
            send_version_request_data(bus, device_id)
            progress_cnt = min(progress_cnt + 0.75, 99)
            burn_progress.update(burn_task, completed=progress_cnt)
            response_msg = receive_can_data(bus)
            if (
                response_msg is not None
                and response_msg.arbitration_id == 0x100 | device_id
            ):
                if len(response_msg.data) == 6:
                    response_data = struct.unpack(">BB4s", response_msg.data)
                    response_index1, response_index2, response_status = response_data
                elif len(response_msg.data) == 8:
                    response_data = struct.unpack(">BB4s2s", response_msg.data)
                    response_index1, response_index2, response_status, crc = (
                        response_data
                    )
                if response_index1 == 0x02 and response_index2 == 0x01:
                    burn_progress.update(burn_task, completed=100)
                    burned_flag = True

    print("Burn finished.")
    print(
        "Version: ",
        response_status[1],
        ".",
        response_status[2],
        ".",
        response_status[3],
        sep="",
    )
    # 结束计时
    end_time = time.time()
    print(f"Time elapsed: {end_time - start_time} seconds")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="IAP burn tool")
    parser.add_argument("firmware_path", type=str, help="Firmware path")
    parser.add_argument("-i", "--can-id", type=int, help="", default=-1)
    parser.add_argument(
        "-m", "--can-interface", type=str, help="CAN interface", default="can0"
    )
    parser.add_argument(
        "-n",
        "--device-name",
        type=str,
        help="Device name: arm-interface-board-end or vesc-motor-control",
        default="",
    )
    args = parser.parse_args()

    if args.can_id == -1:
        args.can_id = int(input("Please input device can id: "))

    main(args.can_interface, args.can_id, args.firmware_path, args.device_name)
