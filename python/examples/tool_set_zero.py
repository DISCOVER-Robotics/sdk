import argparse
import threading
import time
import airbot

# ANSI颜色代码
red = "\033[0;31m"
green = "\033[0;32m"
yellow = "\033[0;33m"
blue = "\033[0;34m"
cyan = "\033[0;36m"
reset = "\033[0m"

# 常量定义
MOTOR_DOWN_ANGLE = [0.0, 0.1922, -0.1376]
RANGE = [0.1, 0.1, 0.1]


# 判断是否在指定范围内
def in_range(input):
    return (
        MOTOR_DOWN_ANGLE[0] - RANGE[0] < input[0] < MOTOR_DOWN_ANGLE[0] + RANGE[0]
        and MOTOR_DOWN_ANGLE[1] - RANGE[1] < input[1] < MOTOR_DOWN_ANGLE[1] + RANGE[1]
        and MOTOR_DOWN_ANGLE[2] - RANGE[2] < input[2] < MOTOR_DOWN_ANGLE[2] + RANGE[2]
    )


def main():
    # 命令行参数解析
    parser = argparse.ArgumentParser(
        description="Set zero points for the motors of AIRBOT Play."
    )
    parser.add_argument(
        "-m",
        "--master",
        required=True,
        default="can0",
        help="Can device interface of the master arm. Default: can0",
    )
    parser.add_argument(
        "-e",
        "--master-end-mode",
        default="none",
        choices=["teacher", "gripper", "yinshi", "newteacher", "none"],
        help="The mode of the master arm end effector.",
    )
    parser.add_argument(
        "--forearm-type",
        default="DM",
        choices=["DM", "OD"],
        help="The type of forearm.",
    )

    args = parser.parse_args()

    interface = args.master
    gripper_type = args.master_end_mode
    forearm_type = args.forearm_type

    # 初始化电机驱动
    motor_driver = []
    for i in range(1, 7 if forearm_type == "OD" else 4):
        motor = airbot.create_motor(i, interface, "OD")
        motor.MotorInit()
        motor.set_motor_control_mode(1)
        motor_driver.append(motor)
    if forearm_type == "DM":
        for i in range(4, 7):
            motor = airbot.create_motor(i, interface, "DM")
            motor.MotorInit()
            motor.set_motor_control_mode(1)
            motor_driver.append(motor)

    if gripper_type in ["newteacher", "gripper", "teacher"]:
        motor_gripper = airbot.create_motor(
            7, interface, "OD" if gripper_type == "newteacher" else "DM"
        )
        motor_gripper.MotorInit()
        motor_gripper.set_motor_control_mode(1)
        motor_driver.append(motor_gripper)

    # 控制标零流程
    release_brake = 1
    mutex = threading.Lock()

    def control_brake():
        nonlocal release_brake
        while True:
            if mutex.acquire(timeout=0.1):
                brake = release_brake
                mutex.release()
                if brake == 0:
                    continue
                elif brake == 2:
                    break
                for i in range(3):
                    motor_driver[i].set_motor_control_mode(1)
                    time.sleep(0.001)
                    motor_driver[i].MotorMitModeCmd(0, 0, 0, 2, 0)
                    time.sleep(0.001)
                for i in range(3, 6):
                    motor_driver[i].set_motor_control_mode(1)
                    time.sleep(0.001)
                    motor_driver[i].MotorMitModeCmd(
                        0, 0, 0, 1 if forearm_type == "DM" else 0.1, 0
                    )
                    time.sleep(0.001)
                if len(motor_driver) == 7:
                    motor_driver[6].set_motor_control_mode(1)
                    time.sleep(0.001)
                    motor_driver[6].MotorMitModeCmd(0, 0, 0, 0, 0)
                    time.sleep(0.001)

    # 启动控制线程
    thread_brake = threading.Thread(target=control_brake)
    thread_brake.start()

    # 提示操作
    print(blue + "放入标零工具，将机械臂牵引到零点后，按下回车键" + reset)
    input()

    mutex.acquire()
    release_brake = 0
    mutex.release()

    for motor in motor_driver:
        motor.MotorSetZero()
        time.sleep(0.01)

    print(green + "标零完成。" + reset)

    mutex.acquire()
    release_brake = 1
    mutex.release()

    print(blue + "移去标零工具，按下回车键" + reset)
    input()

    mutex.acquire()
    release_brake = 2
    mutex.release()

    thread_brake.join()

    for motor in motor_driver:
        motor.MotorDeInit()


if __name__ == "__main__":
    main()
