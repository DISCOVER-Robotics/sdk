import curses
import os
import time
import math
import json
import airbot


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def main(stdscr):
    # 设置curses
    stdscr.nodelay(True)
    curses.curs_set(0)  # 隐藏光标
    stdscr.clear()  # 清空屏幕
    stdscr.refresh()

    record = False

    # 初始化机器人
    urdf_path = airbot.AIRBOT_PLAY_WITH_GRIPPER_URDF

    try:
        robot = airbot.create_agent(urdf_path, "down", "can0", 1.0, "gripper", False)
    except RuntimeError as e:
        print(e)
        exit(1)

    traj_list = []
    # 控制参数
    TIME = 1.5
    angle_step = 0.1
    step = 0.01
    use_planning = True
    from_base = True
    stdscr.clear()  # 清空屏幕
    running = True
    while running:
        stdscr.refresh()
        key = stdscr.getch()

        if key in [ord("z"), 3]:  # 'z'键或Ctrl+C退出
            running = False

        if key == ord("y"):
            use_planning = not use_planning

        # 控制关节角度
        joint_movement = {
            ord("1"): (angle_step, 0, 0, 0, 0, 0),
            ord("2"): (-angle_step, 0, 0, 0, 0, 0),
            ord("3"): (0, angle_step, 0, 0, 0, 0),
            ord("4"): (0, -angle_step, 0, 0, 0, 0),
            ord("5"): (0, 0, angle_step, 0, 0, 0),
            ord("6"): (0, 0, -angle_step, 0, 0, 0),
            ord("7"): (0, 0, 0, angle_step, 0, 0),
            ord("8"): (0, 0, 0, -angle_step, 0, 0),
            ord("9"): (0, 0, 0, 0, angle_step, 0),
            ord("0"): (0, 0, 0, 0, -angle_step, 0),
            ord("-"): (0, 0, 0, 0, 0, angle_step),
            ord("="): (0, 0, 0, 0, 0, -angle_step),
        }

        if key in joint_movement:
            robot.add_target_joint_q(joint_movement[key], use_planning, TIME)

        joint_rotations = {
            ord("j"): (0, 0, angle_step),
            ord("l"): (0, 0, -angle_step),
            ord("i"): (0, angle_step, 0),
            ord("k"): (0, -angle_step, 0),
            ord("u"): (angle_step, 0, 0),
            ord("o"): (-angle_step, 0, 0),
        }

        if key in joint_rotations:

            robot.add_target_relative_rotation(
                euler_to_quaternion(*(joint_rotations[key])), use_planning, TIME
            )

        # 控制位置移动
        translation_movement = {
            ord("w"): (step, 0, 0),
            ord("s"): (-step, 0, 0),
            ord("a"): (0, step, 0),
            ord("d"): (0, -step, 0),
            ord("q"): (0, 0, step),
            ord("e"): (0, 0, -step),
        }

        if key in translation_movement:
            if from_base:
                robot.add_target_translation(
                    translation_movement[key], use_planning, TIME
                )
            else:
                robot.add_target_relative_translation(
                    translation_movement[key], use_planning, TIME
                )

        if key == ord("r"):
            from_base = not from_base

        if key == ord("`"):
            robot.set_target_joint_q((0, 0, 0, 0, 0, 0), use_planning, TIME)

        if key == 32:
            record = not record

        # 其他控制
        if key == ord("t"):
            step = 0.1 if step == 0.01 else 0.01

        if record:
            traj_list.append(robot.get_current_joint_q())

        # 延迟以防止频繁操作
        time.sleep(0.002)

    curses.endwin()

    if record:
        # 定义要保存的 JSON 文件路径
        record = "trajectory/record.json"

        # 使用 json.dump() 将嵌套列表写入 JSON 文件
        with open(record, "w") as json_file:
            json.dump(traj_list, json_file)


if __name__ == "__main__":

    curses.wrapper(main)
