import json
import airbot
import time
import os
import math


def replay(json_file):
    with open(json_file) as json_file:
        traj_list = json.load(json_file)

    # 初始化机器人
    urdf_path = airbot.AIRBOT_PLAY_WITH_GRIPPER_URDF

    try:
        robot = airbot.create_agent("down", "can0", math.pi * 3, "none")
    except RuntimeError as e:
        print(e)
        exit(1)

    # 控制机器人
    for traj in traj_list:
        robot.set_target_joint_q(traj, use_planning=False)
        # robot.set_target_joint_mit(
        #     traj,
        #     [0, 0, 0, 0, 0, 0],
        #     [120, 120, 120, 30, 30, 30],
        #     [1.2, 1.2, 1.2, 0.15, 0.15, 0.15],
        # )
        time.sleep(0.001)


if __name__ == "__main__":
    json_file = "examples/trajectory/record.json"
    if os.path.exists(json_file):
        replay(json_file)
