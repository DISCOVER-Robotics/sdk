import json
import airbot
import time
import os


def replay(json_file):
    with open(json_file) as json_file:
        traj_list = json.load(json_file)

    # 初始化机器人
    urdf_path = airbot.AIRBOT_PLAY_WITH_GRIPPER_URDF

    try:
        robot = airbot.create_agent(urdf_path, "down", "can0", 1.0, "gripper")
    except RuntimeError as e:
        print(e)
        exit(1)

    # 控制机器人
    for traj in traj_list:
        # robot.set_target_joint_q(traj, use_planning=False)
        robot.set_target_joint_mit(
            traj,
            [0, 0, 0, 0, 0, 0],
            [75, 75, 75, 10, 10, 10],
            [1, 1, 1, 0.06, 0.06, 0.06],
        )
        time.sleep(0.001)


if __name__ == "__main__":
    json_file = "examples/trajectory/record.json"
    if os.path.exists(json_file):
        replay(json_file)
