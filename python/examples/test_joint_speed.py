import airbot
import math
import time

urdf_path = airbot.AIRBOT_PLAY_WITH_GRIPPER_URDF
bot = airbot.create_agent(urdf_path, "down", "can0", 1.0, "gripper")

step = 0
bot.set_target_joint_q([0, 0, math.pi / 2, 0, 0, math.pi / 4], blocking=True)
time.sleep(3)
while True:
    step += 0.1
    time.sleep(0.05)
    bot.set_target_joint_v(
        [0, 0, -math.sin(step) * math.pi / 3, 0, 0, -math.sin(step) * math.pi / 3]
    )
