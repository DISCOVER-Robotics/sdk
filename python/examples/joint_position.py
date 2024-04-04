import airbot
import time
import math

urdf_path = airbot.AIRBOT_PLAY_URDF
bot = airbot.create_agent(urdf_path, "down", "can0", 1.0, "gripper", False, False)

joint_pos = bot.get_current_joint_q()
step = 0
while True:
    joint_pos[0] = math.sin(step) * math.pi / 2
    step += math.pi / 3
    bot.set_target_joint_q(joint_pos)
    time.sleep(1.5)
