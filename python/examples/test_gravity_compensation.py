import airbot
import time
import math

urdf_path = airbot.AIRBOT_PLAY_WITH_GRIPPER_URDF
bot = airbot.create_agent(urdf_path, "down", "can0", 1.0, "gripper")

bot.manual_mode()
time.sleep(5)
bot.online_mode()
bot.set_target_joint_q([0, 0, 0, 0, 0, 0], blocking=True)
