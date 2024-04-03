import airbot
import time
import math

urdf_path = airbot.AIRBOT_PLAY_WITH_GRIPPER_URDF
bot = airbot.create_agent(urdf_path, "down", "can0", 1.0, "gripper", False, False)

bot.gravity_compensation()
time.sleep(5)
bot.stop_gravity_compensation()
time.sleep(5)
