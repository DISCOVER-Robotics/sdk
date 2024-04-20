import airbot
import time
import math

urdf_path = airbot.AIRBOT_PLAY_WITH_GRIPPER_URDF
bot = airbot.create_agent(urdf_path, "down", "can0", 1.0, "gripper", False)

bot.gravity_compensation()
time.sleep(5)
bot.stop_gravity_compensation()
for i in range(20):
    bot.set_target_end(1)
    time.sleep(0.05)
    bot.set_target_end(0)
    time.sleep(0.05)
