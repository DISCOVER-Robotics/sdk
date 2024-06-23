import airbot
import time
import math

bot = airbot.create_agent("down", "can0", 1.0, "none")

bot.manual_mode()
time.sleep(5)
bot.online_mode()
bot.set_target_joint_q([0, 0, 0, 0, 0, 0], blocking=True)
