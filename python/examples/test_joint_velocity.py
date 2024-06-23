import airbot
import math
import time

bot = airbot.create_agent("down", "can0", 1.0, "none")

step = 0
bot.set_target_joint_q([0, 0, math.pi / 2, 0, 0, math.pi / 4], blocking=True)
time.sleep(3)
while True:
    step += 0.1
    time.sleep(0.05)
    bot.set_target_joint_v(
        [0, 0, -math.sin(step) * math.pi / 3, 0, 0, -math.sin(step) * math.pi / 3]
    )
