import airbot
import math

bot = airbot.create_agent(end_mode="none")

joint_pos = bot.get_current_joint_q()
step = 0
while True:
    joint_pos[0] = math.sin(step) * math.pi / 2
    step += math.pi / 2
    bot.set_target_joint_q(joint_pos, blocking=True)
