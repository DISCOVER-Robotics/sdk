import airbot

urdf_path = airbot.AIRBOT_PLAY_URDF
bot = airbot.create_agent(urdf_path, "down", "can0", 1.0, "gripper", False, False)

cp = bot.get_current_joint_q()
cv = bot.get_current_joint_v()
ct = bot.get_current_joint_t()
print(cp, cv, ct, sep='\n')
