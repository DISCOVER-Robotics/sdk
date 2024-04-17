import time
import airbot

motor = airbot.create_motor(motor_id=1, interface="can0", type="OD")

while True:
    motor.MotorPosModeCmd(1.5, 1.0)  # motor_angle_pose, motor_angle_speed
    # motor.MotorSpdModeCmd(0.5) # motor_angle_speed
    # motor.MotorMitModeCmd(0, 0, 0, 0, 0.5) # f_p, f_v, f_kp, f_kd, f_t
    time.sleep(0.25)
