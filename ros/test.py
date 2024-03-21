from scipy.spatial.transform import Rotation as R
import numpy as np
import os

pose = np.array([0., 0., 0.0])
# quat = R.from_rotvec([-np.pi / 4, 0, 0]).as_quat()
quat = np.array([0,0,0,0.
#     # 0.015596234239637852,
#     # 0.4379034638404846,
#     # -0.02391313761472702,
#     # 0.8985685706138611
])
quat = R.from_euler('xyz', [-15, -15., -0], degrees=True).as_quat()
print(quat)
message = f"""position:
  x: {pose[0]}
  y: {pose[1]}
  z: {pose[2]}
orientation:
  x: {quat[0]}
  y: {quat[1]}
  z: {quat[2]}
  w: {quat[3]}"""

os.system(f"rostopic pub -1 /airbot_play/pose_cmd geometry_msgs/Pose \"{message}\"")
