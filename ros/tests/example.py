import sys
from scipy.spatial.transform import Rotation
import numpy as np
import os
import threading
import time
angles = [0., 0.1, 0.2, 0.3, 0.2, 0.1, 0., -0.1, -0.2, -0.3, -0.2, -0.1]

idx = 2
for i in range(1):
    # idx += 1
    # idx = idx % len(angles)
    pos = np.array([0.32,0,0.25])
    rot = Rotation.from_rotvec([0 ,0, np.pi * float(sys.argv[1])]).as_quat()
    print(pos, rot)
    cmd = (f"""rostopic pub -1 /airbot_play/pose_cmd geometry_msgs/Pose "position:
    x: {pos[0]}
    y: {pos[1]}
    z: {pos[2]}
orientation:
    x: {rot[0]}
    y: {rot[1]}
    z: {rot[2]}
    w: {rot[3]}"
"""
    )

    try:
        threading.Thread(target=lambda: os.system(cmd)).start()
        time.sleep(2)
        # os.system(cmd)
    except KeyboardInterrupt:
        break