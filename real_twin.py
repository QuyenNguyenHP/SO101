'''Digital twin: move the real SO101 by hand and watch the MuJoCo model follow it.

Torque is OFF the whole time: the arm is limp, so support it with your hand.
Run from anywhere:  python ~/robot/SO101/real_twin.py
'''
import os
import sys
import time
import numpy as np
import mujoco
import mujoco.viewer
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

PORT = '/dev/ttyACM0'
MENAGERIE = os.path.expanduser('~/robot/mujoco_menagerie')
SCENE = os.path.join(MENAGERIE, 'robotstudio_so101', 'scene.xml')
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']

# ---- Fill these in from what you see (step 6 of DEPLOY_REAL_SO101.md) ----
SIGN   = np.array([1, 1, 1, 1, 1])        # +1 or -1 per joint: -1 if the sim turns the opposite way
OFFSET = np.zeros(5)                      # degrees: real reading when the arm is in the sim's zero pose

# Sim gripper range (rad) and LeRobot gripper range (0..100)
G_LO, G_HI = -0.17453, 1.74533

def real_to_sim(deg):
    '''Real angle (deg) -> sim angle (rad).'''
    return np.radians(SIGN * (np.asarray(deg) - OFFSET))

def real_gripper_to_sim(g):
    '''LeRobot gripper 0..100 -> sim gripper (rad).'''
    return G_LO + np.clip(g, 0, 100) / 100 * (G_HI - G_LO)

robot = SO101Follower(SO101FollowerConfig(port=PORT, id='my_follower', use_degrees=True))
robot.connect()                           # runs calibration first if it was never done
robot.bus.disable_torque()                # arm goes limp: move it by hand

model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)
print('Move one joint at a time and compare with the viewer. Close the viewer to stop.')
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            obs = robot.get_observation()
            deg = np.array([obs[f'{j}.pos'] for j in JOINTS])
            data.qpos[:5] = real_to_sim(deg)
            data.qpos[5] = real_gripper_to_sim(obs['gripper.pos'])
            mujoco.mj_kinematics(model, data)
            viewer.sync()
            text = '  '.join(f'{j[:8]}:{d:7.1f}' for j, d in zip(JOINTS, deg))
            sys.stdout.write(f'\r{text}  grip:{obs["gripper.pos"]:5.1f}   ')
            sys.stdout.flush()
            time.sleep(1 / 30)
finally:
    print()
    robot.disconnect()
