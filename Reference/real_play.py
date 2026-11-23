'''Play a MuJoCo joint plan (radians, 30 Hz) on the real SO101.

A plan has 5 columns (arm joints) or 6 (arm joints + gripper).
Run:  python Reference/real_play.py                     (plays square_plan.npy at half speed)
      python Reference/real_play.py npy/wave_plan.npy       (wave hello, with gripper)
      python Reference/real_play.py npy/square_plan.npy 1.0 (full speed, only after it works slowly)
Ctrl+C stops at any time.
'''
import json
import os
import sys
import time
import numpy as np
import mujoco
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = '/dev/ttyACM0'
PLAN_FILE = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'npy', 'square_plan.npy')
SPEED = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5   # 0.5 = half speed
RATE = 30            # commands per second, same as the sim
RAMP_SECONDS = 3     # time to move from the current pose to the plan's first pose
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
SCENE = os.path.expanduser('~/robot/mujoco_menagerie/robotstudio_so101/scene.xml')

# Same values as real_twin.py (checked: sim and real agree)
SIGN   = np.array([1, 1, 1, 1, 1])
OFFSET = np.zeros(5)

def smooth(s):
    s = np.clip(s, 0.0, 1.0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5

def sim_to_real(rad):
    '''Sim angle (rad) -> real angle (deg).'''
    return SIGN * np.degrees(rad) + OFFSET

def real_to_sim(deg):
    return np.radians(SIGN * (np.asarray(deg) - OFFSET))

G_LO, G_HI = -0.17453, 1.74533      # sim gripper range (rad)

def sim_gripper_to_real(rad):
    '''Sim gripper (rad) -> LeRobot gripper (0..100).'''
    return float(np.clip((rad - G_LO) / (G_HI - G_LO) * 100, 0, 100))

def real_gripper_to_sim(g):
    return G_LO + np.clip(g, 0, 100) / 100 * (G_HI - G_LO)

# Real joint limits (deg) from the calibration file: LeRobot puts 0 deg at the middle of
# each calibrated range and does NOT clip degree commands, so we do it here.
CALIB = os.path.expanduser('~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json')
MARGIN = 3.0
cal = json.load(open(CALIB))
half = np.array([(cal[j]['range_max'] - cal[j]['range_min']) / 2 * 360 / 4095 for j in JOINTS])
REAL_LO, REAL_HI = -half + MARGIN, half - MARGIN

def send(robot, q_rad):
    deg = np.clip(sim_to_real(q_rad[:5]), REAL_LO, REAL_HI)
    action = {f'{j}.pos': float(v) for j, v in zip(JOINTS, deg)}
    if len(q_rad) > 5:
        action['gripper.pos'] = sim_gripper_to_real(q_rad[5])
    robot.send_action(action)

# --- Load and check the plan in sim units ---
model = mujoco.MjModel.from_xml_path(SCENE)
plan = np.load(PLAN_FILE)
assert plan.ndim == 2 and plan.shape[1] in (5, 6), f'expected (N, 5) or (N, 6), got {plan.shape}'
cols = plan.shape[1]
lo, hi = model.jnt_range[:cols, 0], model.jnt_range[:cols, 1]
if np.any(plan < lo) or np.any(plan > hi):
    print('WARNING: plan goes outside sim joint limits, clipping')
plan = np.clip(plan, lo, hi)
deg = sim_to_real(plan[:, :5])
if np.any(deg < REAL_LO) or np.any(deg > REAL_HI):
    bad = [j for i, j in enumerate(JOINTS) if np.any(deg[:, i] < REAL_LO[i]) or np.any(deg[:, i] > REAL_HI[i])]
    print('WARNING: plan goes past the calibrated range of', bad, '- those commands are clipped')
print(f'{len(plan)} commands ({"with" if cols == 6 else "no"} gripper), '
      f'{len(plan) / RATE / SPEED:.1f} s at SPEED={SPEED}')

# max_relative_target: no joint may jump more than 10 deg in one command (safety net)
robot = SO101Follower(SO101FollowerConfig(
    port=PORT, id='my_follower', use_degrees=True, max_relative_target=10.0))
robot.connect()

try:
    # 1. Smoothly go from wherever the arm is now to the plan's first pose
    obs = robot.get_observation()
    now = real_to_sim([obs[f'{j}.pos'] for j in JOINTS])
    if cols == 6:
        now = np.append(now, real_gripper_to_sim(obs['gripper.pos']))
    print('Moving to start pose...')
    n = int(RAMP_SECONDS * RATE)
    for k in range(n + 1):
        send(robot, now + smooth(k / n) * (plan[0] - now))
        time.sleep(1 / RATE)

    input('At start pose. Press Enter to run the plan (Ctrl+C to stop)...')

    # 2. Play the plan at a fixed rate
    dt = 1 / (RATE * SPEED)
    t_next = time.perf_counter()
    for q in plan:
        send(robot, q)
        t_next += dt
        time.sleep(max(0.0, t_next - time.perf_counter()))
    time.sleep(0.5)
    obs = robot.get_observation()
    err = sim_to_real(plan[-1][:5]) - np.array([obs[f'{j}.pos'] for j in JOINTS])
    print('Done. Final joint error (deg):', np.round(err, 1))
    input('Support the arm with your hand, then press Enter to turn torque off...')
except KeyboardInterrupt:
    print('\nStopped by user.')
finally:
    robot.disconnect()      # torque off: the arm goes limp
