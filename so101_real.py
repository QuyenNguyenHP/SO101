'''Shared helpers for driving the real SO101 from MuJoCo (sim units = radians).

    from so101_real import RealArm
    arm = RealArm(dry=False)          # dry=True: fake arm, nothing moves
    q = arm.read_sim()                # 6 values (rad): 5 joints + gripper, sim units
    arm.send_sim(q)                   # clipped to calibrated range + speed-limited
    arm.close()                       # asks you to hold the arm, then torque off
'''
import json
import os
import numpy as np

PORT = '/dev/ttyACM0'
ROBOT_ID = 'my_follower'
SCENE = os.path.expanduser('~/robot/mujoco_menagerie/robotstudio_so101/scene.xml')
CALIB = os.path.expanduser(
    f'~/.cache/huggingface/lerobot/calibration/robots/so_follower/{ROBOT_ID}.json')
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
RATE = 30                         # commands per second

# Checked with real_twin.py: sim and real agree (by eye, a few degrees)
SIGN = np.array([1, 1, 1, 1, 1])
OFFSET = np.zeros(5)
# Fine joint offsets (deg) measured by table_calib.py: sim angle = real reading - OFFSET
OFFSETS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'joint_offsets.json')
if os.path.exists(OFFSETS_FILE):
    try:
        _o = json.load(open(OFFSETS_FILE))
        OFFSET = np.array([_o.get(j, 0.0) for j in
                           ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']], float)
    except (ValueError, OSError):
        print('WARNING: joint_offsets.json unreadable, using zero offsets')
G_LO, G_HI = -0.17453, 1.74533    # sim gripper range (rad) <-> LeRobot 0..100
# Home pose: recorded with record_home.py (home_pose.json, real servo degrees), else this default
HOME_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'home_pose.json')
HOME_DEFAULT = np.radians([0, -20, 30, 40, 0])
MARGIN = 3.0                      # degrees kept away from the calibrated ends


def configure(port=None, robot_id=None):
    '''Use another serial port / calibration id (call before creating RealArm).'''
    global PORT, ROBOT_ID, CALIB
    if port:
        PORT = port
    if robot_id:
        ROBOT_ID = robot_id
        CALIB = os.path.expanduser(
            f'~/.cache/huggingface/lerobot/calibration/robots/so_follower/{ROBOT_ID}.json')


def sim_to_real(rad):
    return SIGN * np.degrees(rad) + OFFSET


def real_to_sim(deg):
    return np.radians(SIGN * (np.asarray(deg) - OFFSET))


def load_home():
    '''Home pose (sim rad, 5 joints). Stored as REAL servo degrees, so it stays the same physical
    pose if joint_offsets.json changes later.'''
    if os.path.exists(HOME_FILE):
        try:
            h = json.load(open(HOME_FILE))
            return real_to_sim(np.array([h['real_deg'][j] for j in JOINTS], float))
        except (ValueError, OSError, KeyError):
            print('WARNING: home_pose.json unreadable, using the default home')
    return HOME_DEFAULT.copy()


def sim_gripper_to_real(rad):
    return float(np.clip((rad - G_LO) / (G_HI - G_LO) * 100, 0, 100))


def real_gripper_to_sim(g):
    return G_LO + np.clip(g, 0, 100) / 100 * (G_HI - G_LO)


def real_limits_deg():
    '''Calibrated joint range (deg) minus MARGIN: LeRobot puts 0 deg mid-range.'''
    cal = json.load(open(CALIB))
    half = np.array([(cal[j]['range_max'] - cal[j]['range_min']) / 2 * 360 / 4095 for j in JOINTS])
    return -half + MARGIN, half - MARGIN


def real_limits_sim():
    '''Same limits in sim radians (for IK).'''
    lo, hi = real_limits_deg()
    a, b = real_to_sim(lo), real_to_sim(hi)
    return np.minimum(a, b), np.maximum(a, b)


class _FakeRobot:
    def __init__(self):
        self.state = {f'{j}.pos': 0.0 for j in JOINTS} | {'gripper.pos': 50.0}
        self.n = 0

    def connect(self):
        print('[dry] fake arm connected: nothing will move')

    def get_observation(self):
        return dict(self.state)

    def send_action(self, a):
        self.state.update(a)
        self.n += 1

    def disconnect(self):
        print(f'[dry] fake arm disconnected after {self.n} commands')


class RealArm:
    def __init__(self, dry=False, max_step_deg=2.0):
        '''max_step_deg: most a joint may change per command (2 deg * 30 Hz = 60 deg/s).'''
        self.dry = dry
        self.max_step = max_step_deg
        self.lo, self.hi = real_limits_deg()
        if dry:
            self.robot = _FakeRobot()
        else:
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
            self.robot = SO101Follower(SO101FollowerConfig(
                port=PORT, id=ROBOT_ID, use_degrees=True, max_relative_target=10.0))
        self.robot.connect()
        self.last_deg, self.last_grip = self.read_real()

    def read_real(self):
        '''Real arm: 5 joint angles (deg) and gripper (0..100).'''
        obs = self.robot.get_observation()
        return np.array([obs[f'{j}.pos'] for j in JOINTS]), obs['gripper.pos']

    def read_sim(self):
        '''Real arm pose in sim units: 6 values (rad).'''
        deg, g = self.read_real()
        return np.append(real_to_sim(deg), real_gripper_to_sim(g))

    def send_sim(self, q):
        '''Send 6 sim values (rad): clipped to the calibrated range and speed-limited.'''
        target = np.clip(sim_to_real(q[:5]), self.lo, self.hi)
        self.last_deg = self.last_deg + np.clip(target - self.last_deg, -self.max_step, self.max_step)
        g = sim_gripper_to_real(q[5])
        self.last_grip = self.last_grip + np.clip(g - self.last_grip, -2 * self.max_step, 2 * self.max_step)
        action = {f'{j}.pos': float(v) for j, v in zip(JOINTS, self.last_deg)}
        action['gripper.pos'] = float(self.last_grip)
        self.robot.send_action(action)

    def correct_sag(self, q, iters=3, wait=0.4, tol_deg=0.3):
        '''Arm is at (or near) q but sags under gravity: servos stop a degree or two short.
        Re-aim each joint by its remaining error, a few times. Returns the final error (deg, 5).'''
        import time
        target = np.clip(sim_to_real(q[:5]), self.lo, self.hi)
        aim = target.copy()
        err = np.zeros(5)
        for _ in range(iters):
            time.sleep(wait)
            real, _ = self.read_real()
            err = target - real
            if np.abs(err).max() < tol_deg:
                break
            aim = np.clip(aim + np.clip(err, -3, 3), self.lo, self.hi)   # never more than 3 deg per step
            self.last_deg = aim
            action = {f'{j}.pos': float(v) for j, v in zip(JOINTS, aim)}
            action['gripper.pos'] = float(self.last_grip)
            self.robot.send_action(action)
        return err

    def close(self):
        if not self.dry:
            try:
                input('\nSupport the arm with your hand, then press Enter to turn torque off...')
            except (KeyboardInterrupt, EOFError):
                pass
        self.robot.disconnect()


HOME = load_home()
