'''Sim -> Real: the MuJoCo simulation drives the real SO101 live.

Two modes:
  python Reference/sim_to_real.py                    SLIDER mode: open the "Control" panel on the right of
                                           the viewer and drag the sliders: the real arm follows.
  python Reference/sim_to_real.py npy/wave_plan.npy      PLAN mode: plays a plan (5 or 6 columns, radians,
                                           30 Hz) in the sim and on the real arm together.
  python Reference/sim_to_real.py npy/wave_plan.npy 1.0  ...at full speed (default 0.5)
  add --dry to any of these to test without the arm (prints instead of moving).

Close the viewer (or Ctrl+C) to stop. You'll be asked to hold the arm before torque turns off.
'''
import json
import os
import sys
import time
import numpy as np
import mujoco
import mujoco.viewer

# ---------------- settings ----------------
PORT = '/dev/ttyACM0'
RATE = 30                     # commands per second to the real arm
MAX_STEP_DEG = 3.0            # max change per command = 90 deg/s: real arm can't be yanked
RAMP_SECONDS = 3              # plan mode: time to move from current pose to the plan start
MARGIN = 3.0                  # stay this many degrees inside the calibrated range
SCENE = os.path.expanduser('~/robot/mujoco_menagerie/robotstudio_so101/scene.xml')
CALIB = os.path.expanduser('~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json')
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']

# Same values as real_twin.py / real_play.py (checked: sim and real agree)
SIGN   = np.array([1, 1, 1, 1, 1])
OFFSET = np.zeros(5)
G_LO, G_HI = -0.17453, 1.74533      # sim gripper range (rad) <-> LeRobot 0..100

# ---------------- arguments ----------------
args = [a for a in sys.argv[1:] if a != '--dry']
DRY = '--dry' in sys.argv
PLAN_FILE = args[0] if args else None
SPEED = float(args[1]) if len(args) > 1 else 0.5

# ---------------- conversions ----------------
def sim_to_real(rad):
    return SIGN * np.degrees(rad) + OFFSET

def real_to_sim(deg):
    return np.radians(SIGN * (np.asarray(deg) - OFFSET))

def sim_gripper_to_real(rad):
    return float(np.clip((rad - G_LO) / (G_HI - G_LO) * 100, 0, 100))

def real_gripper_to_sim(g):
    return G_LO + np.clip(g, 0, 100) / 100 * (G_HI - G_LO)

def smooth(s):
    s = np.clip(s, 0.0, 1.0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5

cal = json.load(open(CALIB))
half = np.array([(cal[j]['range_max'] - cal[j]['range_min']) / 2 * 360 / 4095 for j in JOINTS])
REAL_LO, REAL_HI = -half + MARGIN, half - MARGIN

# ---------------- robot (real or fake) ----------------
class FakeRobot:
    '''Stands in for the arm with --dry: remembers commands, prints a few.'''
    def __init__(self):
        self.state = {f'{j}.pos': 0.0 for j in JOINTS} | {'gripper.pos': 50.0}
        self.n = 0
    def connect(self): print('[dry] connected (fake arm)')
    def get_observation(self): return dict(self.state)
    def send_action(self, a):
        self.state.update(a); self.n += 1
        if self.n % RATE == 0:
            print('\n[dry] t=%4.1fs ' % (self.n / RATE) + ' '.join(f'{v:6.1f}' for v in a.values()))
    def disconnect(self): print(f'[dry] disconnected after {self.n} commands')

if DRY:
    robot = FakeRobot()
else:
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    robot = SO101Follower(SO101FollowerConfig(
        port=PORT, id='my_follower', use_degrees=True, max_relative_target=10.0))
robot.connect()

def read_real():
    obs = robot.get_observation()
    return np.array([obs[f'{j}.pos'] for j in JOINTS]), obs['gripper.pos']

last_deg, last_grip = read_real()

def send(ctrl):
    '''Send sim controls (6 values, rad) to the real arm: clipped and speed-limited.'''
    global last_deg, last_grip
    target = np.clip(sim_to_real(ctrl[:5]), REAL_LO, REAL_HI)
    last_deg = last_deg + np.clip(target - last_deg, -MAX_STEP_DEG, MAX_STEP_DEG)
    g = sim_gripper_to_real(ctrl[5])
    last_grip = last_grip + np.clip(g - last_grip, -MAX_STEP_DEG, MAX_STEP_DEG)
    action = {f'{j}.pos': float(v) for j, v in zip(JOINTS, last_deg)}
    action['gripper.pos'] = float(last_grip)
    robot.send_action(action)

# ---------------- simulation, starting at the real pose ----------------
model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)
lo, hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
start = np.append(real_to_sim(last_deg), real_gripper_to_sim(last_grip))
data.qpos[:6] = start
data.ctrl[:] = np.clip(start, lo, hi)
mujoco.mj_forward(model, data)

plan = None
if PLAN_FILE:
    plan = np.load(PLAN_FILE)
    assert plan.ndim == 2 and plan.shape[1] in (5, 6), f'expected (N, 5) or (N, 6), got {plan.shape}'
    if plan.shape[1] == 5:                       # no gripper column: keep the gripper where it is
        plan = np.hstack([plan, np.full((len(plan), 1), start[5])])
    plan = np.clip(plan, lo, hi)
    ramp = [start + smooth(k / (RAMP_SECONDS * RATE)) * (plan[0] - start)
            for k in range(RAMP_SECONDS * RATE + 1)]
    print(f'PLAN mode: {len(plan)} commands at SPEED={SPEED} ({len(plan) / RATE / SPEED:.1f} s), '
          f'after a {RAMP_SECONDS} s ramp to the start pose')
else:
    print('SLIDER mode: in the viewer, open the "Control" panel (right side) and drag the sliders.')

# ---------------- main loop ----------------
dt = 1 / RATE
steps_per_cmd = max(1, round(dt / model.opt.timestep))
k = 0                                            # index into ramp + plan
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        t_next = time.perf_counter()
        while viewer.is_running():
            if plan is not None:
                if k < len(ramp):                          # ramp at normal rate
                    data.ctrl[:] = ramp[k]
                    k += 1
                else:                                      # plan at chosen speed
                    i = int((k - len(ramp)) * SPEED)
                    if i >= len(plan):
                        print('\nPlan finished. Close the viewer to stop.')
                        plan = None                        # hand over to the sliders
                    else:
                        data.ctrl[:] = plan[i]
                        k += 1
            data.ctrl[:] = np.clip(data.ctrl, lo, hi)
            send(data.ctrl.copy())                         # sim command -> real arm
            for _ in range(steps_per_cmd):                 # advance the sim in real time
                mujoco.mj_step(model, data)
            viewer.sync()                                  # also picks up slider changes

            real_deg, _ = read_real()
            err = np.abs(sim_to_real(data.qpos[:5]) - real_deg).max()
            sys.stdout.write(f'\rsim vs real: max joint difference {err:5.1f} deg   ')
            sys.stdout.flush()

            t_next += dt
            time.sleep(max(0.0, t_next - time.perf_counter()))
except KeyboardInterrupt:
    print('\nStopped by user.')
finally:
    print()
    if not DRY:
        try:
            input('Support the arm with your hand, then press Enter to turn torque off...')
        except (KeyboardInterrupt, EOFError):
            pass
    robot.disconnect()
