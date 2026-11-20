# Deploying to a Real SO101 Arm

This guide takes the motions you built in MuJoCo (`~/robot/mujoco_menagerie`, e.g. `draw_square.py`, `wave_hello.py`) and runs them on a **real SO101 follower arm**. It uses [LeRobot](https://github.com/huggingface/lerobot), Hugging Face's library for the SO100/SO101 arms, to talk to the Feetech servos.

The workflow:

```
MuJoCo plan (radians)  ──►  convert (deg, sign, offset)  ──►  LeRobot  ──►  STS3215 servos
        ▲                                                                       │
        └──────────────  digital twin: read real angles back  ◄─────────────────┘
```

> [!WARNING]
> A real arm can pinch fingers, knock things over, and burn out servos. Clear the table, clamp the base down, keep one hand on the **power switch or plug**, and always test new motions **slowly** first.

---

## 0. What you need

| Item | Notes |
|---|---|
| SO101 **follower** arm, assembled | 6 × Feetech STS3215 servos (IDs 1–6) |
| Bus servo driver board | Waveshare-style board that ships with the kit, USB-C to the PC |
| Power supply | **5 V** for the 7.4 V servos, **12 V** for the 12 V servos. Check the label on your servos; the wrong voltage destroys them |
| Linux PC | This machine (Python 3.10+) |
| *(optional)* SO101 **leader** arm | Only needed for teleoperation (step 5) |

Joint names are the same in MuJoCo and LeRobot:

| # | MuJoCo joint / actuator | LeRobot key | Servo ID | Sim range (rad) |
|---|---|---|---|---|
| 0 | `shoulder_pan`  | `shoulder_pan.pos`  | 1 | −1.92 … 1.92 |
| 1 | `shoulder_lift` | `shoulder_lift.pos` | 2 | −1.75 … 1.75 |
| 2 | `elbow_flex`    | `elbow_flex.pos`    | 3 | −1.69 … 1.69 |
| 3 | `wrist_flex`    | `wrist_flex.pos`    | 4 | −1.66 … 1.66 |
| 4 | `wrist_roll`    | `wrist_roll.pos`    | 5 | −2.74 … 2.84 |
| 5 | `gripper`       | `gripper.pos`       | 6 | −0.17 … 1.75 |

---

## 1. Install LeRobot

Use a separate environment so LeRobot's pinned dependencies don't break your MuJoCo setup.

```bash
conda create -y -n lerobot python=3.10
```

```bash
conda activate lerobot
```

```bash
conda install -y ffmpeg -c conda-forge
```

```bash
git clone https://github.com/huggingface/lerobot.git ~/robot/lerobot
```

```bash
cd ~/robot/lerobot && pip install -e ".[feetech]"
```

Then install MuJoCo into the **same** env, so one script can run both the sim and the real arm:

```bash
pip install mujoco
```

> [!NOTE]
> LeRobot changes quickly. The commands below (`lerobot-find-port`, `lerobot-calibrate`, …) are the current CLI names. If your version doesn't have them, the older form is `python -m lerobot.find_port`, `python -m lerobot.calibrate`, and so on. Check `~/robot/lerobot/docs/source/so101.mdx` for your version.

---

## 2. Connect and find the USB port

1. Plug the power supply into the driver board, then USB into the PC.
2. Run the finder, unplug the USB cable when it asks, and press Enter:

```bash
lerobot-find-port
```

It prints something like `/dev/ttyACM0`. Use that wherever this guide says `/dev/ttyACM0`.

3. Give your user access to serial ports (one time; log out and back in afterwards):

```bash
sudo usermod -aG dialout $USER
```

For a quick test without logging out, `sudo chmod 666 /dev/ttyACM0` works until the next replug.

---

## 3. Set motor IDs (only if the servos are new)

New STS3215 servos all have ID 1. LeRobot assigns IDs 1–6 and the baud rate one servo at a time. **Skip this step if your kit came pre-configured.**

```bash
lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0
```

Follow the prompts: connect **only** the servo it names (gripper first, then wrist_roll, and so on down to shoulder_pan), press Enter, and repeat.

---

## 4. Calibrate

Calibration teaches LeRobot where each joint's middle and limits are. **It also decides how the real zero pose lines up with the MuJoCo zero pose**, so do it carefully.

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower
```

1. When asked, move the arm by hand to the **middle of every joint's range** (the pose in the LeRobot SO101 docs: arm upright and bent, gripper half open) and press Enter.
2. Then move **every joint slowly through its full range**, end to end, and press Enter.

The result is saved to `~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json`. Always use the same `--robot.id` (`my_follower`) afterwards so the file is found.

> [!TIP]
> The MuJoCo model in `robotstudio_so101/` comes from `so101_new_calib.xml`, which was made to match **this** LeRobot calibration style. So after calibration, 0 rad in sim should be close to 0° on the real arm. Step 6 checks whether it actually is.

---

## 5. (Optional) Teleoperate with a leader arm

A quick check that the hardware and calibration work. Calibrate the leader once with `--teleop.type=so101_leader --teleop.id=my_leader` (same command as step 4), then:

```bash
lerobot-teleoperate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=my_leader
```

Moving the leader should move the follower the same way.

---

## 6. Digital twin: check that sim and real agree

Before sending any motion, find out how the real angles relate to the sim angles. This script turns **torque off** (you move the arm by hand) and shows the real pose live in the MuJoCo viewer.

Save it as `~/robot/mujoco_menagerie/real_twin.py` and run it from that folder (`robot_utils.py` loads `robotstudio_so101/scene.xml` by relative path):

```python
'''Digital twin: move the real arm by hand and watch MuJoCo follow it.'''
import time
import numpy as np
import mujoco
import mujoco.viewer
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from robot_utils import load

PORT = '/dev/ttyACM0'
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']

# Fill these in from what you observe (see below)
SIGN   = np.array([1, 1, 1, 1, 1])        # +1 or -1 per joint
OFFSET = np.zeros(5)                      # degrees, added after the sign

def real_to_sim(deg):
    '''Real angle (deg) -> sim angle (rad).'''
    return np.radians(SIGN * (np.asarray(deg) - OFFSET))

robot = SO101Follower(SO101FollowerConfig(port=PORT, id='my_follower', use_degrees=True))
robot.connect()
robot.bus.disable_torque()                # arm goes limp: move it by hand

model, data, ee = load()
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            obs = robot.get_observation()
            deg = np.array([obs[f'{j}.pos'] for j in JOINTS])
            data.qpos[:5] = real_to_sim(deg)
            mujoco.mj_kinematics(model, data)
            viewer.sync()
            print('real deg:', np.round(deg, 1), '| gripper 0-100:', round(obs['gripper.pos'], 1), end='\r')
            time.sleep(1 / 30)
finally:
    robot.disconnect()
```

**How to fill in `SIGN` and `OFFSET`:**

1. Run the script and move **one joint at a time** by hand.
2. If the sim joint turns the **opposite** way, set that joint's `SIGN` to `-1`.
3. Put the real arm in the sim's zero pose (look at the sim with all joints at 0). Read the printed degrees. Those numbers go into `OFFSET`. If they are all within about ±5°, leave `OFFSET` at zero.
4. Repeat until the arm in the viewer matches the real arm in every pose you try.

**Gripper:** in LeRobot the gripper is always **0–100** (0 = closed, 100 = open; confirm on your arm). The sim gripper goes from −0.17 rad to 1.75 rad, so map it linearly (see `sim_gripper_to_real` below).

---

## 7. Play a MuJoCo plan on the real arm

### 7a. Save the plan from the sim

The scripts compute a list of joint commands called `plan` and then play it in the viewer. Save that list so the real arm can play the same thing. In `draw_square.py`, right after the line `print('Lập kế hoạch xong', ...)`, add:

```python
np.save('square_plan.npy', np.array(plan))   # shape (N, 5), radians, 30 commands per second
```

Run `draw_square.py` once in the sim, check that `ok` is `True` and that the motion looks right, then close the viewer.

> [!IMPORTANT]
> `SIZE = 0.2` (a 40 cm square) is near or beyond the real arm's reach. Start with **`SIZE = 0.03`** on the real arm.

### 7b. Send it to the robot

Save as `~/robot/mujoco_menagerie/real_play.py`:

```python
'''Play a MuJoCo joint plan (radians, 30 Hz) on the real SO101.'''
import sys
import time
import numpy as np
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from robot_utils import load, smooth

PORT = '/dev/ttyACM0'
PLAN_FILE = sys.argv[1] if len(sys.argv) > 1 else 'square_plan.npy'
RATE = 30            # commands per second, same as the sim
SPEED = 0.5          # 0.5 = half speed. Raise to 1.0 only after it works
RAMP_SECONDS = 3     # time to move from the current pose to the plan's first pose
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']

# Copy these from real_twin.py once they are correct
SIGN   = np.array([1, 1, 1, 1, 1])
OFFSET = np.zeros(5)

def sim_to_real(rad):
    '''Sim angle (rad) -> real angle (deg). Inverse of real_to_sim.'''
    return SIGN * np.degrees(rad) + OFFSET

def sim_gripper_to_real(rad):
    '''Sim gripper (-0.17..1.75 rad) -> LeRobot gripper (0..100).'''
    return float(np.clip((rad + 0.17453) / (1.74533 + 0.17453) * 100, 0, 100))

def send(robot, q_rad, gripper=None):
    action = {f'{j}.pos': float(v) for j, v in zip(JOINTS, sim_to_real(q_rad))}
    if gripper is not None:
        action['gripper.pos'] = gripper
    robot.send_action(action)

# --- Load and check the plan in sim units ---
model, _, _ = load()
lo, hi = model.jnt_range[:5, 0], model.jnt_range[:5, 1]
plan = np.load(PLAN_FILE)
assert plan.ndim == 2 and plan.shape[1] == 5, f'expected (N, 5), got {plan.shape}'
if np.any(plan < lo) or np.any(plan > hi):
    print('WARNING: plan goes outside joint limits, clipping')
plan = np.clip(plan, lo, hi)
print(f'{len(plan)} commands, {len(plan) / RATE / SPEED:.1f} s at SPEED={SPEED}')

# max_relative_target limits how far (deg) any joint may jump in one command: a safety net
robot = SO101Follower(SO101FollowerConfig(
    port=PORT, id='my_follower', use_degrees=True, max_relative_target=10.0))
robot.connect()

try:
    # 1. Smoothly go from wherever the arm is now to the first pose of the plan
    obs = robot.get_observation()
    now_deg = np.array([obs[f'{j}.pos'] for j in JOINTS])
    now_rad = np.radians(SIGN * (now_deg - OFFSET))
    n = int(RAMP_SECONDS * RATE)
    for k in range(n + 1):
        send(robot, now_rad + smooth(k / n) * (plan[0] - now_rad))
        time.sleep(1 / RATE)

    input('At start pose. Press Enter to run the plan (Ctrl+C to stop at any time)...')

    # 2. Play the plan at a fixed rate
    dt = 1 / (RATE * SPEED)
    t_next = time.perf_counter()
    for q in plan:
        send(robot, q)
        t_next += dt
        time.sleep(max(0.0, t_next - time.perf_counter()))
    print('Done.')
    time.sleep(1.0)
except KeyboardInterrupt:
    print('\nStopped by user.')
finally:
    robot.disconnect()      # turns torque off: support the arm so it doesn't drop!
```

Run it:

```bash
cd ~/robot/mujoco_menagerie && python real_play.py square_plan.npy
```

> [!CAUTION]
> `robot.disconnect()` turns torque **off**, so the arm falls under gravity. Hold it, or rest it in a safe pose, before the script ends.

### 7c. Motions written as live loops (e.g. `wave_hello.py`)

`wave_hello.py` doesn't build a plan first. It computes `u` in a loop. To run it on the real arm, keep the same math and replace the MuJoCo lines:

| Sim (`wave_hello.py`) | Real |
|---|---|
| `data.ctrl[:] = np.clip(u, lo, hi)` | `u = np.clip(u, lo, hi)` then `send(robot, u[:5], sim_gripper_to_real(u[5]))` |
| `mujoco.mj_step(...)`, `viewer.sync()` | *(remove)* |
| `time.sleep(model.opt.timestep)` | `time.sleep(1 / 30)` (servos don't need 500 Hz) |

Also add the 3-second ramp from 7b before the loop, because the first `u` may be far from where the arm is.

---

## 8. First-run checklist

Go through this the **first time** you run any new motion:

- [ ] Plan checked in MuJoCo, `ok == True`, no collisions with the table
- [ ] `SIGN` / `OFFSET` confirmed with `real_twin.py`
- [ ] `SPEED = 0.5` or lower, small motion (`SIZE = 0.03`)
- [ ] Nothing within reach of the arm, base clamped
- [ ] Hand on the power plug; `Ctrl+C` ready
- [ ] Watched the first run end to end before raising `SPEED`

---

## 9. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `Permission denied: '/dev/ttyACM0'` | Step 2: `dialout` group, then log out and in |
| `Missing motor IDs` / `no status packet` | Power is off, a cable is loose, or IDs aren't set (step 3). Check that the servo LEDs flash at power-on |
| Port is `/dev/ttyACM1` after replugging | Run `lerobot-find-port` again, or make a udev rule for a fixed name |
| Arm moves the **wrong way** on one joint | That joint's `SIGN` is wrong (step 6) |
| Real pose is shifted from the sim by a fixed angle | `OFFSET` (step 6), or redo calibration (step 4) |
| Arm jerks or stops partway, LeRobot logs a clipped goal | `max_relative_target` is stopping big jumps: lower `SPEED` or add more `STEPS` in the sim |
| Servo hot, buzzing, or arm sags under load | Holding a pose against gravity or against the table. Reduce the motion, check the supply voltage and current |
| `ImportError: lerobot.robots.so_follower` | Your LeRobot version uses a different module path. Search: `grep -rn "class SO101Follower" ~/robot/lerobot/src` |
| Calibration file not found | Use the same `--robot.id` / `id=` everywhere (`my_follower`) |

---

## Reference

- LeRobot SO101 guide: `~/robot/lerobot/docs/source/so101.mdx` (also on huggingface.co/docs/lerobot/so101)
- SO-ARM100/101 hardware repo: https://github.com/TheRobotStudio/SO-ARM100
- MuJoCo model used in sim: `~/robot/mujoco_menagerie/robotstudio_so101/so101.xml`
