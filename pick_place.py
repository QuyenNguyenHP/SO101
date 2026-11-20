'''Pick an object up and put it somewhere else.

Coordinates are in mm, measured at the gripper tip, robot frame (same numbers as the status
line of keyboard_control.py):  x forward, y left, z up from the table.

  python pick_place.py 250 0  200 120              sim only: viewer with a 4x4x6 cm box
  python pick_place.py 250 0  200 120 --real       real arm (sim shown alongside)
  python pick_place.py 250 0  200 120 --real --dry test the real-arm path without the arm
Options:
  --height 60     object height in mm (default 60, the sim box)
  --width 40      object width between the jaws in mm (default 40)
  --yaw 0         how much the object is turned on the table (deg); the wrist turns to match
  --speed 0.5     playback speed on the real arm (default 0.5)

How to find the numbers on the real table: run keyboard_control.py, move the tip right above the
object (pointing down), read x and y from the status line. Same for the place spot.
'''
import argparse
import os
import sys
import time
import numpy as np
import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from so101_real import RealArm, RATE, real_limits_sim               # noqa: E402
from keyboard_control import ik, tcp, pitch_of, HOME                 # noqa: E402

BOX_SCENE = os.path.expanduser('~/robot/mujoco_menagerie/robotstudio_so101/scene_box.xml')
LIFT = 0.08            # m above the grasp height when travelling
SPEED_MS = 0.08        # tip speed for travel moves (m/s, at speed 1.0)
PITCH = 80.0           # gripper pointing almost straight down (at exactly 90 IK loses the tilt)
GRASP_SEED = np.radians([0, 0, 30, 65, 0])   # a known good "reaching down" pose for IK to start from
GRIP_CLOSED = -0.17    # fully closed command: squeezes the object (force-limited)
CLEAR = 0.008          # m: fixed jaw comes down this far from the object's side
PLACE_ABOVE = 0.010    # m: release the object this much above where it was grasped
def place_shift(width):
    '''m: when opening, the moving jaw drags a WIDE object (40 mm) ~15 mm its way; objects
    30 mm or narrower are not dragged (measured in sim). Release that much toward the fixed jaw.'''
    return 0.015 * float(np.clip((width - 0.030) / 0.010, 0, 1))


def grip_open_for(width_m):
    '''Gripper angle (rad) that opens the fingertips ~3 cm wider than the object.'''
    gap = np.array([2.8, 14.8, 29.9, 44.8, 59.2, 73.1, 86.3, 98.6]) / 1000   # measured in sim
    ang = np.array([-0.17, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2])
    return float(np.interp(width_m + 0.03, gap, ang))


def jaw_dir(model, d, q):
    '''Direction (deg, in the table plane) in which the jaws close, for joint angles q.
    Measured with the gripper open: when closed the fingertips touch and the direction is noise.'''
    d.qpos[:5] = q[:5]
    d.qpos[5] = 0.6
    mujoco.mj_kinematics(model, d)
    v = d.geom_xpos[model.geom('moving_jaw_sph_tip2').id] - d.geom_xpos[model.geom('fixed_jaw_sph_tip2').id]
    return np.degrees(np.arctan2(v[1], v[0]))


def roll_for(model, d, q, yaw_deg, lo, hi):
    '''Wrist roll (rad) that makes the jaws close across the object (object turned yaw_deg).
    The jaws work the same turned 180 deg, so pick the option nearest the wrist's middle.'''
    best = None
    for k in (-1, 0, 1):
        q2 = q.copy()
        for _ in range(3):                      # roll and jaw direction move 1:1, refine twice
            diff = (yaw_deg + 180 * k - jaw_dir(model, d, q2) + 180) % 360 - 180
            q2[4] += np.radians(diff)
        if lo[4] <= q2[4] <= hi[4] and (best is None or abs(q2[4]) < abs(best)):
            best = q2[4]
    return best if best is not None else q[4]


def smooth(s):
    s = np.clip(s, 0.0, 1.0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def build_plan(model, pick, place, height, width, yaw_deg, q_start):
    '''Returns (plan N x 6 rad at 30 Hz, list of (frame index, step name)).'''
    d = mujoco.MjData(model)
    lo, hi = real_limits_sim()
    lo = np.maximum(model.actuator_ctrlrange[:5, 0], lo)
    hi = np.minimum(model.actuator_ctrlrange[:5, 1], hi)
    z_grasp = max(height * 0.5, 0.015)
    g_open = grip_open_for(width)

    plan, labels = [], []
    q = np.array(q_start, dtype=float)

    def go(p_end, grip, name, seconds=None, tol=0.012):
        '''Straight line for the tip. tol: allowed IK error (tight only near the object).'''
        nonlocal q
        p_start, _ = tcp(model, d, q)
        dist = np.linalg.norm(p_end - p_start)
        T = seconds or max(1.0, dist / SPEED_MS)
        n = int(T * RATE)
        g0 = q[5]
        labels.append((len(plan), name))
        for k in range(1, n + 1):
            s = smooth(k / n)
            q, err = ik(model, d, q, p_start + s * (p_end - p_start), PITCH, True, lo, hi)
            if err > tol:
                raise SystemExit(f'Cannot reach {np.round(p_end * 1000)} mm during "{name}" '
                                 f'(off by {err * 1000:.0f} mm). Move the object closer to the robot.')
            q[5] = g0 + s * (grip - g0)
            plan.append(q.copy())

    def wait(grip, name, seconds):
        nonlocal q
        labels.append((len(plan), name))
        g0, n = q[5], int(seconds * RATE)
        for k in range(1, n + 1):
            q[5] = g0 + smooth(k / n) * (grip - g0)
            plan.append(q.copy())

    def blend(q_end, name, seconds):
        '''Joint-space move (used where the exact tip path doesn't matter).'''
        nonlocal q
        labels.append((len(plan), name))
        q0, n = q.copy(), max(1, int(seconds * RATE))
        for k in range(1, n + 1):
            plan.append(q0 + smooth(k / n) * (q_end - q0))
        q = plan[-1].copy()

    def low_pose(center, roll=None, clear=CLEAR):
        '''Joint angles with the tip at the object, offset so the FIXED jaw clears the object
        side by `clear` (the moving jaw then pushes the object against it when closing, which
        leaves the object centred between the fingertips: so placing uses clear=0).'''
        seed = np.append(GRASP_SEED, q[5])
        seed[0] = -np.arctan2(center[1], center[0])   # face the spot (+pan turns toward -y)
        p = np.array([*center, z_grasp])
        qg, err = ik(model, d, seed, p, PITCH, True, lo, hi, iters=500)
        qg[4] = roll_for(model, d, qg, yaw_deg, lo, hi) if roll is None else roll
        for _ in range(2):                            # offset depends on the jaw direction
            a = np.radians(jaw_dir(model, d, qg))
            u = np.array([np.cos(a), np.sin(a), 0])   # fixed jaw -> moving jaw
            qg, err = ik(model, d, qg, p - clear * u, PITCH, True, lo, hi, iters=300)
        if err > 0.004:
            raise SystemExit(f'Cannot reach {np.round(center * 1000)} mm '
                             f'(off by {err * 1000:.0f} mm). Move it closer to the robot.')
        return qg

    def straight_up(q_low, grip):
        '''Tip goes straight up LIFT from q_low. Played backwards it is a clean descent that
        is sure to end exactly at q_low.'''
        p0, _ = tcp(model, d, q_low)
        n, qq, path = int(max(1.0, LIFT / SPEED_MS * 1.5) * RATE), q_low.copy(), []
        for k in range(1, n + 1):
            qq, err = ik(model, d, qq, p0 + smooth(k / n) * np.array([0, 0, LIFT]), PITCH, True, lo, hi)
            if err > 0.010:
                raise SystemExit(f'Cannot move straight up from {np.round(p0 * 1000)} mm '
                                 f'(off by {err * 1000:.0f} mm). Try a spot a little further away.')
            qq[5] = grip
            path.append(qq.copy())
        return path

    # ---- pick ----
    q_pick = low_pose(pick)
    q_pick[5] = g_open
    up_pick = straight_up(q_pick, g_open)
    blend(up_pick[-1], 'move above object', 3.0)
    labels.append((len(plan), 'descend to object'))
    plan += up_pick[::-1][1:] + [q_pick]
    q = q_pick.copy()
    wait(GRIP_CLOSED, 'close gripper', 1.2)
    labels.append((len(plan), 'lift'))
    lift = straight_up(q, GRIP_CLOSED)
    plan += lift
    q = lift[-1].copy()

    # ---- place (keep the wrist roll the object was picked with) ----
    q_place = low_pose(place, roll=q[4], clear=place_shift(width))
    p_low, _ = tcp(model, d, q_place)
    # Stop PLACE_ABOVE higher than the grasp height: the object slips a little down in the
    # jaws while carried, so stopping exactly at grasp height would press it into the table.
    q_place, _ = ik(model, d, q_place, p_low + [0, 0, PLACE_ABOVE], PITCH, True, lo, hi)
    q_place[5] = GRIP_CLOSED
    up_place = straight_up(q_place, GRIP_CLOSED)
    go(tcp(model, d, up_place[-1])[0], GRIP_CLOSED, 'carry')
    blend(up_place[-1], 'line up', 0.5)
    labels.append((len(plan), 'lower'))
    plan += up_place[::-1][1:] + [q_place]
    q = q_place.copy()
    wait(g_open, 'open gripper', 1.0)
    labels.append((len(plan), 'lift away'))
    away = straight_up(q, g_open)
    plan += away
    q = away[-1].copy()
    blend(np.append(HOME, g_open), 'return home', 3.0)
    return np.array(plan), labels


def make_sim(pick, height, width):
    '''Box scene with the box resized to the object and placed at the pick spot.'''
    model = mujoco.MjModel.from_xml_path(BOX_SCENE)
    gid = model.geom('box').id
    model.geom_size[gid] = [width / 2, width / 2, height / 2]
    data = mujoco.MjData(model)
    adr = model.joint(model.body('box').jntadr[0]).qposadr[0]
    return model, data, adr


def reset_sim(model, data, adr, q, pick, height):
    mujoco.mj_resetData(model, data)
    data.qpos[:6] = q
    data.ctrl[:] = q
    data.qpos[adr:adr + 7] = [pick[0], pick[1], height / 2, 1, 0, 0, 0]
    mujoco.mj_forward(model, data)


def physics_check(model, data, adr, plan, pick, place, height):
    '''Run the plan in MuJoCo physics, no viewer. Returns (ok, message).'''
    reset_sim(model, data, adr, plan[0], pick, height)
    steps = max(1, round(1 / RATE / model.opt.timestep))
    zmax = 0.0
    for q in plan:
        data.ctrl[:] = q
        for _ in range(steps):
            mujoco.mj_step(model, data)
        zmax = max(zmax, data.qpos[adr + 2])
    for _ in range(RATE * steps):                      # let it settle 1 s
        mujoco.mj_step(model, data)
    end = data.qpos[adr:adr + 3]
    miss = np.linalg.norm(end[:2] - place) * 1000
    lifted = (zmax - height / 2) * 1000
    ok = lifted > 40 and miss < 15
    return ok, (f'object lifted {lifted:.0f} mm, landed {miss:.0f} mm from the place spot, '
                f'standing {"upright" if abs(end[2] - height / 2) < 0.005 else "NOT upright"}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('pick_x', type=float); ap.add_argument('pick_y', type=float)
    ap.add_argument('place_x', type=float); ap.add_argument('place_y', type=float)
    ap.add_argument('--height', type=float, default=60)
    ap.add_argument('--width', type=float, default=40)
    ap.add_argument('--yaw', type=float, default=0)
    ap.add_argument('--speed', type=float, default=0.5)
    ap.add_argument('--real', action='store_true')
    ap.add_argument('--dry', action='store_true')
    a = ap.parse_args()
    pick = np.array([a.pick_x, a.pick_y]) / 1000
    place = np.array([a.place_x, a.place_y]) / 1000
    height, width = a.height / 1000, a.width / 1000

    model, data, adr = make_sim(pick, height, width)
    arm = RealArm(dry=a.dry) if a.real else None
    q_start = arm.read_sim() if arm else np.append(HOME, 0.5)

    plan, labels = build_plan(model, pick, place, height, width, a.yaw, q_start)
    np.save(os.path.join(HERE, 'pick_place_plan.npy'), plan)
    ok, msg = physics_check(model, data, adr, plan, pick, place, height)
    print(f'Plan: {len(plan)} commands ({len(plan) / RATE:.1f} s at speed 1.0). '
          f'Physics check: {"OK" if ok else "FAILED"} - {msg}')
    if arm and not ok:
        arm.close()
        sys.exit('Not running on the real arm because the simulation failed.')

    speed = a.speed if arm else 1.0
    if arm:
        input(f'Real arm will run at speed {speed}. Clear the area, then press Enter (Ctrl+C stops)...')

    reset_sim(model, data, adr, plan[0] if not arm else q_start, pick, height)
    steps = max(1, round(1 / RATE / model.opt.timestep))
    pending = sorted(labels)                        # (frame, step name), printed as reached
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            t_next, i = time.perf_counter(), 0.0
            while viewer.is_running() and int(i) < len(plan):
                k = int(i)
                while pending and pending[0][0] <= k:
                    print(f'\n> {pending.pop(0)[1]}', end='', flush=True)
                q = plan[k]
                data.ctrl[:] = q
                for _ in range(steps):
                    mujoco.mj_step(model, data)
                if arm:
                    arm.send_sim(q)
                viewer.sync()
                i += speed
                t_next += 1 / RATE
                time.sleep(max(0.0, t_next - time.perf_counter()))
            print('\nDone.' if int(i) >= len(plan) else '\nStopped.')
            if not arm:
                while viewer.is_running():                   # leave the window open to look
                    viewer.sync(); time.sleep(0.05)
            viewer.close()
    except KeyboardInterrupt:
        print('\nStopped by user.')
    finally:
        if arm:
            arm.close()
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
