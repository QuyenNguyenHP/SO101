'''Send the gripper to a 3D point (mm, robot base frame: x forward, y left, z up from the table).

Run:  python goto.py 250 0 80                 grip centre to x 250, y 0, z 80 mm, pointing down
      python goto.py 250 0 80 --pitch 30      gripper tilted 30 deg below horizontal (0 = level)
      python goto.py 250 0 80 --jaw           aim the fixed-jaw tip instead (like verify_click.py)
      python goto.py                          no point: start at the prompt below
      python goto.py 250 0 80 --dry           fake arm: maths only, nothing moves
      python goto.py 250 0 80 --speed 1.5     faster (default 1.0 = 8 cm/s straight moves, max 2.0)
Same coordinates as verify_click.py, keyboard_control.py and pick_place.py.

After each move the arm holds the pose and asks for the next command:
  250 0 80          go to x y z (mm)          250 0 80 45   ... with pitch 45 deg
  o / c             open / close the gripper  g 30           gripper to 30 (0 closed .. 100 open)
  w                 where is the tip now      h              home       q   quit
How it moves: if the tip is low (< 6 cm) it first rises straight up, then swings above the
target, then goes straight down to it, so it never sweeps through objects on the table.
'''
import argparse
import os
import sys
import time
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import so101_real                                                    # noqa: E402
from keyboard_control import ik, tcp, pitch_of, HOME                 # noqa: E402
from pick_place import GRASP_SEED, smooth                            # noqa: E402

JAW_TIP = 'fixed_jaw_sph_tip1'
TIPS = ['fixed_jaw_sph_tip1', 'moving_jaw_sph_tip1']   # lowest points of the gripper
SAFE_Z = 0.06          # m: travel height; lower targets are approached straight from above
MIN_CLEAR = 0.003      # m: no fingertip may end lower than this above the table
SPEED = 0.08           # m/s for straight-line moves
SWING_SECONDS = 2.5    # joint-space swing between two "above" points
REACH_TOL = 0.004      # m


def lowest_tip(model, d, q):
    '''Lowest fingertip z (bottom of the sphere), m.'''
    d.qpos[:6] = q[:6] if len(q) >= 6 else np.append(q[:5], 0.5)
    mujoco.mj_kinematics(model, d)
    return min(d.geom_xpos[model.geom(g).id][2] - model.geom_size[model.geom(g).id][0] for g in TIPS)


def aim_point(model, d, q, use_jaw):
    if not use_jaw:
        return tcp(model, d, q)[0]
    d.qpos[:5] = q[:5]
    mujoco.mj_kinematics(model, d)
    g = model.geom(JAW_TIP).id
    return d.geom_xpos[g].copy() - [0, 0, model.geom_size[g][0]]


def solve(model, d, target, pitch, lo, hi, use_jaw, seeds):
    '''IK for the aimed point at target with the given pitch (locked). Returns (q5, error m).'''
    best = (None, np.inf)
    for seed in seeds:
        seed = np.array(seed[:5], float)
        site_t, q = target.copy(), seed
        for _ in range(5):              # IK works on the grip-centre site: shift it for the jaw tip
            q, _ = ik(model, d, seed, site_t, pitch, True, lo, hi, iters=300)
            miss = target - aim_point(model, d, q, use_jaw)
            if not use_jaw or np.linalg.norm(miss) < 0.0005:
                break
            site_t, seed = site_t + miss, q
        err = np.linalg.norm(aim_point(model, d, q, use_jaw) - target)
        _, axis = tcp(model, d, q)
        err += 0.001 * abs(pitch_of(axis) - pitch)          # 1 mm per degree of tilt missed
        if err < best[1]:
            best = (q.copy(), err)
    return best


def solve_any_pitch(model, d, target, pitch, lo, hi, use_jaw, q_now):
    '''Try the asked pitch first, then the nearest pitches that reach. Returns (q5, pitch, err).'''
    seeds = [GRASP_SEED.copy(), HOME.copy(), q_now[:5].copy()]
    for s in seeds[:2]:
        s[0] = -np.arctan2(target[1], target[0])           # face the point (+pan turns toward -y)
    for p in sorted(set([pitch] + list(range(90, -31, -10))), key=lambda p: abs(p - pitch)):
        q, err = solve(model, d, target, p, lo, hi, use_jaw, seeds)
        if err < REACH_TOL:
            return q, p, err
    return None, None, None


def line(model, d, q, p_from, p_to, pitch, lo, hi):
    '''Straight line for the grip-centre site. Returns list of q (rad, 5) or raises.'''
    n = max(2, int(np.linalg.norm(p_to - p_from) / SPEED * so101_real.RATE))
    path, q = [], q[:5].copy()
    for k in range(1, n + 1):
        q, err = ik(model, d, q, p_from + smooth(k / n) * (p_to - p_from), pitch, True, lo, hi)
        if err > 0.010:
            raise ValueError(f'cannot move in a straight line near {np.round((p_from + k / n * (p_to - p_from)) * 1000)} mm')
        path.append(q.copy())
    return path


def plan_to(model, d, q_now, q_goal, pitch, lo, hi):
    '''Rise (if low) -> swing above the goal -> straight down. Returns list of q (rad, 5).'''
    q_now = q_now[:5].copy()
    path = []
    p_now, axis_now = tcp(model, d, q_now)
    p_goal, _ = tcp(model, d, q_goal)
    pitch_now = pitch_of(axis_now)
    if lowest_tip(model, d, q_now) < SAFE_Z - 0.01:                 # 1) rise straight up
        up = p_now + [0, 0, SAFE_Z - lowest_tip(model, d, q_now) + 0.005]
        try:
            path += line(model, d, q_now, p_now, up, pitch_now, lo, hi)
        except ValueError:                                           # can't keep the tilt: free rise
            path += [q_now]
    q_mid = path[-1] if path else q_now
    if lowest_tip(model, d, q_goal) < SAFE_Z - 0.01:                 # 3) straight down at the end
        lift = SAFE_Z - lowest_tip(model, d, q_goal) + 0.005
        up_goal = line(model, d, q_goal, p_goal, p_goal + [0, 0, lift], pitch, lo, hi)
        down = up_goal[::-1][1:] + [q_goal[:5]]
        q_above = up_goal[-1]
    else:
        down, q_above = [], q_goal[:5]
    n = int(SWING_SECONDS * so101_real.RATE)                         # 2) joint-space swing
    path += [q_mid + smooth(k / n) * (q_above - q_mid) for k in range(1, n + 1)]
    return path + down


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('xyz', type=float, nargs='*', help='x y z in mm (optional)')
    ap.add_argument('--pitch', type=float, default=80.0, help='deg below horizontal (90 = straight down)')
    ap.add_argument('--jaw', action='store_true', help='aim the fixed-jaw tip instead of the grip centre')
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--speed', type=float, default=1.0, help='0.1 .. 2.0')
    ap.add_argument('--no-sag', action='store_true', help='do not correct servo sag after arriving')
    ap.add_argument('--port', default=so101_real.PORT)
    ap.add_argument('--robot-id', default=so101_real.ROBOT_ID)
    a = ap.parse_args()
    if a.xyz and len(a.xyz) != 3:
        sys.exit('give 3 numbers: x y z (mm)')
    so101_real.configure(a.port, a.robot_id)

    model = mujoco.MjModel.from_xml_path(so101_real.SCENE)
    d = mujoco.MjData(model)
    r_lo, r_hi = so101_real.real_limits_sim()
    lo = np.maximum(model.actuator_ctrlrange[:5, 0], r_lo)
    hi = np.minimum(model.actuator_ctrlrange[:5, 1], r_hi)
    name = 'fixed-jaw tip' if a.jaw else 'grip centre'
    global SPEED, SWING_SECONDS
    a.speed = float(np.clip(a.speed, 0.1, 2.0))
    SPEED, SWING_SECONDS = SPEED * a.speed, SWING_SECONDS / a.speed
    arm = so101_real.RealArm(dry=a.dry, max_step_deg=float(np.clip(3.0 * a.speed, 2.0, 6.0)))
    q_hold = arm.read_sim()

    def run(path, grip):
        nonlocal q_hold
        t = time.perf_counter()
        for q in path + [path[-1]] * 15:                 # extra frames: the speed limit catches up
            q_hold = np.append(q[:5], grip)
            arm.send_sim(q_hold)
            t += 1 / so101_real.RATE
            time.sleep(max(0.0, t - time.perf_counter()))

    def where(target=None):
        q = arm.read_sim()
        p = aim_point(model, d, q, a.jaw) * 1000
        _, axis = tcp(model, d, q)
        msg = f'  {name} now  x {p[0]:6.0f}  y {p[1]:6.0f}  z {p[2]:6.0f} mm   pitch {pitch_of(axis):.0f} deg'
        if target is not None:
            e = p - target * 1000
            msg += f'   (off by x {e[0]:+.0f} y {e[1]:+.0f} z {e[2]:+.0f} mm)'
        print(msg)

    def goto(target, pitch):
        q_now = arm.read_sim()
        q, p_used, err = solve_any_pitch(model, d, target, pitch, lo, hi, a.jaw, q_now)
        if q is None:
            r = np.hypot(*target[:2]) * 1000
            print(f'  out of reach: {np.round(target * 1000)} mm is {r:.0f} mm from the base '
                  '(reach is about 100-380 mm, lower when pointing down)')
            return
        if lowest_tip(model, d, q) < MIN_CLEAR:
            print(f'  refused: a fingertip would end {lowest_tip(model, d, q) * 1000:.0f} mm from the table. '
                  f'Raise z (the {name} is not the lowest point at this tilt)')
            return
        if p_used != pitch:
            print(f'  pitch {pitch:.0f} deg does not reach there: using {p_used:.0f} deg')
        q[4] = q_now[4]                                   # keep the wrist roll
        try:
            path = plan_to(model, d, q_now, q, p_used, lo, hi)
        except ValueError as e:
            print(f'  {e}: try a point a little higher or closer')
            return
        print(f'  moving ({len(path) / so101_real.RATE:.1f} s)...')
        run(path, q_now[5])
        if not a.no_sag and not a.dry:
            arm.correct_sag(np.append(q, q_now[5]))
        where(target)

    if a.xyz:
        goto(np.array(a.xyz) / 1000, a.pitch)
    print(__doc__.split('After each move')[1].split('How it moves')[0].split('\n', 1)[1])
    try:
        while True:
            try:
                cmd = input('goto> ').strip().lower().split()
            except EOFError:
                break
            if not cmd:
                continue
            if cmd[0] == 'q':
                break
            elif cmd[0] == 'w':
                where()
            elif cmd[0] == 'h':
                q_now = arm.read_sim()
                q_home = HOME.copy()
                try:
                    path = plan_to(model, d, q_now, q_home, pitch_of(tcp(model, d, q_home)[1]), lo, hi)
                except ValueError:
                    path = [q_now[:5] + smooth(k / 75) * (q_home - q_now[:5]) for k in range(1, 76)]
                run(path, q_now[5])
                where()
            elif cmd[0] in ('o', 'c', 'g'):
                g = {'o': 100.0, 'c': 0.0}.get(cmd[0])
                if g is None:
                    if len(cmd) < 2:
                        print('  g needs a number 0..100'); continue
                    g = float(np.clip(float(cmd[1]), 0, 100))
                q_now = arm.read_sim()
                g_sim = so101_real.real_gripper_to_sim(g)
                run([q_hold[:5]] * 30, g_sim)            # 1 s; send_sim ramps the gripper
            else:
                try:
                    v = [float(x) for x in cmd]
                except ValueError:
                    print('  ? type: x y z [pitch]  |  o  c  g N  w  h  q'); continue
                if len(v) not in (3, 4):
                    print('  give x y z (mm) and optionally pitch (deg)'); continue
                goto(np.array(v[:3]) / 1000, v[3] if len(v) == 4 else a.pitch)
    except KeyboardInterrupt:
        print()
    finally:
        arm.close()


if __name__ == '__main__':
    main()
