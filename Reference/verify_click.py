'''Check the camera calibration: click a point on the table, the gripper goes right above it.

Run:  python Reference/verify_click.py            fixed-jaw tip goes 4 cm above the clicked point
      python Reference/verify_click.py --tcp      aim the grip centre instead (the point pick_place.py uses)
      python Reference/verify_click.py --dry      fake arm: camera + maths only
Mouse: left-click a point (e.g. a pen X on the table). Its x, y, z in the robot base frame appear
       on the status line (top of the image). On the table z should be close to 0.
Keys (click the image window first):
  SPACE  go now: move 4 cm above the clicked point (no confirmation)
  + / -  height above the point +1 / -1 cm (5 mm .. 20 cm); moves again if already there
  g      plan the move and ask first: y = go, n = cancel
  after arriving, if the tip is not right above the X:
  i / k  nudge +x / -x (away from / toward the base)   j / l  nudge +y / -y (left / right)
         (2 mm per press, seen from behind the robot)
  o  save the nudge as a permanent XY correction (camera_offset.json, used for every click)
  0  reset the saved correction to zero
  h  go back to home pose          q  quit
Everything is also written to debug/verify_log.txt. Guide: docs/SETUP_TABLE_2D_CALIB.md
'''
import argparse
import json
import os
import sys
import time
import numpy as np
import cv2
import mujoco

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import so101_real                                                    # noqa: E402
from keyboard_control import ik, tcp, HOME                           # noqa: E402
from pick_place import GRASP_SEED, smooth                            # noqa: E402
from handeye_calib import open_camera                                # noqa: E402

LOG = os.path.join(HERE, 'debug', 'verify_log.txt')
OFFSET_FILE = os.path.join(HERE, 'json', 'camera_offset.json')
ABOVE = 0.04          # m above the clicked point
PITCH = 80.0          # gripper almost straight down (same as pick_place.py)
MOVE_SECONDS = 3.0
NUDGE = 0.002         # m per i/j/k/l press
DEPTH_WIN = 5         # median depth over a (2*5+1)^2 pixel window
JAW_TIP = 'fixed_jaw_sph_tip1'


def pixel_to_cam(depth_frame, intr, u, v):
    '''Clicked pixel -> 3D point in the camera frame (m), or None if no depth there.'''
    import pyrealsense2 as rs
    h, w = depth_frame.get_height(), depth_frame.get_width()
    zs = [depth_frame.get_distance(x, y)
          for y in range(max(0, v - DEPTH_WIN), min(h, v + DEPTH_WIN + 1))
          for x in range(max(0, u - DEPTH_WIN), min(w, u + DEPTH_WIN + 1))]
    zs = [z for z in zs if z > 0]
    if len(zs) < 5:
        return None
    return np.array(rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], float(np.median(zs))))


def load_offset():
    '''Saved XY correction (m), added to every camera point.'''
    try:
        o = json.load(open(OFFSET_FILE))
        return np.array([o['dx_mm'], o['dy_mm'], 0.0]) / 1000
    except (OSError, ValueError, KeyError):
        return np.zeros(3)


def save_offset(off):
    json.dump({'dx_mm': round(float(off[0]) * 1000, 1), 'dy_mm': round(float(off[1]) * 1000, 1),
               'note': 'added to every camera point (robot base frame); set by verify_click.py'},
              open(OFFSET_FILE, 'w'), indent=1)


def pixel_to_base(depth_frame, intr, u, v, T, offset=None):
    '''Clicked pixel -> 3D point in the robot base frame (m), XY correction included.'''
    p_cam = pixel_to_cam(depth_frame, intr, u, v)
    if p_cam is None:
        return None
    return T[:3, :3] @ p_cam + T[:3, 3] + (load_offset() if offset is None else offset)


def jaw_tip(model, d, q):
    d.qpos[:5] = q[:5]
    mujoco.mj_kinematics(model, d)
    return d.geom_xpos[model.geom(JAW_TIP).id].copy()


def aim_point(model, d, q, use_tcp):
    return tcp(model, d, q)[0] if use_tcp else jaw_tip(model, d, q)


def solve(model, d, target, lo, hi, use_tcp, seed=None):
    '''IK so that the aimed point (jaw tip or grip centre) is at target. Returns (q, error m).'''
    if seed is None:
        seed = GRASP_SEED.copy()
        seed[0] = -np.arctan2(target[1], target[0])            # +pan turns toward -y
    seed = np.array(seed[:5], dtype=float)
    site_target = target.copy()
    q = seed
    for _ in range(5):          # IK works on the site; for the jaw tip, shift the site target by the miss
        q, site_err = ik(model, d, seed, site_target, PITCH, True, lo, hi, iters=200)
        if site_err > 0.004:                                    # can't keep the tilt: let it free
            q, site_err = ik(model, d, seed, site_target, PITCH, False, lo, hi, iters=200)
        miss = target - aim_point(model, d, q, use_tcp)
        if use_tcp or np.linalg.norm(miss) < 0.0005:
            break
        site_target, seed = site_target + miss, q
    return q, np.linalg.norm(aim_point(model, d, q, use_tcp) - target)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=so101_real.PORT)
    ap.add_argument('--robot-id', default=so101_real.ROBOT_ID)
    ap.add_argument('--calib', default=os.path.join(HERE, 'T_base_cam.npy'))
    ap.add_argument('--tcp', action='store_true', help='aim the grip centre (gripperframe) instead of the fixed-jaw tip')
    ap.add_argument('--dry', action='store_true', help='fake arm: nothing moves')
    ap.add_argument('--no-sag', action='store_true', help='do not correct servo sag after arriving')
    a = ap.parse_args()
    so101_real.configure(a.port, a.robot_id)
    if not os.path.exists(a.calib):
        sys.exit(f'{a.calib} not found: run table_calib.py first (docs/SETUP_TABLE_2D_CALIB.md).')
    T = np.load(a.calib)
    offset = load_offset()
    point_name = 'grip centre' if a.tcp else 'fixed-jaw tip'

    import pyrealsense2 as rs
    model = mujoco.MjModel.from_xml_path(so101_real.SCENE)
    d = mujoco.MjData(model)
    r_lo, r_hi = so101_real.real_limits_sim()
    lo = np.maximum(model.actuator_ctrlrange[:5, 0], r_lo)
    hi = np.minimum(model.actuator_ctrlrange[:5, 1], r_hi)

    pipe, _, _, intr = open_camera(depth=True)
    align = rs.align(rs.stream.color)
    arm = so101_real.RealArm(dry=a.dry)
    print(__doc__.split('Mouse:')[1].split('Everything')[0])

    st = {'click': None, 'p': None, 'uv': None, 'q_goal': None, 'path': [], 'fps': 0.0,
          'msg': f'click a point on the table  (aiming the {point_name})', 'arrive': '', 'settle': 0,
          'target': None, 'q_cmd': None, 'nudge': np.zeros(3), 'arrived': False, 'above': ABOVE}

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            st['click'] = (x, y)

    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    logf = open(LOG, 'a')
    logf.write(f'\n=== {time.strftime("%Y-%m-%d %H:%M:%S")}  aiming {point_name}, '
               f'T_base_cam translation (mm) {np.round(T[:3, 3] * 1000)}, '
               f'XY correction (mm) {np.round(offset[:2] * 1000, 1)} ===\n')

    def log(text):
        print(text)
        logf.write(text + '\n')
        logf.flush()

    def say(text):
        st['msg'] = text
        log(text)

    def path_to(q_goal):
        q0 = arm.read_sim()
        n = int(MOVE_SECONDS * so101_real.RATE)
        return [q0 + smooth(k / n) * (q_goal - q0) for k in range(n + 1)]

    def go(target, quick=False):
        '''Plan to target (m); quick=True moves at once (nudges), else waits for y.'''
        q, err = solve(model, d, target, lo, hi, a.tcp, seed=st['q_cmd'] if quick else None)
        if err > 0.004:
            say(f'out of reach (IK error {err*1000:.0f} mm): pick a point 180-300 mm from the base')
            return
        st['target'] = target
        if quick:
            st['q_cmd'] = np.append(q[:5], arm.read_sim()[5])
            q0 = arm.read_sim()
            st['path'] = [q0 + smooth(k / 15) * (st['q_cmd'] - q0) for k in range(16)]
            st['arrive'] = 'nudged'
        else:
            st['q_goal'] = q
            say(f'IK ok ({err*1000:.1f} mm). Press y to move the {point_name} {st["above"]*1000:.0f} mm above the point, n to cancel')

    def start_move():
        st['q_cmd'] = np.append(st['q_goal'][:5], arm.read_sim()[5])
        st.update(path=path_to(st['q_cmd']), q_goal=None, arrive='move', arrived=False)
        say('moving...')

    win = 'verify_click  (click + SPACE go | +/- height | g/y/n | i/k/j/l nudge | o save | h home | q quit)'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    t_last = time.perf_counter()
    try:
        while True:
            ok, fs = pipe.try_wait_for_frames(2000)
            if not ok:
                say('camera sent no frame for 2 s (USB 2 bandwidth?): replug the camera if this repeats')
                continue
            frames = align.process(fs)
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            img = np.asanyarray(color.get_data()).copy()
            now = time.perf_counter()
            st['fps'] = 0.9 * st['fps'] + 0.1 / max(now - t_last, 1e-3)
            t_last = now

            if st['path']:                                       # moving: one small step per frame
                arm.send_sim(st['path'].pop(0))
                if not st['path']:
                    st['settle'] = 20                           # ~0.7 s, then correct sag and report
            elif st['settle']:
                st['settle'] -= 1
                arm.send_sim(st['q_cmd'])                    # keep sending: the speed limit catches up
                if st['settle'] == 0 and st['target'] is not None:
                    before = np.degrees(arm.read_sim()[:5] - st['q_cmd'][:5])
                    if not a.no_sag and not a.dry:
                        arm.correct_sag(st['q_cmd'])            # ~1 s: re-aim joints that sagged
                    q_real = arm.read_sim()
                    real = aim_point(model, d, q_real, a.tcp)
                    tgt = st['target']
                    dx = (real - tgt) * 1000
                    log(f'--- arrived ({st["arrive"]}) ---')
                    log(f'  target               x {tgt[0]*1000:6.0f}  y {tgt[1]*1000:6.0f}  z {tgt[2]*1000:6.0f} mm'
                        f'   (nudge so far x {st["nudge"][0]*1000:+.0f} y {st["nudge"][1]*1000:+.0f} mm)')
                    log(f'  {point_name} (servos)  x {real[0]*1000:6.0f}  y {real[1]*1000:6.0f}  z {real[2]*1000:6.0f} mm'
                        f'   -> off by x {dx[0]:+.0f} y {dx[1]:+.0f} z {dx[2]:+.0f} mm')
                    log(f'  joint sag before correction (deg): {np.round(before, 1)}  [pan lift elbow wrist roll]')
                    st['arrived'] = True
                    st['msg'] = ('arrived. Over the X?  if not: i/k/j/l to nudge, then o to save the correction'
                                 if not st['nudge'].any() else
                                 f'nudge x {st["nudge"][0]*1000:+.0f} y {st["nudge"][1]*1000:+.0f} mm. '
                                 'Right over the X? press o to save')

            if st['click'] is not None:
                u, v = st['click']
                st['click'] = None
                p = pixel_to_base(depth, intr, u, v, T, offset)
                if p is None:
                    say('no depth at that pixel: click a little to the side')
                else:
                    st.update(p=p, uv=(u, v), q_goal=None, nudge=np.zeros(3), arrived=False)
                    say(f'point x {p[0]*1000:.0f}  y {p[1]*1000:.0f}  z {p[2]*1000:.0f} mm  (pixel {u},{v})  -> SPACE to go')

            if st['p'] is not None:
                u, v = st['uv']
                p = st['p']
                cv2.drawMarker(img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
                cv2.putText(img, f'x {p[0]*1000:.0f}  y {p[1]*1000:.0f}  z {p[2]*1000:.0f} mm', (u + 15, v - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
            cv2.putText(img, st['msg'], (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.putText(img, f'{st["fps"]:.0f} fps   XY correction x {offset[0]*1000:+.1f} y {offset[1]*1000:+.1f} mm'
                        + ('   MOVING' if st['path'] else ''),
                        (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            cv2.imshow(win, img)
            key = cv2.waitKey(1) & 0xFF
            busy = bool(st['path']) or st['settle'] > 0

            if key == ord('q'):
                break
            elif key == ord('h') and not busy:
                st['q_cmd'] = np.append(HOME, arm.read_sim()[5])
                st.update(path=path_to(st['q_cmd']), q_goal=None, target=None,
                          arrived=False, arrive='home')
                say('going home...')
            elif key == ord('g') and not busy:
                if st['p'] is None:
                    say('click a point first')
                else:
                    st['nudge'] = np.zeros(3)
                    go(st['p'] + np.array([0, 0, st['above']]))
            elif key == ord(' ') and not busy:                  # SPACE: plan and go at once
                if st['p'] is None:
                    say('click a point first')
                else:
                    st['nudge'] = np.zeros(3)
                    go(st['p'] + np.array([0, 0, st['above']]))
                    if st['q_goal'] is not None:
                        start_move()
            elif chr(key) in '+=-_' and not busy:
                up = chr(key) in '+='                           # 10 mm steps, plus 5 mm at the bottom
                a_ = st['above']
                st['above'] = min(0.20, a_ + 0.01 if a_ >= 0.01 else 0.01) if up else (a_ - 0.01 if a_ > 0.015 else 0.005)
                if st['p'] is not None and st['arrived']:          # already there: go to the new height
                    st['arrived'] = False
                    go(st['p'] + np.array([0, 0, st['above']]) + st['nudge'], quick=True)
                    if st['path']:
                        st['path'] = path_to(st['q_cmd'])           # same smooth 3 s move as SPACE
                        st['arrive'] = 'height'
                say(f'height above the point: {st["above"]*1000:.0f} mm'
                    + ('  (SPACE to go)' if not st['arrived'] and st['p'] is not None and not st['path'] else ''))
            elif key == ord('y') and st['q_goal'] is not None and not busy:
                start_move()
            elif key == ord('n') and st['q_goal'] is not None:
                st['q_goal'] = None
                say('cancelled')
            elif chr(key) in 'ikjl' and st['arrived'] and not busy:
                step = {'i': (NUDGE, 0), 'k': (-NUDGE, 0), 'j': (0, NUDGE), 'l': (0, -NUDGE)}[chr(key)]
                st['nudge'] = st['nudge'] + np.array([step[0], step[1], 0])
                st['arrived'] = False
                go(st['p'] + np.array([0, 0, st['above']]) + st['nudge'], quick=True)
            elif key == ord('o'):
                if not st['nudge'].any():
                    say('nothing to save: nudge with i/k/j/l first')
                else:
                    offset = offset + st['nudge']
                    save_offset(offset)
                    st['p'] = st['p'] + st['nudge']             # same physical X, now with the correction
                    st['nudge'] = np.zeros(3)
                    say(f'saved XY correction x {offset[0]*1000:+.1f} y {offset[1]*1000:+.1f} mm '
                        '(camera_offset.json). Check other X marks')
            elif key == ord('0'):
                offset = np.zeros(3)
                save_offset(offset)
                say('XY correction reset to 0 (applies to the next click)')
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        arm.close()


if __name__ == '__main__':
    main()
