'''Pick an object the camera sees and put it where you click.

Run:  python camera_pick.py              real arm
      python camera_pick.py --dry        fake arm: camera, measuring and sim check only
      python camera_pick.py --speed 1.5  faster (default 1.0, max 2.0; sim check runs at that speed)
      python camera_pick.py --lower 30   grip 30 mm lower than half the object height (default 20;
                                         fingertips always stay >= 4 mm above the table)
In the camera window:
  1. click the OBJECT (on its top). It is measured from the depth image: outline (green),
     height, width between the jaws, length, and how it is turned (yaw).
  2. click the PLACE spot on the table (red cross).
  3. SPACE: plan, check the whole pick in MuJoCo physics, then run it on the real arm.
  s  stop the arm now (holds the pose)     r  start over     h  home     q  quit
Needs a good table calibration (T_base_cam.npy, see SETUP_TABLE_2D_CALIB.md).
Objects: about 15-60 mm tall, 10-65 mm wide across the jaws, standing on the table.
Everything is also written to debug/camera_pick_log.txt.
'''
import argparse
import os
import sys
import threading
import time
import numpy as np
import cv2
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import so101_real                                                    # noqa: E402
from keyboard_control import HOME                                    # noqa: E402
from pick_place import build_plan, smooth, speed_up, joint_step_for, BOX_SCENE                 # noqa: E402
from verify_click import load_offset                                 # noqa: E402
from handeye_calib import open_camera                                # noqa: E402

LOG = os.path.join(HERE, 'debug', 'camera_pick_log.txt')
OBJ_MIN = 0.008        # m above the table: taller pixels belong to an object
SEARCH = 0.12          # m: only look this far (in x, y) around the click
TOP_FRACTION = 0.6     # footprint from points above 60% of the height (the top face; sides are half-hidden)
MAX_WIDTH = 0.065      # m: widest object the gripper opens around
MIN_HEIGHT = 0.012


def cloud_base(depth_frame, intr, T, offset):
    '''Whole depth image -> base-frame points (H x W x 3, m); NaN where there is no depth.'''
    z = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_frame.get_units()
    h, w = z.shape
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    x = (u - intr.ppx) / intr.fx * z
    y = (v - intr.ppy) / intr.fy * z
    P = np.stack([x, y, z], -1) @ T[:3, :3].T + T[:3, 3] + offset
    P[z <= 0] = np.nan
    return P


def measure(P, u, v):
    '''Object under pixel (u, v). Returns dict or an error string.'''
    h, w, _ = P.shape
    if not np.isfinite(P[v, u, 0]):
        win = P[max(0, v - 4):v + 5, max(0, u - 4):u + 5].reshape(-1, 3)
        win = win[np.isfinite(win[:, 0])]
        if len(win) == 0:
            return 'no depth at that pixel: click the middle of the object top'
        c = np.median(win, 0)
    else:
        c = P[v, u]
    near = np.isfinite(P[..., 0]) & (np.abs(P[..., 0] - c[0]) < SEARCH) & (np.abs(P[..., 1] - c[1]) < SEARCH)
    if near.sum() < 50:
        return 'too few depth points around the click'
    zs = P[..., 2]
    ring = near & (zs < OBJ_MIN)
    table = float(np.median(zs[ring])) if ring.sum() > 50 else 0.0
    mask = (near & (zs > table + OBJ_MIN)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab = cv2.connectedComponents(mask)
    k = lab[v, u]
    if k == 0:                                           # clicked just off it: nearest blob within 12 px
        ys, xs = np.nonzero(lab[max(0, v - 12):v + 13, max(0, u - 12):u + 13])
        if len(ys) == 0:
            return f'nothing taller than {OBJ_MIN*1000:.0f} mm under the click (table z here {table*1000:.0f} mm)'
        j = np.argmin((ys - 12) ** 2 + (xs - 12) ** 2)
        k = lab[max(0, v - 12) + ys[j], max(0, u - 12) + xs[j]]
    blob = lab == k
    pts = P[blob]
    height = float(np.percentile(pts[:, 2], 95) - table)
    top = pts[pts[:, 2] - table > TOP_FRACTION * height]
    if len(top) < 20:
        return 'object too small in the image'
    rect = cv2.minAreaRect((top[:, :2] * 1000).astype(np.float32))
    box = cv2.boxPoints(rect) / 1000                     # 4 corners (m)
    e1, e2 = box[1] - box[0], box[2] - box[1]
    short, long_ = (e1, e2) if np.linalg.norm(e1) <= np.linalg.norm(e2) else (e2, e1)
    yaw = float(np.degrees(np.arctan2(short[1], short[0])))
    yaw = (yaw + 90) % 180 - 90                          # jaws work the same turned 180 deg
    centre = np.array(rect[0]) / 1000
    contours, _ = cv2.findContours(blob.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return {'centre': centre, 'height': height, 'width': float(np.linalg.norm(short)),
            'length': float(np.linalg.norm(long_)), 'yaw': yaw, 'table': table,
            'contour': max(contours, key=cv2.contourArea), 'n': int(blob.sum())}


def physics_check(plan, obj, place):
    '''Run the plan in MuJoCo physics with a box of the measured size and yaw.'''
    model = mujoco.MjModel.from_xml_path(BOX_SCENE)
    data = mujoco.MjData(model)
    model.geom_size[model.geom('box').id] = [obj['width'] / 2, obj['length'] / 2, obj['height'] / 2]
    adr = model.joint(model.body('box').jntadr[0]).qposadr[0]
    a = np.radians(obj['yaw'])                           # box x axis along the jaws' closing direction
    data.qpos[:6] = plan[0]
    data.ctrl[:] = plan[0]
    data.qpos[adr:adr + 7] = [*obj['centre'], obj['height'] / 2, np.cos(a / 2), 0, 0, np.sin(a / 2)]
    mujoco.mj_forward(model, data)
    steps = max(1, round(1 / so101_real.RATE / model.opt.timestep))
    zmax = 0.0
    for q in plan:
        data.ctrl[:] = q
        for _ in range(steps):
            mujoco.mj_step(model, data)
        zmax = max(zmax, data.qpos[adr + 2])
    for _ in range(so101_real.RATE * steps):
        mujoco.mj_step(model, data)
    end = data.qpos[adr:adr + 3]
    miss = np.linalg.norm(end[:2] - place) * 1000
    lifted = (zmax - obj['height'] / 2) * 1000
    upright = abs(end[2] - obj['height'] / 2) < 0.006
    ok = lifted > 40 and miss < 20 and upright
    return ok, (f'lifted {lifted:.0f} mm, landed {miss:.0f} mm from the spot, '
                f'{"upright" if upright else "NOT upright"}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dry', action='store_true', help='fake arm: nothing moves')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--lower', type=float, default=20,
                    help='mm lower than half the object height (default 20)')
    ap.add_argument('--calib', default=os.path.join(HERE, 'T_base_cam.npy'))
    ap.add_argument('--port', default=so101_real.PORT)
    ap.add_argument('--robot-id', default=so101_real.ROBOT_ID)
    a = ap.parse_args()
    so101_real.configure(a.port, a.robot_id)
    if not os.path.exists(a.calib):
        sys.exit(f'{a.calib} not found: run table_calib.py first (SETUP_TABLE_2D_CALIB.md).')
    T, offset = np.load(a.calib), load_offset()

    import pyrealsense2 as rs
    model = mujoco.MjModel.from_xml_path(BOX_SCENE)
    pipe, _, _, intr = open_camera(depth=True)
    align = rs.align(rs.stream.color)
    a.speed = float(np.clip(a.speed, 0.1, 2.0))
    arm = so101_real.RealArm(dry=a.dry, max_step_deg=joint_step_for(a.speed))
    print(__doc__.split('In the camera window:')[1].split('Needs')[0])

    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    logf = open(LOG, 'a')
    logf.write(f'\n=== {time.strftime("%Y-%m-%d %H:%M:%S")} ===\n')
    st = {'click': None, 'obj': None, 'place': None, 'place_uv': None,
          'msg': 'click the OBJECT (its top)', 'stop': False, 'thread': None}

    def say(text):
        st['msg'] = text
        print(text)
        logf.write(text + '\n')
        logf.flush()

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            st['click'] = (x, y)

    def play(plan, labels, speed):
        '''Runs in a thread: one command per 1/30 s, `speed` plan frames per command.'''
        pending, i, t = sorted(labels), 0.0, time.perf_counter()
        while int(i) < len(plan) and not st['stop']:
            k = int(i)
            while pending and pending[0][0] <= k:
                say(f'> {pending.pop(0)[1]}')
            arm.send_sim(plan[k])
            i += speed
            t += 1 / so101_real.RATE
            time.sleep(max(0.0, t - time.perf_counter()))
        say('stopped: arm holds its pose (h = home)' if st['stop'] else 'done. Click the next object')
        st['stop'] = False

    def run_path(path):
        st['thread'] = threading.Thread(target=play, args=(path, [], 1.0), daemon=True)
        st['thread'].start()

    def busy():
        return st['thread'] is not None and st['thread'].is_alive()

    def start_pick():
        '''Plan + sim check; if it fails, retry gripping higher (lower 20 -> 10 -> 0 mm).
        On failure the object stays selected: click another place spot, or r for another object.'''
        obj, place = st['obj'], st['place']
        q_start = arm.read_sim()
        say('planning...')
        tries = [a.lower] + [x for x in (10.0, 0.0) if x < a.lower]
        why = ''
        for lower in tries:
            try:
                plan, labels = build_plan(model, obj['centre'], place, obj['height'], obj['width'],
                                          obj['yaw'], q_start, lower / 1000)
                plan, labels = speed_up(plan, labels, a.speed)
            except SystemExit as e:
                why = f'cannot plan: {e}'
                continue
            ok, msg = physics_check(plan, obj, place)
            if ok:
                break
            why = f'sim check FAILED: {msg}'
        else:
            say(why)
            r_obj = np.hypot(*obj['centre']) * 1000
            if 'from the object' in why or 'Cannot reach [' in why and r_obj < 150:
                say(f'NOT running: the OBJECT is {r_obj:.0f} mm from the base, too close/far. '
                    'Move it to 150-300 mm, then r and click it again')
            else:
                say('NOT running. Click another PLACE spot (object kept), or r to choose another object')
            st.update(place=None, place_uv=None)
            return False
        if lower != a.lower:
            say(f'note: gripping only {lower:.0f} mm lower (not {a.lower:.0f}): lower failed here')
        say(f'sim check OK: {msg}')
        say(f'running ({len(plan) / so101_real.RATE:.0f} s at speed {a.speed}), s = stop')
        st['thread'] = threading.Thread(target=play, args=(plan, labels, 1.0), daemon=True)
        st['thread'].start()
        return True

    win = 'camera_pick  (click object, click place, SPACE go | s stop | r reset | h home | q quit)'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    try:
        while True:
            ok, fs = pipe.try_wait_for_frames(2000)
            if not ok:
                say('camera sent no frame for 2 s: replug it if this repeats')
                continue
            frames = align.process(fs)
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            img = np.asanyarray(color.get_data()).copy()

            if st['click'] is not None and not busy():
                u, v = st['click']
                st['click'] = None
                P = cloud_base(depth, intr, T, offset)
                if st['obj'] is None:
                    r = measure(P, u, v)
                    if isinstance(r, str):
                        say(r)
                    else:
                        c = r['centre'] * 1000
                        say(f'object at x {c[0]:.0f} y {c[1]:.0f} mm: height {r["height"]*1000:.0f}, '
                            f'width {r["width"]*1000:.0f}, length {r["length"]*1000:.0f} mm, '
                            f'yaw {r["yaw"]:.0f} deg (table z {r["table"]*1000:+.0f})')
                        if r['width'] > MAX_WIDTH:
                            say(f'too wide for the gripper ({r["width"]*1000:.0f} mm > {MAX_WIDTH*1000:.0f}): r to reset')
                        elif r['height'] < MIN_HEIGHT:
                            say(f'too flat ({r["height"]*1000:.0f} mm): r to reset')
                        else:
                            st['obj'] = r
                            st['msg'] += '  -> click the PLACE spot'
                else:
                    p = P[v, u]
                    if not np.isfinite(p[0]):
                        say('no depth at that pixel: click a little to the side')
                    else:
                        if p[2] > 0.015:
                            say(f'warning: that spot is {p[2]*1000:.0f} mm above the table (on something?)')
                        st['place'], st['place_uv'] = p[:2].copy(), (u, v)
                        say(f'place at x {p[0]*1000:.0f} y {p[1]*1000:.0f} mm  -> SPACE to pick and place')

            if st['obj'] is not None:
                cv2.drawContours(img, [st['obj']['contour']], -1, (0, 255, 0), 2)
                c = st['obj']['contour'].reshape(-1, 2).mean(0).astype(int)
                o = st['obj']
                cv2.putText(img, f'h{o["height"]*1000:.0f} w{o["width"]*1000:.0f} yaw{o["yaw"]:.0f}',
                            (int(c[0]) + 10, int(c[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            if st['place_uv'] is not None:
                cv2.drawMarker(img, st['place_uv'], (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
            cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
            cv2.putText(img, st['msg'][:110], (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            if busy():
                cv2.putText(img, 'ARM MOVING  (s = stop)', (8, img.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow(win, img)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                st['stop'] = True
                break
            elif key == ord('s') and busy():
                st['stop'] = True
            elif key == ord('r') and not busy():
                st.update(obj=None, place=None, place_uv=None)
                say('reset: click the OBJECT (its top)')
            elif key == ord('h') and not busy():
                q0 = arm.read_sim()
                q1 = np.append(HOME, q0[5])
                run_path([q0 + smooth(k / 90) * (q1 - q0) for k in range(1, 91)])
                say('going home...')
            elif key == ord(' ') and not busy():
                if st['obj'] is None or st['place'] is None:
                    say('click the object, then the place spot, then SPACE')
                else:
                    if start_pick():
                        st.update(obj=None, place=None, place_uv=None)
    except KeyboardInterrupt:
        st['stop'] = True
    finally:
        if st['thread'] is not None:
            st['thread'].join(timeout=2)
        pipe.stop()
        cv2.destroyAllWindows()
        arm.close()


if __name__ == '__main__':
    main()
