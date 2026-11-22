'''Pick an object the camera sees and put it where you click.

Run:  python camera_pick.py              real arm
      python camera_pick.py --dry        fake arm: camera, measuring and sim check only
      python camera_pick.py --speed 1.5  faster (default 1.0, max 2.0; sim check runs at that speed)
      python camera_pick.py --lower 30   grip 30 mm lower than half the object height (default 20;
                                         fingertips always stay >= 4 mm above the table)
      python camera_pick.py --detect "green cube, white cube, tape measure"
                                         find every object on the table (depth image, arm masked out)
                                         and name it (CLIP): numbered boxes with size and angle.
                                         Press 1-9 (or click a box) to pick it. --backend yolo: YOLO-World
In the camera window:
  1. click the OBJECT (on its top), or with --detect press its number 1-9. It is measured from the
     depth image: outline (green), height, width between the jaws, length, and how it is turned (yaw).
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
from detector import robot_mask                                      # noqa: E402

LOG = os.path.join(HERE, 'debug', 'camera_pick_log.txt')
OBJ_MIN = 0.008        # m above the table: taller pixels belong to an object
SEARCH = 0.12          # m: only look this far (in x, y) around the click
TOP_FRACTION = 0.6     # footprint from points above 60% of the height (the top face; sides are half-hidden)
MAX_WIDTH = 0.065      # m: widest object the gripper opens around
MIN_HEIGHT = 0.012


def cloud_base(depth_frame, intr, T, offset):
    '''Whole depth image -> base-frame points (H x W x 3, m); NaN where there is no depth.'''
    return cloud_from(np.asanyarray(depth_frame.get_data()), depth_frame.get_units(), intr, T, offset)


def cloud_from(z_raw, units, intr, T, offset):
    '''Same, from a raw depth array (uint16) and its units (m per step).'''
    z = z_raw.astype(np.float32) * units
    h, w = z.shape
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    x = (u - intr.ppx) / intr.fx * z
    y = (v - intr.ppy) / intr.fy * z
    P = np.stack([x, y, z], -1) @ T[:3, :3].T + T[:3, 3] + offset
    P[z <= 0] = np.nan
    return P


def measure(P, u, v, roi=None, robot=None):
    '''Object under pixel (u, v), or with roi=(x0, y0, x1, y1) (a detection box) the biggest
    raised blob inside that box. robot: image mask of the arm (never part of an object).
    Returns dict or an error string.'''
    h, w, _ = P.shape
    if roi is not None:
        x0, y0, x1, y1 = roi
        u, v = int(np.clip((x0 + x1) // 2, 0, w - 1)), int(np.clip((y0 + y1) // 2, 0, h - 1))
        inner = P[max(0, y0 + (y1 - y0) // 4):y1 - (y1 - y0) // 4, max(0, x0 + (x1 - x0) // 4):x1 - (x1 - x0) // 4]
        inner = inner.reshape(-1, 3)
        inner = inner[np.isfinite(inner[:, 0])]
        if len(inner) < 10:
            return 'no depth inside the box'
        P_seed = np.median(inner, 0)
    if not np.isfinite(P[v, u, 0]):
        win = P[max(0, v - 4):v + 5, max(0, u - 4):u + 5].reshape(-1, 3)
        win = win[np.isfinite(win[:, 0])]
        if len(win) == 0:
            return 'no depth at that pixel: click the middle of the object top'
        c = np.median(win, 0)
    else:
        c = P[v, u]
    if roi is not None:
        c = P_seed
    near = np.isfinite(P[..., 0]) & (np.abs(P[..., 0] - c[0]) < SEARCH) & (np.abs(P[..., 1] - c[1]) < SEARCH)
    if near.sum() < 50:
        return 'too few depth points around the click'
    zs = P[..., 2]
    ring = near & (zs < OBJ_MIN)
    table = float(np.median(zs[ring])) if ring.sum() > 50 else 0.0
    mask = (near & (zs > table + OBJ_MIN)).astype(np.uint8)
    if robot is not None:
        mask[robot] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    if roi is not None:                                  # only what is inside the box (+8 px)
        keep = np.zeros_like(mask)
        keep[max(0, y0 - 8):y1 + 8, max(0, x0 - 8):x1 + 8] = 1
        mask &= keep
    n, lab = cv2.connectedComponents(mask)
    k = lab[v, u]
    if roi is not None:
        counts = np.bincount(lab.ravel(), minlength=n)
        counts[0] = 0
        if counts.max() == 0:
            return f'nothing taller than {OBJ_MIN*1000:.0f} mm in the box (flat or no depth)'
        k = int(counts.argmax())
    if k == 0:                                           # clicked just off it: nearest blob within 12 px
        ys, xs = np.nonzero(lab[max(0, v - 12):v + 13, max(0, u - 12):u + 13])
        if len(ys) == 0:
            return f'nothing taller than {OBJ_MIN*1000:.0f} mm under the click (table z here {table*1000:.0f} mm)'
        j = np.argmin((ys - 12) ** 2 + (xs - 12) ** 2)
        k = lab[max(0, v - 12) + ys[j], max(0, u - 12) + xs[j]]
    return describe(P, lab == k, table)


def describe(P, blob, table):
    '''Size, centre and yaw of one object (blob: image mask of its pixels, table: table z in m).'''
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


def problem(obj):
    '''Why this object can't be picked (or None).'''
    r = np.hypot(*obj['centre']) * 1000
    if obj['width'] > MAX_WIDTH:
        return f'too wide ({obj["width"]*1000:.0f} mm)'
    if obj['height'] < MIN_HEIGHT:
        return f'too flat ({obj["height"]*1000:.0f} mm)'
    if not 120 <= r <= 340:
        return f'out of reach ({r:.0f} mm from base)'
    return None


_PHYS = {}


def physics_check(plan, obj, place):
    '''Run the plan in MuJoCo physics with a box of the measured size and yaw.
    The scene is loaded once (loading takes 0.3 s, the simulation itself ~0.05 s).'''
    if 'model' not in _PHYS:
        _PHYS['model'] = mujoco.MjModel.from_xml_path(BOX_SCENE)
    model = _PHYS['model']
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
    ap.add_argument('--detect', metavar='NAMES', help='comma-separated object names for YOLO-World')
    ap.add_argument('--backend', choices=['depth', 'yolo'], default='depth',
                    help='depth = depth blobs + CLIP names (default), yolo = YOLO-World boxes')
    ap.add_argument('--conf', type=float, default=0.15, help='YOLO-World confidence (default 0.15)')
    ap.add_argument('--calib', default=os.path.join(HERE, 'T_base_cam.npy'))
    ap.add_argument('--port', default=so101_real.PORT)
    ap.add_argument('--robot-id', default=so101_real.ROBOT_ID)
    a = ap.parse_args()
    so101_real.configure(a.port, a.robot_id)
    if not os.path.exists(a.calib):
        sys.exit(f'{a.calib} not found: run table_calib.py first (SETUP_TABLE_2D_CALIB.md).')
    T, offset = np.load(a.calib), load_offset()

    import pyrealsense2 as rs
    detector = None
    if a.detect:
        from detector import Detector
        print(f'loading the detector ({a.backend})...')
        detector = Detector(a.detect.split(','), backend=a.backend, conf=a.conf)
        print('looking for:', ', '.join(detector.names))
    model = mujoco.MjModel.from_xml_path(BOX_SCENE)
    d_mask, d_click = mujoco.MjData(model), mujoco.MjData(model)   # one per thread (arm mask)
    pipe, _, _, intr = open_camera(depth=True)
    align = rs.align(rs.stream.color)
    a.speed = float(np.clip(a.speed, 0.1, 2.0))
    arm = so101_real.RealArm(dry=a.dry, max_step_deg=joint_step_for(a.speed))
    print(__doc__.split('In the camera window:')[1].split('Needs')[0])

    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    logf = open(LOG, 'a')
    logf.write(f'\n=== {time.strftime("%Y-%m-%d %H:%M:%S")} ===\n')
    st = {'click': None, 'obj': None, 'place': None, 'place_uv': None,
          'msg': 'press an object number 1-9 (or click an object)' if detector else 'click the OBJECT (its top)',
          'stop': False, 'thread': None, 'job': None, 'dets': [], 'quit': False,
          'plan_job': None, 'go': False}

    def detect_loop():
        '''Background: YOLO-World on the newest frame, then measure every box from depth.
        Paused while the arm moves (keeps the 30 Hz arm commands on time; the arm hides objects).'''
        while not st['quit']:
            job, st['job'] = st['job'], None
            if job is None or busy() or planning():          # planning gets the CPU
                time.sleep(0.03)
                continue
            img, z_raw, units, q = job
            P = cloud_from(z_raw, units, intr, T, offset)
            robot = robot_mask(model, d_mask, q, T, offset, intr, img.shape[:2])
            out = []
            for dd in detector.detect(img, P, robot):
                if dd['blob'] is not None:
                    m = describe(P, dd['blob'], dd['table'])
                else:
                    m = measure(P, 0, 0, roi=dd['box'], robot=robot)
                dd['meas'] = m if isinstance(m, dict) else None
                dd['why'] = m if isinstance(m, str) else problem(m)
                out.append(dd)
            st['dets'] = out[:9]

    def choose(i):
        if i >= len(st['dets']):
            say(f'no object {i + 1}')
            return
        dd = st['dets'][i]
        if dd['why']:
            say(f'{i + 1} {dd["name"]}: {dd["why"]}')
            return
        r = dict(dd['meas'], name=dd['name'])
        c = r['centre'] * 1000
        st['obj'] = r
        say(f'{i + 1} {dd["name"]} at x {c[0]:.0f} y {c[1]:.0f} mm: height {r["height"]*1000:.0f}, '
            f'width {r["width"]*1000:.0f}, yaw {r["yaw"]:.0f} deg  -> click the PLACE spot')

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

    def plan_job(job):
        '''Background thread: plan + sim check; if that fails, grip higher (lower 20 -> 10 -> 0 mm).
        Starts as soon as the place spot is clicked, so SPACE usually finds it ready.'''
        t0 = time.time()
        obj, place = job['obj'], job['place']
        why = ''
        for lower in [a.lower] + [x for x in (10.0, 0.0) if x < a.lower]:
            try:
                plan, labels = build_plan(model, obj['centre'], place, obj['height'], obj['width'],
                                          obj['yaw'], job['q'], lower / 1000)
                plan, labels = speed_up(plan, labels, a.speed)
            except SystemExit as e:
                why = f'cannot plan: {e}'
                continue
            ok, msg = physics_check(plan, obj, place)
            if ok:
                job.update(plan=plan, labels=labels, lower=lower, msg=msg)
                break
            why = f'sim check FAILED: {msg}'
            if msg.startswith('lifted') and float(msg.split()[1]) < 20:   # never got hold of it
                why += ' (cannot grab it from the object side: from the object)'
        else:
            job['why'] = why
        job['secs'] = time.time() - t0
        job['done'] = True

    def start_planning(go=False):
        job = {'obj': st['obj'], 'place': st['place'].copy(), 'q': arm.read_sim(), 'done': False,
               'reported': False}
        st.update(plan_job=job, go=go)
        threading.Thread(target=plan_job, args=(job,), daemon=True).start()

    def planning():
        return st['plan_job'] is not None and not st['plan_job']['done']

    def report(job):
        '''Called once when a background plan finishes.'''
        job['reported'] = True
        if 'why' in job:
            say(job['why'])
            obj = job['obj']
            r_obj = np.hypot(*obj['centre']) * 1000
            r_axis = np.hypot(obj['centre'][0] - 0.039, obj['centre'][1]) * 1000   # from the turning axis
            if 'from the object' in job['why'] or 'Cannot reach [' in job['why'] and r_obj < 150:
                say(f'NOT possible: the gripper cannot take the OBJECT where it is ({r_axis:.0f} mm from the '
                    f'base axis). Move it to 130-300 mm, then r and choose it again')
            else:
                say('NOT possible from here: click another PLACE spot (object kept), or r for another object')
            st.update(place=None, place_uv=None, plan_job=None, go=False)
            return
        if job['lower'] != a.lower:
            say(f'note: gripping only {job["lower"]:.0f} mm lower (not {a.lower:.0f}): lower failed here')
        say(f'plan ready in {job["secs"]:.1f} s, sim check OK: {job["msg"]}'
            + ('' if st['go'] else '  -> SPACE to go'))

    def launch():
        job = st['plan_job']
        if np.degrees(np.abs(arm.read_sim()[:5] - job['q'][:5])).max() > 2:    # arm moved since: replan
            say('arm moved since planning: planning again...')
            start_planning(go=True)
            return
        say(f'running ({len(job["plan"]) / so101_real.RATE:.0f} s at speed {a.speed}), s = stop')
        st['thread'] = threading.Thread(target=play, args=(job['plan'], job['labels'], 1.0), daemon=True)
        st['thread'].start()
        st.update(obj=None, place=None, place_uv=None, plan_job=None, go=False)

    win = ('camera_pick  (' + ('1-9 object' if detector else 'click object')
           + ', click place, SPACE go | s stop | r reset | h home | q quit)')
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    det_thread = None
    if detector is not None:
        det_thread = threading.Thread(target=detect_loop, daemon=True)
        det_thread.start()
    try:
        while True:
            ok, fs = pipe.try_wait_for_frames(2000)
            if not ok:
                say('camera sent no frame for 2 s: replug it if this repeats')
                continue
            frames = align.process(fs)
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            img = np.asanyarray(color.get_data()).copy()
            if detector is not None and st['job'] is None and not busy():
                st['job'] = (img.copy(), np.asanyarray(depth.get_data()).copy(), depth.get_units(), arm.read_sim())

            if st['click'] is not None and not busy() and detector is not None and st['obj'] is None:
                u, v = st['click']                               # click inside a box = choose it
                for i, dd in enumerate(st['dets']):
                    x0, y0, x1, y1 = dd['box']
                    if x0 <= u <= x1 and y0 <= v <= y1:
                        st['click'] = None
                        choose(i)
                        break

            if st['click'] is not None and planning():         # one plan at a time
                st['click'] = None
                say('still planning: wait a moment, then click again')
            if st['click'] is not None and not busy():
                u, v = st['click']
                st['click'] = None
                P = cloud_base(depth, intr, T, offset)
                if st['obj'] is None:
                    r = measure(P, u, v, robot=robot_mask(model, d_click, arm.read_sim(), T, offset, intr,
                                                          img.shape[:2]))
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
                        r_axis = np.hypot(p[0] - 0.039, p[1]) * 1000
                        say(f'place at x {p[0]*1000:.0f} y {p[1]*1000:.0f} mm: planning...  (SPACE to go)'
                            + (f'  NOTE: only {r_axis:.0f} mm from the base axis: picking it up again from'
                               ' there may not work' if r_axis < 120 else ''))
                        start_planning()

            pj = st['plan_job']
            if pj is not None and pj['done'] and not pj['reported']:
                report(pj)
            pj = st['plan_job']
            if pj is not None and pj['done'] and st['go'] and not busy():
                launch()

            if detector is not None and st['obj'] is None:
                for i, dd in enumerate(st['dets']):
                    x0, y0, x1, y1 = dd['box']
                    col = (0, 200, 0) if not dd['why'] else (0, 0, 230)
                    cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
                    m = dd['meas']
                    text = f'{i + 1} {dd["name"]} {dd["conf"]:.2f}'
                    if m is not None:
                        text += f'  h{m["height"]*1000:.0f} w{m["width"]*1000:.0f} {m["yaw"]:.0f}deg'
                        cv2.drawContours(img, [m['contour']], -1, col, 1)
                    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                    ty = max(44, y0 - 6)
                    cv2.rectangle(img, (x0, ty - th - 4), (x0 + tw + 4, ty + 3), (0, 0, 0), -1)
                    cv2.putText(img, text, (x0 + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
                    if dd['why']:
                        (tw, th), _ = cv2.getTextSize(dd['why'], cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                        cv2.rectangle(img, (x0, y1 + 2), (x0 + tw + 4, y1 + th + 8), (0, 0, 0), -1)
                        cv2.putText(img, dd['why'], (x0 + 2, y1 + th + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
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
            elif detector is not None and ord('1') <= key <= ord('9') and not busy():
                if st['obj'] is not None:
                    say('object already chosen: click the PLACE spot, or r to choose again')
                else:
                    choose(key - ord('1'))
            elif key == ord('s') and busy():
                st['stop'] = True
            elif key == ord('r') and not busy():
                st.update(obj=None, place=None, place_uv=None, plan_job=None, go=False)
                say('reset: ' + ('press an object number 1-9' if detector else 'click the OBJECT (its top)'))
            elif key == ord('h') and not busy():
                q0 = arm.read_sim()
                q1 = np.append(HOME, q0[5])
                run_path([q0 + smooth(k / 90) * (q1 - q0) for k in range(1, 91)])
                say('going home...')
            elif key == ord(' ') and not busy():
                if st['obj'] is None or st['place'] is None:
                    say('click the object, then the place spot, then SPACE')
                elif st['plan_job'] is None:
                    start_planning(go=True)
                    say('planning... starts by itself when ready')
                elif planning():
                    st['go'] = True
                    say('still planning... starts by itself when ready')
                else:
                    launch()
    except KeyboardInterrupt:
        st['stop'] = True
    finally:
        st['quit'] = True
        if det_thread is not None:
            det_thread.join(timeout=5)              # torch must finish before Python exits
        if st['thread'] is not None:
            st['thread'].join(timeout=2)
        pipe.stop()
        cv2.destroyAllWindows()
        arm.close()


if __name__ == '__main__':
    main()
