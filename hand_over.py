'''Hand-over: hold out your open hand, the arm picks a chosen object and drops it into your palm.

Loop (fully automatic once started with SPACE):
  1. waits until it sees the TARGET object on the table AND an open hand held still (1 s) within reach
  2. picks the object (plan + MuJoCo physics check first, same planner as camera_pick.py)
  3. holds it up, looks for your hand again (fresh position), moves above the palm, lowers slowly,
     opens the gripper ~3 cm above your palm, goes up and back HOME
  4. waits until you take your hand away, then starts again from 1 when it sees the object + hand
  If your hand moves away (> 5 cm) while the arm comes down, the arm stops, holds the object and
  aims again. No hand for 30 s while holding: it puts the object back where it was.

Which object: reference photo(s) of it, matched with CLIP image features (offline, CPU):
  python hand_over.py --ref targets/cube.jpg          one or more photos (or a folder)
  python hand_over.py                                 every photo in targets/ (or teach it: key t)
Several objects, in PRIORITY order: start the file names with a number. Photos with the same number
are the same object:  targets/1_green_cube.jpg  targets/2_white_cube.jpg  (targets/2_white_cube_b.jpg)
-> it hands over the green cube while one is on the table, the white one only when no green one is
(or the green one cannot be picked where it is).
A box counts as an object only when its similarity is >= --sim AND higher (by 0.03) than how
similar that photo is to crops of the bare table in the same frame (shown at the bottom as 'table:'),
so white letters on the mat do not pass for a white cube. Flat blobs (< 12 mm: depth noise, cables)
are ignored. Boxes that match nothing show '? 0.xx'.
Teach from the camera (best match, same camera and light): press t, then click the object's box.
The crop is saved in targets/ as the LOWEST priority (N_taught_<time>.jpg) and used from then on.

Run:  python hand_over.py            real arm      --dry  fake arm (camera, detection, plans only)
      --sim 0.80  similarity needed to call a box the target (shown on each box; +/- keys change it)
      --speed 1.0 pick speed (the hand part always runs at most at 1.0)   --lower 20 (as camera_pick)
Keys (click the window first):
  SPACE start / pause the automatic loop (starts PAUSED)    s  stop the arm now (and pause)
  t  teach: click an object box to use it as the target     x  forget the taught photos (not --ref ones)
  + / -  similarity threshold     h  home     q  quit
Hand detection: MediaPipe Hands (models/hand_landmarker.task). Palm 3D position from the depth camera.
SAFETY: the arm moves to a person. Keep the speed low, keep the other hand on 's' the first times,
hold the hand flat and still, and keep your face away from the arm.
Everything is written to debug/hand_over_log.txt.
'''
import argparse
import glob
import re
import os
import sys
import threading
import time
import numpy as np
import cv2
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "Reference"))
sys.path.insert(0, HERE)
import so101_real                                                    # noqa: E402
from so101_real import HOME, RATE                                    # noqa: E402
from keyboard_control import ik, tcp, pitch_of                       # noqa: E402
from pick_place import (build_plan, smooth, speed_up, joint_step_for, BOX_SCENE, PITCH, SEEDS,   # noqa: E402
                        GRIP_CLOSED, LIFT, PAN_AXIS, grip_open_for, lowest_tip)
from camera_pick import cloud_from, describe, problem, physics_check, MIN_HEIGHT  # noqa: E402
from detector import robot_mask, depth_blobs                         # noqa: E402
from verify_click import load_offset                                 # noqa: E402
from handeye_calib import open_camera                                # noqa: E402

HAND_MODEL = os.path.join(HERE, 'models', 'hand_landmarker.task')
TARGET_DIR = os.path.join(HERE, 'targets')
LOG = os.path.join(HERE, 'debug', 'hand_over_log.txt')
STILL_S = 1.0            # s the palm must stay still before the arm goes
STILL_M = 0.02           # m: "still" = every palm position of the last STILL_S within this
HAND_R = (0.14, 0.31)    # m from the turning axis: palm must be in this ring
HAND_Z = (-0.01, 0.11)   # m above the table (higher: the arm cannot point down above it)
GAP = 0.03               # m: object bottom this far above the palm when the gripper opens
APPROACH = 0.06          # m: comes down this last bit straight, slowly
MOVED = 0.05             # m: palm moved this far from the aim while coming down -> stop, aim again
HOLD_TIMEOUT = 30.0      # s holding with no hand -> put the object back
GONE_S = 1.0             # s with no hand seen = hand taken away (next round may start)
FINGERS = [(8, 6), (12, 10), (16, 14), (20, 18)]    # (tip, middle joint) MediaPipe landmark ids
PALM = [0, 1, 5, 9, 13, 17]
BG_EVERY = 4.0           # s: re-sample crops of the bare table (what a photo must beat)
BG_CROPS = 8
BG_MARGIN = 0.03         # a box must be this much MORE similar to the photo than the bare table is


# ---------------------------------------------------------------- hand ------
class Hands:
    '''MediaPipe hand landmarks (VIDEO mode: tracks between frames, ~15 ms on the CPU).'''

    def __init__(self):
        import mediapipe as mp
        from mediapipe.tasks.python import vision, BaseOptions
        if not os.path.exists(HAND_MODEL):
            sys.exit(f'{HAND_MODEL} not found (MediaPipe hand model).')
        self.mp = mp
        self.lm = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=HAND_MODEL), num_hands=2,
            running_mode=vision.RunningMode.VIDEO, min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5, min_tracking_confidence=0.5))
        self.ts = 0

    def detect(self, bgr):
        '''List of hands: {'pts': 21 x 2 pixels, 'open': bool, 'fingers': n extended}.'''
        self.ts = max(self.ts + 1, int(time.monotonic() * 1000))
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=np.ascontiguousarray(bgr[:, :, ::-1]))
        r = self.lm.detect_for_video(img, self.ts)
        h, w = bgr.shape[:2]
        out = []
        for lms in r.hand_landmarks:
            pts = np.array([[p.x * w, p.y * h] for p in lms])
            wrist = pts[0]
            n = sum(np.linalg.norm(pts[t] - wrist) > 1.15 * np.linalg.norm(pts[m] - wrist) for t, m in FINGERS)
            out.append({'pts': pts, 'fingers': int(n), 'open': n >= 4})
        return out

    def close(self):
        try:
            self.lm.close()
        except Exception:
            pass


def pixels_to_base(z_raw, units, intr, T, offset, us, vs):
    '''Base-frame points (N x 3, m) for the given pixels; pixels without depth are dropped.'''
    z = z_raw[vs, us].astype(np.float64) * units
    ok = z > 0
    us, vs, z = us[ok], vs[ok], z[ok]
    p = np.stack([(us - intr.ppx) / intr.fx * z, (vs - intr.ppy) / intr.fy * z, z], 1)
    return p @ T[:3, :3].T + T[:3, 3] + offset


def palm_point(hand, z_raw, units, intr, T, offset):
    '''3D palm centre (m, base frame) from the depth inside the palm polygon, or None.'''
    poly = hand['pts'][PALM].astype(np.int32)
    h, w = z_raw.shape
    mask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(mask, cv2.convexHull(poly), 1)
    vs, us = np.nonzero(mask)
    if len(us) < 30:
        return None
    P = pixels_to_base(z_raw, units, intr, T, offset, us, vs)
    if len(P) < 30:
        return None
    m = np.median(P, 0)
    P = P[np.abs(P[:, 2] - m[2]) < 0.03]           # drop fingers / table seen through the gaps
    return np.median(P, 0) if len(P) >= 20 else None


def hand_mask(hands, shape, grow=30):
    '''Image mask of every hand (convex hull of its landmarks, grown): never an object.'''
    m = np.zeros(shape, np.uint8)
    for hd in hands:
        cv2.fillConvexPoly(m, cv2.convexHull(hd['pts'].astype(np.int32)), 1)
    if hands:
        m = cv2.dilate(m, np.ones((grow * 2 + 1, grow * 2 + 1), np.uint8))
    return m.astype(bool)


def in_reach(p):
    r = np.hypot(p[0] - PAN_AXIS[0], p[1] - PAN_AXIS[1])
    if not HAND_R[0] <= r <= HAND_R[1]:
        return f'hand {r*1000:.0f} mm from the base: hold it {HAND_R[0]*1000:.0f}-{HAND_R[1]*1000:.0f} mm away'
    if not HAND_Z[0] <= p[2] <= HAND_Z[1]:
        return f'hand {p[2]*1000:.0f} mm above the table: hold it lower (under {HAND_Z[1]*1000:.0f} mm)'
    return None


# -------------------------------------------------------------- target ------
class Target:
    '''The objects to hand over, in PRIORITY order, each from one or more reference photos (CLIP image
    features, cosine similarity). A file name starting with a number sets the priority and groups photos
    of the same object: 1_green_cube.jpg, 1_green_cube_b.jpg -> object 1; 2_white_cube.jpg -> object 2.
    Files without a number come after the numbered ones (each its own object, in the order given).'''

    def __init__(self, refs):
        import torch
        import clip
        self.torch = torch
        self.clip, self.pre = clip.load('ViT-B/32', device='cpu')        # ~/.cache/clip
        self.objs = []          # [{'key', 'prio', 'name', 'feats': [...], 'paths': [...]}] sorted by prio
        for p in refs:
            img = cv2.imread(p)
            if img is None:
                print(f'cannot read {p}: skipped')
                continue
            self.add(img, p)

    def embed(self, bgr):
        from PIL import Image
        with self.torch.no_grad():
            f = self.clip.encode_image(self.pre(Image.fromarray(np.ascontiguousarray(bgr[:, :, ::-1])))
                                       .unsqueeze(0)).float()[0].numpy()
        return f / np.linalg.norm(f)

    def add(self, bgr, path):
        stem = os.path.splitext(os.path.basename(path))[0]
        m = re.match(r'(\d+)[_\- ]*(.*)', stem)
        if m:
            key, prio, name = f'#{int(m.group(1))}', int(m.group(1)), m.group(2) or stem
        else:
            key, prio, name = stem, 1000 + len(self.objs), stem
        o = next((o for o in self.objs if o['key'] == key), None)
        if o is None:
            o = {'key': key, 'prio': prio, 'name': name, 'feats': [], 'paths': []}
            self.objs.append(o)
            self.objs.sort(key=lambda o: o['prio'])
        o['feats'].append(self.embed(bgr))
        o['paths'].append(path)

    @property
    def n(self):
        return sum(len(o['feats']) for o in self.objs)

    def names(self):
        return [o['name'] for o in self.objs]

    def forget_taught(self):
        self.objs = [o for o in self.objs if not o['name'].startswith('taught_')]

    def describe(self):
        return ''.join(f"\n  priority {i + 1}: {o['name']} ({len(o['feats'])} photo{'s' * (len(o['feats']) > 1)})"
                       for i, o in enumerate(self.objs))

    @staticmethod
    def crop(bgr, box):
        '''Square crop around a box with some context (same as detector.py).'''
        x0, y0, x1, y1 = box
        s = int(max(x1 - x0, y1 - y0) * 1.6) + 16
        cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
        return bgr[max(0, cy - s // 2):cy + s // 2, max(0, cx - s // 2):cx + s // 2]

    def score(self, bgr, box):
        '''Similarity of the box to each object (best photo of each), in priority order.'''
        if not self.objs:
            return np.zeros(0)
        f = self.embed(self.crop(bgr, box))
        return np.array([max(np.array(o['feats']) @ f) for o in self.objs])

    def background(self, crops):
        '''How similar each object's photos are to crops of the BARE table (max), in priority order.
        A photo of a white cube on the dark mat is ~0.8 similar to the white letters on the mat:
        a box only counts as that object when it beats this by BG_MARGIN.'''
        if not self.objs or not crops:
            return np.zeros(len(self.objs))
        B = np.array([self.embed(c) for c in crops])
        return np.array([(np.array(o['feats']) @ B.T).max() for o in self.objs])


def table_crops(img, P, table, avoid, n=BG_CROPS, half=28, rng=np.random.default_rng()):
    '''n crops of the bare table in the reach ring (nothing sticking out, not the arm or a hand).'''
    r = np.hypot(P[..., 0], P[..., 1])
    ok = (np.isfinite(P[..., 2]) & (r > 0.12) & (r < 0.35) & (np.abs(P[..., 2] - table) < 0.004)
          & ~cv2.dilate(avoid.astype(np.uint8), np.ones((61, 61), np.uint8)).astype(bool))
    ok[:half] = ok[-half:] = False
    ok[:, :half] = ok[:, -half:] = False
    ys, xs = np.nonzero(ok)
    if len(xs) < 50:
        return []
    pick = rng.choice(len(xs), min(n, len(xs)), replace=False)
    return [Target.crop(img, (xs[i] - half, ys[i] - half, xs[i] + half, ys[i] + half)) for i in pick]


# ------------------------------------------------------------- planning -----
def plan_delivery(model, q_now, target, g_hold, g_open, slow=1.0, min_tip=0.01):
    '''From q_now (holding the object) to the grip-centre point `target` (m): joint move to
    APPROACH above it, straight down, open, straight up, home. Gripper as straight down as the
    arm allows there. Returns (plan, labels, frame where it opens).'''
    d = mujoco.MjData(model)
    lo, hi = so101_real.real_limits_sim()
    lo = np.maximum(model.actuator_ctrlrange[:5, 0], lo)
    hi = np.minimum(model.actuator_ctrlrange[:5, 1], hi)
    above = target + [0, 0, APPROACH]
    n_down = int(2.0 * RATE / slow)
    best = None
    for tilt in (PITCH, 70, 60, 50, 40):
        for s in SEEDS:
            seed = np.array(q_now, dtype=float)
            seed[0] = -np.arctan2(above[1], above[0])
            seed[1:4] = np.radians(s)
            qa, err = ik(model, d, seed, above, tilt, True, lo, hi, iters=400)
            if err > 0.004 or abs(pitch_of(tcp(model, d, qa)[1]) - tilt) > 3:
                continue
            down, qq, ok = [], qa.copy(), True
            for k in range(1, n_down + 1):
                qq, err = ik(model, d, qq, above + smooth(k / n_down) * (target - above), tilt, True, lo, hi)
                if err > 0.004:
                    ok = False
                    break
                down.append(qq.copy())
            if not ok:
                continue
            jump = np.degrees(np.abs(np.diff(np.vstack([qa, down])[:, :5], axis=0))).max()
            score = jump + np.degrees(np.abs(qa[:4] - q_now[:4])).max() / 30
            if best is None or score < best[0]:
                best = (score, qa, down, tilt)
            if jump < 3:
                break
        if best is not None:
            break
    if best is None:
        raise SystemExit(f'cannot reach {np.round(target * 1000)} mm with the gripper pointing down: '
                         'hold the hand closer to the robot / lower')
    _, qa, down, tilt = best
    plan, labels = [], []

    def blend(q0, q1, name, secs):
        labels.append((len(plan), name))
        n = max(1, int(secs * RATE / slow))
        plan.extend(q0 + smooth(k / n) * (q1 - q0) for k in range(1, n + 1))

    q0 = np.array(q_now, dtype=float)
    q_above = np.append(qa[:5], g_hold)
    blend(q0, q_above, f'move above your hand (tilt {tilt:.0f} deg)',
          max(1.5, np.degrees(np.abs(q_above[:5] - q0[:5])).max() / 40))
    labels.append((len(plan), 'lower to your hand'))
    down = [np.append(q[:5], g_hold) for q in down]
    plan += down
    i_open = len(plan)
    blend(down[-1], np.append(down[-1][:5], g_open), 'open gripper', 1.0)
    labels.append((len(plan), 'up'))
    plan += [np.append(q[:5], g_open) for q in down[::-1]] + [np.append(qa[:5], g_open)]
    q_end = plan[-1]
    blend(q_end, np.append(HOME, g_open), 'return home', 3.0)
    plan = np.array(plan)
    tips = min(lowest_tip(model, d, q) for q in plan[::3])
    if tips < min_tip:
        raise SystemExit(f'path would bring a fingertip {tips*1000:.0f} mm from the table')
    return plan, labels, i_open


# ----------------------------------------------------------------- main -----
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ref', nargs='*', default=None, help='photo(s) or folder of the object (default: targets/)')
    ap.add_argument('--sim', type=float, default=0.80, help='CLIP similarity needed (default 0.80)')
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--lower', type=float, default=20)
    ap.add_argument('--calib', default=os.path.join(HERE, 'T_base_cam.npy'))
    ap.add_argument('--port', default=so101_real.PORT)
    ap.add_argument('--robot-id', default=so101_real.ROBOT_ID)
    a = ap.parse_args()
    so101_real.configure(a.port, a.robot_id)
    if not os.path.exists(a.calib):
        sys.exit(f'{a.calib} not found: run table_calib.py first (docs/SETUP_TABLE_2D_CALIB.md).')
    T, offset = np.load(a.calib), load_offset()
    a.speed = float(np.clip(a.speed, 0.1, 2.0))
    slow = min(a.speed, 1.0)

    refs = []
    for r in (a.ref if a.ref is not None else [TARGET_DIR]):
        refs += sorted(glob.glob(os.path.join(r, '*.*'))) if os.path.isdir(r) else [r]
    refs = [r for r in refs if r.lower().endswith(('.jpg', '.jpeg', '.png', '.webp', '.bmp'))]
    print('loading CLIP and MediaPipe...')
    target = Target(refs)
    hands_det = Hands()
    print(f'objects to hand over ({target.n} photos):' + target.describe())

    import pyrealsense2 as rs
    model = mujoco.MjModel.from_xml_path(BOX_SCENE)
    d_mask, d_main = mujoco.MjData(model), mujoco.MjData(model)     # one per thread
    try:
        pipe, _, _, intr = open_camera(depth=True)
        align = rs.align(rs.stream.color)
        arm = so101_real.RealArm(dry=a.dry, max_step_deg=joint_step_for(a.speed))
    except BaseException:
        hands_det.close()
        raise
    print(__doc__.split('Keys')[1].split('Hand detection')[0])

    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    os.makedirs(TARGET_DIR, exist_ok=True)
    logf = open(LOG, 'a')
    logf.write(f'\n=== {time.strftime("%Y-%m-%d %H:%M:%S")} ===\n')
    st = {'state': 'paused', 'msg': '', 'objs': [], 'objs_t': 0.0, 'job': None, 'quit': False,
          'thread': None, 'stop': False, 'frame': 0, 'result': None, 'click': None, 'teach': False,
          'work': None, 'obj': None, 'z_grip': None, 'aim': None, 'i_open': 0, 'hold_t': 0.0,
          'last_hand_t': 0.0, 'retry_t': 0.0, 'palm_hist': [], 'sim': a.sim, 'paused_holding': False,
          'deliver': None, 'putback': False, 'pick': None,
          'bg': np.zeros(0), 'bg_t': 0.0}

    def say(text):
        st['msg'] = text
        print(text)
        logf.write(time.strftime('%H:%M:%S ') + text + '\n')
        logf.flush()

    def busy():
        return st['thread'] is not None and st['thread'].is_alive()

    def working():
        return st['work'] is not None and st['work'].is_alive()

    def play(plan, labels):
        pending, t = sorted(labels), time.perf_counter()
        st['frame'] = 0
        for k, q in enumerate(plan):
            if st['stop']:
                break
            while pending and pending[0][0] <= k:
                say(f'> {pending.pop(0)[1]}')
            st['frame'] = k
            arm.send_sim(q)
            t += 1 / RATE
            time.sleep(max(0.0, t - time.perf_counter()))
        st['result'] = 'stopped' if st['stop'] else 'done'
        st['stop'] = False

    def run(plan, labels):
        st['result'] = None
        st['thread'] = threading.Thread(target=play, args=(plan, labels), daemon=True)
        st['thread'].start()

    def detect_loop():
        '''Background: depth blobs (arm and hands masked out) -> size + CLIP similarity to the target.'''
        while not st['quit']:
            job, st['job'] = st['job'], None
            if job is None or busy() or working():
                time.sleep(0.03)
                continue
            img, z_raw, units, q, hmask = job
            P = cloud_from(z_raw, units, intr, T, offset)
            mask = robot_mask(model, d_mask, q, T, offset, intr, img.shape[:2]) | hmask
            out, blobs = [], depth_blobs(P, mask)
            if blobs and (time.time() - st['bg_t'] > BG_EVERY or len(st['bg']) != len(target.objs)):
                avoid = mask | np.any([b for b, _ in blobs], 0)
                st['bg'], st['bg_t'] = target.background(table_crops(img, P, blobs[0][1], avoid)), time.time()
            for blob, table in blobs:
                m = describe(P, blob, table, img)
                if isinstance(m, str) or m['height'] < MIN_HEIGHT:     # depth noise, cables, flat things:
                    continue                                          # not an object, not shown
                ys, xs = np.nonzero(blob)
                box = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
                sims = target.score(img, box)
                g = int(sims.argmax()) if len(sims) else -1          # the object this box looks most like
                bg = float(st['bg'][g]) if 0 <= g < len(st['bg']) else 0.0
                sim = float(sims[g]) if g >= 0 else 0.0
                out.append({'box': box, 'sims': sims, 'g': g, 'sim': sim, 'bg': bg,
                            'match': sim >= st['sim'] and sim >= bg + BG_MARGIN,
                            'name': target.objs[g]['name'] if 0 <= g < len(target.objs) else '?', 'meas': m,
                            'why': problem(m), 'crop': Target.crop(img, box)})
            st['objs'], st['objs_t'] = out, time.time()

    def the_target():
        '''Pickable object of the highest priority seen in the last 1.5 s (best match), or None.'''
        if time.time() - st['objs_t'] > 1.5:
            return None
        for g in range(len(target.objs)):                      # priority 1 first, then 2, ...
            ok = [o for o in st['objs'] if o['g'] == g and o['match'] and not o['why']]
            if ok:
                return max(ok, key=lambda o: o['sim'])
        return None

    def pick_job(obj, q):
        '''Background: pick-only plan + physics check (grip higher if the low grip fails).'''
        job = {'ok': False}
        for lower in [a.lower] + [x for x in (10.0, 0.0) if x < a.lower]:
            try:
                plan, labels = build_plan(model, obj['centre'], None, obj['height'], obj['width'],
                                          obj['yaw'], q, lower / 1000)
                plan, labels = speed_up(plan, labels, a.speed)
            except SystemExit as e:
                job['why'] = f'cannot plan the pick: {e}'
                continue
            ok, msg = physics_check(plan, obj, None)
            if ok:
                job.update(ok=True, plan=plan, labels=labels, msg=msg)
                break
            job['why'] = f'sim check of the pick FAILED: {msg}'
        st['pick'] = job

    def deliver_job(q, aim, min_tip=0.01):
        '''Background: plan to the palm (aim = grip-centre point, m); min_tip 3 mm = back on the table.'''
        try:
            plan, labels, i_open = plan_delivery(model, q, aim, GRIP_CLOSED, grip_open_for(st['obj']['width']),
                                                 slow, min_tip)
            st['deliver'] = {'ok': True, 'plan': plan, 'labels': labels, 'i_open': i_open}
        except SystemExit as e:
            st['deliver'] = {'ok': False, 'why': str(e)}

    def start(fn, *args):
        st['work'] = threading.Thread(target=fn, args=args, daemon=True)
        st['work'].start()

    def release_point(palm):
        '''Grip-centre point that leaves the object's bottom GAP above the palm.'''
        below = st['z_grip'] - st['obj']['table']            # grip centre above the object's bottom
        return np.array([palm[0], palm[1], palm[2] + GAP + below])

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            st['click'] = (x, y)

    win = 'hand_over  (SPACE start/pause | s stop | t teach | +/- sim | h home | q quit)'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    det_thread = threading.Thread(target=detect_loop, daemon=True)
    det_thread.start()
    say('PAUSED. ' + ('Press SPACE to start.' if target.n else
                      'No target photo yet: press t and click the object (or run with --ref photo.jpg).'))
    try:
        while True:
            ok, fs = pipe.try_wait_for_frames(2000)
            if not ok:
                say('camera sent no frame for 2 s: replug it if this repeats')
                continue
            frames = align.process(fs)
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            img = np.asanyarray(color.get_data()).copy()
            z_raw, units = np.asanyarray(depth.get_data()).copy(), depth.get_units()
            now = time.time()

            # ---- hands: palm 3D, open?, still? ----
            hands = hands_det.detect(img)
            if hands:
                st['last_hand_t'] = now
            ready, palm, hand_why = None, None, ''
            for hd in hands:
                hd['palm'] = palm_point(hd, z_raw, units, intr, T, offset) if hd['open'] else None
            open_hands = [hd for hd in hands if hd['palm'] is not None]
            if open_hands:
                hd = min(open_hands, key=lambda h: np.hypot(*(h['palm'][:2] - PAN_AXIS)))
                palm = hd['palm']
                hand_why = in_reach(palm) or ''
                hist = [(t, p) for t, p in st['palm_hist'] if now - t < STILL_S + 0.3]
                if hist and np.linalg.norm(hist[-1][1] - palm) > 0.06:      # another hand / jump: restart
                    hist = []
                hist.append((now, palm))
                st['palm_hist'] = hist
                recent = [p for t, p in hist if now - t <= STILL_S]
                still = (now - hist[0][0] >= STILL_S and
                         max(np.linalg.norm(p - palm) for p in recent) < STILL_M)
                hd['still'] = still
                if still and not hand_why:
                    ready = hd
            else:
                st['palm_hist'] = []

            # ---- objects (background, only while nothing moves) ----
            if st['job'] is None and not busy() and st['state'] in ('paused', 'wait', 'away'):
                st['job'] = (img.copy(), z_raw, units, arm.read_sim(), hand_mask(hands, img.shape[:2]))

            # ---- teach: click a box ----
            if st['click'] is not None:
                u, v = st['click']
                st['click'] = None
                if st['teach']:
                    hit = [o for o in st['objs'] if o['box'][0] <= u <= o['box'][2] and o['box'][1] <= v <= o['box'][3]]
                    if hit:
                        nxt = max([o['prio'] for o in target.objs if o['prio'] < 1000] + [0]) + 1     # lowest priority
                        p = os.path.join(TARGET_DIR, f'{nxt}_' + time.strftime('taught_%Y%m%d_%H%M%S.jpg'))
                        cv2.imwrite(p, hit[0]['crop'])
                        target.add(hit[0]['crop'], p)
                        st['teach'] = False
                        say(f'learned it as priority {nxt} ({os.path.basename(p)}; rename the file to change the priority)'
                            + ('. SPACE to start' if st['state'] == 'paused' else ''))
                    else:
                        say('click INSIDE an object box (boxes appear around everything on the table)')

            # ---- the automatic loop ----
            s = st['state']
            if s == 'wait' and not working() and now > st['retry_t']:
                obj = the_target()
                if obj is None:
                    st['msg'] = ('looking for the target object...' if target.n
                                 else 'no target photo: press t and click the object')
                elif ready is None:
                    st['msg'] = 'object found. Hold out your OPEN hand, flat and still' + (f' ({hand_why})' if hand_why else '')
                else:
                    st['obj'] = dict(obj['meas'])
                    st['pick'] = None
                    c = obj['meas']['centre'] * 1000
                    say(f'{obj["name"]} (priority {obj["g"] + 1}) at x {c[0]:.0f} y {c[1]:.0f} mm (similarity {obj["sim"]:.2f}), '
                        f'yaw {obj["meas"]["yaw"]:.0f} deg (depth alone {obj["meas"]["yaw_depth"]:.0f}, edges {obj["meas"]["yaw_edge"]:.2f}), '
                        f'width {obj["meas"]["width"]*1000:.0f} mm, hand ready: planning the pick...')
                    st['state'] = 'plan_pick'
                    start(pick_job, st['obj'], arm.read_sim())
            elif s == 'plan_pick' and not working():
                job = st['pick']
                if job['ok']:
                    say(f'pick OK in sim ({job["msg"]}): picking')
                    st['z_grip'] = tcp(model, d_main, job['plan'][-1])[0][2] - LIFT
                    run(job['plan'], job['labels'])
                    st['state'] = 'picking'
                else:
                    say(job['why'] + '  -> move the object (150-300 mm from the base); trying again in 5 s')
                    st.update(state='wait', retry_t=now + 5)
            elif s == 'picking' and not busy():
                if st['result'] == 'done':
                    st.update(state='holding', hold_t=now, deliver=None)
                    say('holding the object: hold out your open hand, flat and still')
                else:
                    st['state'] = 'paused'
            elif s == 'holding' and not busy() and not working():
                if st.get('deliver') is not None:                        # a delivery plan just finished
                    job, st['deliver'] = st['deliver'], None
                    if job['ok']:
                        run(job['plan'], job['labels'])
                        st.update(state='giving', i_open=job['i_open'])
                    else:
                        say(job['why'] + '  (trying again in 3 s)')
                        st['retry_t'] = now + 3
                elif ready is not None and now > st['retry_t']:
                    st['aim'] = ready['palm'].copy()
                    p = st['aim'] * 1000
                    say(f'hand at x {p[0]:.0f} y {p[1]:.0f} z {p[2]:.0f} mm: planning the hand-over...')
                    start(deliver_job, arm.read_sim(), release_point(st['aim']))
                elif now - st['hold_t'] > HOLD_TIMEOUT:
                    o = st['obj']
                    say(f'no hand for {HOLD_TIMEOUT:.0f} s: putting the object back')
                    start(deliver_job, arm.read_sim(), np.array([*o['centre'], st['z_grip'] + 0.01]), 0.003)
                    st['putback'] = True
                    st['state'] = 'putback'
                else:
                    left = HOLD_TIMEOUT - (now - st['hold_t'])
                    st['msg'] = (f'holding: open hand, flat and still please ({hand_why})' if hand_why
                                 else f'holding: show your open hand ({left:.0f} s before putting it back)')
            elif s == 'putback' and not working() and not busy():
                job, st['deliver'] = st['deliver'], None
                if job is not None and job['ok'] and st.get('putback'):
                    st['putback'] = False
                    run(job['plan'], job['labels'])
                elif job is not None and not job['ok']:
                    say(f'cannot put it back ({job["why"]}): still holding it. s / h, or show your hand')
                    st.update(state='holding', hold_t=now)
                else:
                    say('object put back. PAUSED (SPACE to start again)')
                    st['state'] = 'paused'
            elif s == 'giving':
                if busy() and st['frame'] < st['i_open'] and hands:      # watch the hand until it opens
                    moved = [np.linalg.norm(hd['palm'][:2] - st['aim'][:2]) for hd in hands if hd.get('palm') is not None]
                    if moved and min(moved) > MOVED:
                        st['stop'] = True
                        say(f'your hand moved {min(moved)*1000:.0f} mm: stopped, aiming again')
                if not busy():
                    if st['result'] == 'done':
                        say('done: take your hand away. Next round when the object and a hand are seen again')
                        st['state'] = 'away'
                    else:
                        st.update(state='holding', hold_t=now, deliver=None)
            elif s == 'away':
                if now - st['last_hand_t'] > GONE_S:
                    st['state'] = 'wait'
                    say('ready: looking for the object and your hand')

            # ---- draw ----
            if st['state'] in ('paused', 'wait', 'away', 'plan_pick'):
                tg = the_target()
                for o in st['objs']:
                    x0, y0, x1, y1 = o['box']
                    col = (0, 255, 0) if o is tg else ((0, 200, 255) if o['match'] else (160, 160, 160))
                    cv2.rectangle(img, (x0, y0), (x1, y1), col, 3 if o is tg else 1)
                    if not o['match']:                                # not one of the objects: no name
                        text = f'? {o["sim"]:.2f}'
                    else:
                        text = (f'{o["name"]} {o["sim"]:.2f}' + (' TARGET' if o is tg else '')
                                + (f' {o["why"]}' if o['why'] else ''))
                    cv2.putText(img, text, (x0, max(44, y0 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
            for hd in hands:
                col = (0, 255, 0) if hd is ready else ((0, 220, 255) if hd.get('palm') is not None else (150, 150, 150))
                for i, j in [(0, 5), (5, 9), (9, 13), (13, 17), (17, 0), (0, 1)] + [(t, m) for t, m in FINGERS]:
                    cv2.line(img, tuple(hd['pts'][i].astype(int)), tuple(hd['pts'][j].astype(int)), col, 2)
                c = hd['pts'][[0, 5, 9, 17]].mean(0).astype(int)
                label = 'OPEN' if hd['open'] else f'closed ({hd["fingers"]}/4 fingers)'
                if hd.get('palm') is not None:
                    p = hd['palm'] * 1000
                    label += f' z{p[2]:.0f}' + (' still' if hd.get('still') else '')
                cv2.putText(img, label, (int(c[0]) + 8, int(c[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
            if st['state'] in ('giving',) and st['aim'] is not None:
                cv2.putText(img, 'GIVING', (8, img.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
            cv2.putText(img, st['msg'][:110], (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            info = (f'{st["state"].upper()}  sim>={st["sim"]:.2f}  table: '
                    + ' '.join(f'{o["name"]} {b:.2f}' for o, b in zip(target.objs, st['bg']))
                    + f'  photos {target.n}') + ('  TEACH: click a box' if st['teach'] else '')
            cv2.putText(img, info, (8, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 255) if busy() else (255, 255, 255), 2)
            cv2.imshow(win, img)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                st['stop'] = True
                break
            elif key == ord('s'):
                if busy():
                    st['stop'] = True
                st['paused_holding'] = st['paused_holding'] or st['state'] in ('picking', 'holding', 'giving', 'putback')
                st.update(state='paused', deliver=None)
                say('stopped, PAUSED: ' + ('the arm holds the object (SPACE = continue to your hand, h = home)'
                                           if st['paused_holding'] else 'SPACE to start'))
            elif key == ord(' ') and not busy():
                if st['state'] == 'paused':
                    if st.get('paused_holding'):
                        st.update(state='holding', hold_t=time.time(), deliver=None, paused_holding=False)
                        say('continuing: show your open hand')
                    elif not target.n:
                        say('no target photo: press t and click the object first')
                    else:
                        st['state'] = 'away' if now - st['last_hand_t'] < GONE_S else 'wait'
                        say('started' + (': take your hand away first' if st['state'] == 'away' else ''))
                elif st['state'] in ('wait', 'away'):
                    st['state'] = 'paused'
                    say('PAUSED (SPACE to start)')
            elif key == ord('t'):
                st['teach'] = not st['teach']
                say('TEACH: click the box of the object to hand over' if st['teach'] else 'teach cancelled')
            elif key == ord('x'):
                target.forget_taught()
                say(f'taught photos forgotten (files stay in targets/); {target.n} photos left')
            elif key in (ord('+'), ord('=')):
                st['sim'] = min(0.99, st['sim'] + 0.02)
                say(f'similarity threshold {st["sim"]:.2f}')
            elif key in (ord('-'), ord('_')):
                st['sim'] = max(0.5, st['sim'] - 0.02)
                say(f'similarity threshold {st["sim"]:.2f}')
            elif key == ord('h') and not busy() and st['state'] == 'paused':
                q0 = arm.read_sim()
                q1 = np.append(HOME, q0[5])
                run([q0 + smooth(k / 90) * (q1 - q0) for k in range(1, 91)], [(0, 'home')])
                st['paused_holding'] = False
    except KeyboardInterrupt:
        st['stop'] = True
    finally:
        st['quit'] = True
        det_thread.join(timeout=5)
        if st['work'] is not None:
            st['work'].join(timeout=5)
        if st['thread'] is not None:
            st['thread'].join(timeout=2)
        pipe.stop()
        cv2.destroyAllWindows()
        hands_det.close()
        arm.close()


if __name__ == '__main__':
    main()
