'''Hand follow (relative): the arm copies your hand in front of the D415.

  hand moves          -> gripper tip moves the same way, 1:1 (like marker_follow.py)
  hand closes (fist)  -> gripper closes; hand opens -> gripper opens (in between = in between)
  palm turns (forearm twist, turning the way the palm faces) -> wrist_roll turns the same angle

Camera: looks the same way the arm points (as in marker_follow.py):
    hand further from the camera -> tip forward,  right in the image -> tip right,  up -> tip up
    --mirror flips left/right.
SPACE: the arm goes to START (tip 20 cm in front, 15 cm up, nose 45 deg down, gripper open), and
as soon as an OPEN hand is seen, that hand position and palm angle are the reference.
The tip follows the hand whether it is open or closed (so you can carry what it grips). Its 3D
position is taken at the heel of the palm, which a fist does not cover, so closing the hand does
not make the arm jump.
Out of reach: red frame, the tip stops at the edge (never below 1 cm). Hand lost: the arm holds.

Hand: MediaPipe Hands (models/hand_landmarker.task), one hand. Openness and palm angle come from
MediaPipe's 3D "world" landmarks (metres), so tilting the hand does not fool them.

Run:  python hand_follow.py --dry       camera + IK only, fake arm (check the directions first!)
      python hand_follow.py             real arm
      --mirror  --roll-sign -1 (wrist turns the other way)  --roll-scale 1.0  --no-roll
      --open 1.75 --fist 1.2  (finger-length ratio for fully open / fully closed, shown on screen)
      --start 200,0,150  --tilt 45  --scale 1.0  --speed 0.15  --mincutoff 1.0  --beta 4.0
Keys (click the image window first):
  SPACE  go to START and take the reference (again any time = re-zero)
  p      pause / resume (paused: the arm holds)     h  pause and go back to START
  q      quit (goes HOME first)
SAFETY: start with --dry. Real arm: slow, small moves; keep the other hand near 'p'.
'''
import argparse
import os
import sys
import time
import numpy as np
import cv2
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'Reference'))
sys.path.insert(0, HERE)
import so101_real                                                    # noqa: E402
from so101_real import HOME, SCENE, real_limits_sim                  # noqa: E402
from keyboard_control import ik, tcp, pitch_of                       # noqa: E402
from handeye_calib import open_camera                                # noqa: E402
from marker_follow import OneEuro, Reach, cam_to_robot, REACH_TOL, JUMP_M, HOME_TOL_DEG, JOINT_STEP  # noqa: E402

HAND_MODEL = os.path.join(HERE, 'models', 'hand_landmarker.task')
GRIP_OPEN, GRIP_CLOSED = 1.2, -0.17       # sim rad (closed = squeeze, force-limited, as pick_place.py)
LOST_S = 0.3                              # s without a hand -> "lost" (the arm already holds)
TIPS, MCPS = [8, 12, 16, 20], [5, 9, 13, 17]
BONES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
         (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]


# ---------------------------------------------------------------- hand ------
class Hand:
    '''One hand per frame: pixel landmarks (21 x 2) and 3D world landmarks (21 x 3, m), or None.'''

    def __init__(self):
        import mediapipe as mp
        from mediapipe.tasks.python import vision, BaseOptions
        if not os.path.exists(HAND_MODEL):
            sys.exit(f'{HAND_MODEL} not found (MediaPipe hand model).')
        self.mp = mp
        self.lm = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=HAND_MODEL), num_hands=1,
            running_mode=vision.RunningMode.VIDEO, min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5, min_tracking_confidence=0.5))
        self.ts = 0

    def detect(self, bgr):
        self.ts = max(self.ts + 1, int(time.monotonic() * 1000))
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=np.ascontiguousarray(bgr[:, :, ::-1]))
        r = self.lm.detect_for_video(img, self.ts)
        if not r.hand_landmarks:
            return None
        h, w = bgr.shape[:2]
        pts = np.array([[p.x * w, p.y * h] for p in r.hand_landmarks[0]])
        world = np.array([[p.x, p.y, p.z] for p in r.hand_world_landmarks[0]])
        return {'pts': pts, 'world': world}

    def close(self):
        try:
            self.lm.close()
        except Exception:
            pass


def finger_ratio(world):
    '''Mean wrist->fingertip length / mean wrist->knuckle length: ~1.9 open hand, ~1.0 fist.'''
    w = world[0]
    return float(np.mean([np.linalg.norm(world[t] - w) for t in TIPS]) /
                 np.mean([np.linalg.norm(world[m] - w) for m in MCPS]))


def palm_axes(world):
    '''u = wrist -> middle knuckle (along the fingers), a = index knuckle -> pinky knuckle, made
    perpendicular to u. Both unit vectors, camera axes.'''
    u = world[9] - world[0]
    u /= np.linalg.norm(u)
    a = world[17] - world[5]
    a -= np.dot(a, u) * u
    return u, a / np.linalg.norm(a)


def palm_turn(ref_a, u, a):
    '''Signed angle (rad) the palm turned about the finger axis u since the reference.'''
    r = ref_a - np.dot(ref_a, u) * u
    n = np.linalg.norm(r)
    if n < 1e-6:
        return 0.0
    r /= n
    return float(np.arctan2(np.dot(np.cross(r, a), u), np.dot(r, a)))


def heel_point(hand, z_raw, units, intr):
    '''3D point (camera frame, m) at the heel of the palm: 30 % of the way wrist -> middle knuckle,
    depth = median over a small disc there (a fist does not cover it). Or None.'''
    pts = hand['pts']
    c = pts[0] + 0.3 * (pts[9] - pts[0])
    r = max(4, int(0.15 * np.linalg.norm(pts[9] - pts[0])))
    h, w = z_raw.shape
    u0, v0 = int(round(c[0])), int(round(c[1]))
    if not (0 <= u0 < w and 0 <= v0 < h):
        return None
    mask = np.zeros(z_raw.shape, np.uint8)
    cv2.circle(mask, (u0, v0), r, 1, -1)
    z = z_raw[mask > 0].astype(np.float64) * units
    z = z[z > 0]
    if len(z) < 10:
        return None
    zm = float(np.median(z))
    return np.array([(c[0] - intr.ppx) / intr.fx * zm, (c[1] - intr.ppy) / intr.fy * zm, zm])


# ---------------------------------------------------------------- main ------
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--dry', action='store_true', help='fake arm: nothing moves')
    ap.add_argument('--start', default='200,0,150', help='START tip position x,y,z in mm (robot base)')
    ap.add_argument('--tilt', type=float, default=45.0, help='START tilt, deg below horizontal')
    ap.add_argument('--scale', type=float, default=1.0, help='tip moves scale x the hand move')
    ap.add_argument('--mirror', action='store_true', help='flip left/right')
    ap.add_argument('--speed', type=float, default=0.15, help='max tip speed, m/s')
    ap.add_argument('--mincutoff', type=float, default=1.0, help='filter: lower = smoother when still')
    ap.add_argument('--beta', type=float, default=4.0, help='filter: higher = less lag when moving fast')
    ap.add_argument('--open', type=float, default=1.75, help='finger ratio of a fully open hand')
    ap.add_argument('--fist', type=float, default=1.2, help='finger ratio of a closed hand')
    ap.add_argument('--roll-sign', type=float, default=1.0, choices=[1.0, -1.0])
    ap.add_argument('--roll-scale', type=float, default=1.0)
    ap.add_argument('--no-roll', action='store_true', help='keep the wrist roll fixed')
    a = ap.parse_args()

    import pyrealsense2 as rs
    model = mujoco.MjModel.from_xml_path(SCENE)
    d_fk = mujoco.MjData(model)
    r_lo, r_hi = real_limits_sim()
    lo = np.maximum(model.actuator_ctrlrange[:5, 0], r_lo)
    hi = np.minimum(model.actuator_ctrlrange[:5, 1], r_hi)

    q_rest = np.clip(HOME, lo, hi)                                 # the usual HOME (folded)
    start = np.array([float(v) for v in a.start.split(',')]) / 1000
    q_home, err = ik(model, d_fk, np.append(q_rest, 0), start, a.tilt, True, lo, hi, iters=300)
    q_home = q_home[:5]
    if err > REACH_TOL:
        sys.exit(f'START {a.start} mm with tilt {a.tilt} deg cannot be reached ({err*1000:.0f} mm off)')
    tip_home, axis = tcp(model, d_fk, q_home)
    reach = Reach(model, lo, hi, pitch_of(axis))

    print('loading MediaPipe...')
    hand_det = Hand()
    try:
        pipe, _, _, intr = open_camera(depth=True)
    except BaseException:
        hand_det.close()
        raise
    try:
        units = pipe.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        arm = so101_real.RealArm(dry=a.dry, max_step_deg=2.0)
    except BaseException:
        pipe.stop()
        hand_det.close()
        raise

    q_cmd = arm.read_sim()
    st = {'mode': 'homing', 'after_home': 'wait', 'ref': None, 'tip': tip_home.copy(),
          'last_seen': 0.0, 'clamped': False, 'note': '', 'grip': 1.0, 'turn': 0.0, 'ratio': None}
    goal = {'q': q_home}
    f_pos = OneEuro(a.mincutoff, a.beta)
    f_grip = OneEuro(2.0, 0.5)              # x[0] = openness 0..1
    f_turn = OneEuro(1.0, 0.5)              # x[0] = palm turn, rad
    print(__doc__.split('Keys')[1].split('SAFETY')[0])
    print(f'START tip: x{tip_home[0]*1000:.0f} y{tip_home[1]*1000:.0f} z{tip_home[2]*1000:.0f} mm, '
          f'tilt {reach.pitch:.0f} deg' + ('   [DRY: fake arm]' if a.dry else ''))

    def go_home(then):
        st['mode'], st['after_home'] = 'homing', then
        goal['q'] = q_home
        q_cmd[5] = GRIP_OPEN

    def step_to(q_goal):
        '''Move q_cmd toward q_goal, all joints together; True when the real arm is there.'''
        diff = q_goal - q_cmd[:5]
        m = np.degrees(np.abs(diff)).max()
        q_cmd[:5] += diff * (min(1.0, JOINT_STEP / m) if m > 0 else 1.0)
        arm.send_sim(q_cmd)
        return np.abs(arm.last_deg - np.clip(so101_real.sim_to_real(q_goal), arm.lo, arm.hi)).max() < HOME_TOL_DEG

    def reset_filters():
        f_pos.reset(), f_grip.reset(), f_turn.reset()

    go_home('wait')
    t_prev = time.perf_counter()
    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            cf, df = frames.get_color_frame(), frames.get_depth_frame()
            if not cf or not df:
                continue
            img = np.asanyarray(cf.get_data()).copy()
            z_raw = np.asanyarray(df.get_data())
            now = time.perf_counter()
            dt, t_prev = min(now - t_prev, 0.2), now

            # ---- hand ----
            hand = hand_det.detect(img)
            p = None
            if hand is not None:
                for i, j in BONES:
                    cv2.line(img, tuple(hand['pts'][i].astype(int)), tuple(hand['pts'][j].astype(int)), (0, 255, 0), 2)
                p_raw = heel_point(hand, z_raw, units, intr)
                if p_raw is not None and f_pos.x is not None and np.linalg.norm(p_raw - f_pos.x) > JUMP_M:
                    p_raw = None                                  # depth glitch: ignore this frame
                if p_raw is not None:
                    heel = hand['pts'][0] + 0.3 * (hand['pts'][9] - hand['pts'][0])
                    cv2.circle(img, (int(heel[0]), int(heel[1])), 8, (0, 0, 255), -1)
                    p, st['last_seen'] = f_pos(p_raw, now), now
                    st['ratio'] = finger_ratio(hand['world'])
                    g = np.clip((st['ratio'] - a.fist) / (a.open - a.fist), 0, 1)
                    st['grip'] = float(np.clip(f_grip(np.array([g, 0, 0]), now)[0], 0, 1))
                    u, ax = palm_axes(hand['world'])
                    if st['ref'] is not None:
                        st['turn'] = f_turn(np.array([palm_turn(st['ref']['a'], u, ax), 0, 0]), now)[0]
            if now - st['last_seen'] > LOST_S:
                reset_filters()
            seen = now - st['last_seen'] <= LOST_S

            # ---- arm ----
            if st['mode'] == 'homing':
                if step_to(goal['q']):
                    st['tip'] = tip_home.copy()
                    if st['after_home'] == 'ref':
                        if p is not None and st['grip'] > 0.7:        # open hand = reference
                            u, ax = palm_axes(hand['world'])
                            st['ref'] = {'p': p.copy(), 'a': ax}
                            st['turn'], st['mode'] = 0.0, 'follow'
                            f_turn.reset()
                            print('reference taken: following')
                    else:
                        st['mode'] = st['after_home']
            elif st['mode'] == 'follow' and p is not None:
                d_cam = p - st['ref']['p']
                target = tip_home + a.scale * cam_to_robot(d_cam, a.mirror)
                step = target - st['tip']                          # tip speed limit
                n = np.linalg.norm(step)
                if n > a.speed * dt:
                    target = st['tip'] + step * (a.speed * dt / n)
                if not a.no_roll:                                  # wrist roll first: IK sees it
                    q_cmd[4] = np.clip(q_home[4] + a.roll_sign * a.roll_scale * st['turn'], lo[4], hi[4])
                q_cmd[5] = GRIP_CLOSED + st['grip'] * (GRIP_OPEN - GRIP_CLOSED)
                q_new, aimed, clamped, note = reach.go(q_cmd, st['tip'], target)
                q_cmd[:4] = q_new[:4]
                st['tip'], st['note'], st['clamped'] = aimed, note, clamped
                arm.send_sim(q_cmd)
            else:
                arm.send_sim(q_cmd)                                # hold

            # ---- screen ----
            h, w = img.shape[:2]
            red = st['mode'] == 'follow' and st['clamped']
            if red:
                cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 0, 255), 14)
            lines = [f'mode: {st["mode"].upper()}' + ('  [DRY]' if a.dry else '') + ('   HAND LOST' if not seen else '')]
            if st['mode'] == 'wait':
                lines.append('SPACE = take reference and follow')
            if st['mode'] == 'homing' and st['after_home'] == 'ref':
                lines.append('show your OPEN hand to take the reference')
            if seen and st['ratio'] is not None:
                lines.append(f'hand {"OPEN" if st["grip"] > 0.5 else "CLOSED"}  open {st["grip"]*100:3.0f}%  '
                             f'(ratio {st["ratio"]:.2f})   palm turn {np.degrees(st["turn"]):+4.0f} deg')
            tip = st['tip'] - tip_home
            lines.append(f'tip  fwd {tip[0]*1000:+5.0f}  left {tip[1]*1000:+5.0f}  up {tip[2]*1000:+5.0f} mm'
                         + (f'  ({st["note"]})' if st['mode'] == 'follow' else ''))
            lines.append(f'wrist roll {np.degrees(q_cmd[4] - q_home[4]):+4.0f} deg   gripper '
                         f'{(q_cmd[5] - GRIP_CLOSED) / (GRIP_OPEN - GRIP_CLOSED) * 100:3.0f}% open')
            if red:
                lines.append('OUT OF REACH: tip stopped at the edge')
            for k, s in enumerate(lines):
                y = 40 + 38 * k
                cv2.putText(img, s, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5)
                cv2.putText(img, s, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255) if (red and k == len(lines) - 1) else (255, 255, 255), 2)
            if w > 960:
                img = cv2.resize(img, (w * 2 // 3, h * 2 // 3))
            cv2.imshow('hand follow', img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                reset_filters()
                st['ref'], st['clamped'], st['turn'] = None, False, 0.0
                q_cmd[4] = q_home[4]
                go_home('ref')
            elif key == ord('p'):
                if st['mode'] == 'follow':
                    st['mode'] = 'paused'
                elif st['mode'] == 'paused' and st['ref'] is not None:
                    st['mode'] = 'follow'
            elif key == ord('h'):
                st['ref'], st['turn'] = None, 0.0
                go_home('wait')
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        print('\ngoing HOME...')
        t_end = time.perf_counter() + 15
        while not step_to(q_rest) and time.perf_counter() < t_end:
            time.sleep(1 / so101_real.RATE)
        pipe.stop()
        hand_det.close()
        arm.close()


if __name__ == '__main__':
    main()
