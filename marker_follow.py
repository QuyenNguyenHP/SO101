'''Marker follow (relative): the gripper tip copies how an ArUco marker moves in front of the D415.

Idea: the camera and the arm can be anywhere; only the camera's DIRECTION matters (it must look the
same way the arm points). When you press SPACE the arm goes to its START pose (tip 20 cm in front,
15 cm up, nose 45 deg down: the middle of what it can reach) and the marker's 3D position is taken
as the reference. From then on
    tip target = tip at START + scale * (marker now - marker at reference)
with the camera axes turned into robot axes (camera looking the way the arm points):
    marker further from the camera -> tip forward (+X)
    marker right in the image      -> tip right   (-Y, seen from behind the robot)
    marker up in the image         -> tip up      (+Z)
--mirror flips left/right (use it if you stand FACING the camera and want a mirror image).

The usual HOME (home_pose.json, arm folded up) is a bad place to start: there the arm can hardly
move forward. It starts at HOME, goes to START, and goes back to HOME when you quit.

Version 1: position only. The gripper keeps the START tilt when it can; if the tip
cannot be reached with that tilt, the tilt is let free. Out of reach: the screen turns RED and the
tip stops at the closest reachable point on the way to the target (and never below 1 cm).
Smooth: One-Euro filter on the marker position + speed limit on the tip + joint step limit.
Marker lost: the arm holds still.

Marker: ArUco DICT_4X4_50, ID 0 (--marker-id). Its 3D position = marker centre pixel + median depth
over the marker (aligned depth), so no marker size is needed.

Run:  python marker_follow.py --dry       camera + IK only, fake arm (check the axes first!)
      python marker_follow.py             real arm
      --start 200,0,150 (START tip, mm)  --tilt 45 (deg below horizontal)
      --scale 1.0   --mirror   --speed 0.15 (m/s max tip speed)   --mincutoff 1.0 --beta 4.0 (filter)
Keys (click the image window first):
  SPACE  take the reference (arm at START, marker where it is now) and start following
         (press again any time to re-zero: the arm goes back to START, the marker's new spot is the reference)
  p      pause / resume following (paused: the arm holds still)
  h      pause and go back to START          q  quit (goes HOME first)
SAFETY: start with --dry, then the real arm with small, slow moves. Keep a hand near 'p'.
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
from handeye_calib import open_camera, make_detector                 # noqa: E402

Z_MIN = 0.01             # m: tip never below this (table)
REACH_TOL = 0.003        # m: IK must land this close to count as reachable
LOST_S = 0.3             # s without the marker -> "lost" shown (the arm already holds)
JUMP_M = 0.15            # m: a measurement this far from the filtered one in one frame = depth glitch
HOME_TOL_DEG = 1.0       # deg: "arrived"
JOINT_STEP = 1.5         # deg per frame, all joints in step (straight in joint space) when going to a pose


# ---------------------------------------------------------------- filter ----
class OneEuro:
    '''One-Euro filter (Casiez 2012): smooth when still, little lag when moving. x: 3-vector.'''

    def __init__(self, mincutoff=1.0, beta=4.0, dcutoff=1.0):
        self.mincutoff, self.beta, self.dcutoff = mincutoff, beta, dcutoff
        self.x = self.dx = self.t = None

    @staticmethod
    def _a(cutoff, dt):
        tau = 1.0 / (2 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self):
        self.x = self.dx = self.t = None

    def __call__(self, x, t):
        x = np.asarray(x, float)
        if self.x is None:
            self.x, self.dx, self.t = x.copy(), np.zeros(3), t
            return self.x.copy()
        dt = max(t - self.t, 1e-3)
        self.t = t
        a_d = self._a(self.dcutoff, dt)
        self.dx = a_d * (x - self.x) / dt + (1 - a_d) * self.dx
        cutoff = self.mincutoff + self.beta * np.linalg.norm(self.dx)
        a = self._a(cutoff, dt)
        self.x = a * x + (1 - a) * self.x
        return self.x.copy()


# ---------------------------------------------------------------- marker ----
def marker_point(corners, z_raw, units, intr):
    '''Marker centre in the camera frame (m): centre pixel + median depth inside the marker. Or None.'''
    c = corners.reshape(4, 2)
    centre = c.mean(0)
    poly = (centre + 0.8 * (c - centre)).astype(np.int32)     # a bit inside the border
    mask = np.zeros(z_raw.shape, np.uint8)
    cv2.fillConvexPoly(mask, poly, 1)
    z = z_raw[mask > 0].astype(np.float64) * units
    z = z[z > 0]
    if len(z) < 20:
        return None
    zm = float(np.median(z))
    u, v = centre
    return np.array([(u - intr.ppx) / intr.fx * zm, (v - intr.ppy) / intr.fy * zm, zm])


def cam_to_robot(d, mirror):
    '''Camera-frame displacement (x right, y down, z away) -> robot base displacement.'''
    r = np.array([d[2], -d[0], -d[1]])
    if mirror:
        r[1] = -r[1]
    return r


# ---------------------------------------------------------------- reach -----
class Reach:
    '''IK with the HOME tilt, then free tilt; out of reach -> closest point on the way.'''

    def __init__(self, model, lo, hi, pitch):
        self.model, self.d = model, mujoco.MjData(model)
        self.lo, self.hi, self.pitch = lo, hi, pitch

    def solve(self, q, target):
        q1, e = ik(self.model, self.d, q, target, self.pitch, True, self.lo, self.hi)
        if e < REACH_TOL:
            return q1, 'tilt held'
        q2, e = ik(self.model, self.d, q, target, self.pitch, False, self.lo, self.hi)
        if e < REACH_TOL:
            return q2, 'tilt free'
        return None, ''

    def go(self, q, tip_now, target):
        '''(q, tip actually aimed at, clamped?, note). tip_now must be reachable (it is where q is).'''
        clamped = target[2] < Z_MIN
        target = target.copy()
        target[2] = max(target[2], Z_MIN)
        q_new, note = self.solve(q, target)
        if q_new is not None:
            return q_new, target, clamped, note
        lo_s, hi_s, best = 0.0, 1.0, (q, tip_now, 'hold')       # bisection along tip_now -> target
        for _ in range(7):
            s = (lo_s + hi_s) / 2
            p = tip_now + s * (target - tip_now)
            q_s, note = self.solve(q, p)
            if q_s is None:
                hi_s = s
            else:
                lo_s, best = s, (q_s, p, note)
        return best[0], best[1], True, best[2]


# ---------------------------------------------------------------- main ------
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--dry', action='store_true', help='fake arm: nothing moves')
    ap.add_argument('--marker-id', type=int, default=0)
    ap.add_argument('--start', default='200,0,150', help='START tip position x,y,z in mm (robot base)')
    ap.add_argument('--tilt', type=float, default=45.0, help='START tilt, deg below horizontal')
    ap.add_argument('--scale', type=float, default=1.0, help='tip moves scale x the marker move')
    ap.add_argument('--mirror', action='store_true', help='flip left/right')
    ap.add_argument('--speed', type=float, default=0.15, help='max tip speed, m/s')
    ap.add_argument('--mincutoff', type=float, default=1.0, help='filter: lower = smoother when still')
    ap.add_argument('--beta', type=float, default=4.0, help='filter: higher = less lag when moving fast')
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
    tip_home, axis = tcp(model, d_fk, q_home)                      # tip_home = START tip
    reach = Reach(model, lo, hi, pitch_of(axis))
    detector = make_detector()

    pipe, _, _, intr = open_camera(depth=True)
    try:
        units = pipe.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        arm = so101_real.RealArm(dry=a.dry, max_step_deg=2.0)
    except BaseException:
        pipe.stop()
        raise

    q = arm.read_sim()
    q_cmd = q.copy()                      # what we send (5 joints + gripper as it is now)
    st = {'mode': 'homing', 'after_home': 'wait', 'ref': None, 'tip': tip_home.copy(),
          'last_seen': 0.0, 'clamped': False, 'note': '', 'raw': None}
    filt = OneEuro(a.mincutoff, a.beta)
    print(__doc__.split('Keys')[1].split('SAFETY')[0])
    print(f'START tip: x{tip_home[0]*1000:.0f} y{tip_home[1]*1000:.0f} z{tip_home[2]*1000:.0f} mm, '
          f'tilt {reach.pitch:.0f} deg' + ('   [DRY: fake arm]' if a.dry else ''))

    goal = {'q': q_home}

    def go_home(then):
        st['mode'], st['after_home'] = 'homing', then
        goal['q'] = q_home

    def step_to(q_goal):
        '''Move q_cmd toward q_goal, all joints together; True when the real arm is there.'''
        diff = q_goal - q_cmd[:5]
        m = np.degrees(np.abs(diff)).max()
        q_cmd[:5] += diff * (min(1.0, JOINT_STEP / m) if m > 0 else 1.0)
        arm.send_sim(q_cmd)
        return np.abs(arm.last_deg - np.clip(so101_real.sim_to_real(q_goal), arm.lo, arm.hi)).max() < HOME_TOL_DEG

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

            # ---- marker ----
            corners, ids, _ = detector.detectMarkers(img)
            p_raw = None
            if ids is not None:
                for c, i in zip(corners, ids.ravel()):
                    if i == a.marker_id:
                        cv2.polylines(img, [c.reshape(4, 2).astype(np.int32)], True, (0, 255, 0), 2)
                        p_raw = marker_point(c, z_raw, units, intr)
                        break
            if p_raw is not None and filt.x is not None and np.linalg.norm(p_raw - filt.x) > JUMP_M:
                p_raw = None                                   # depth glitch: ignore this frame
            p = None
            if p_raw is not None:
                p, st['last_seen'] = filt(p_raw, now), now
            elif now - st['last_seen'] > LOST_S:
                filt.reset()                                   # restart cleanly when it comes back
            seen = now - st['last_seen'] <= LOST_S

            # ---- arm ----
            if st['mode'] == 'homing':
                if step_to(goal['q']):
                    st['tip'] = tip_home.copy()
                    if st['after_home'] == 'ref':
                        if p is not None:
                            st['ref'], st['mode'] = p.copy(), 'follow'
                            print('reference taken: following')
                    else:
                        st['mode'] = st['after_home']
            elif st['mode'] == 'follow' and p is not None:
                d_cam = p - st['ref']
                target = tip_home + a.scale * cam_to_robot(d_cam, a.mirror)
                step = target - st['tip']                         # tip speed limit
                n = np.linalg.norm(step)
                if n > a.speed * dt:
                    target = st['tip'] + step * (a.speed * dt / n)
                q_new, aimed, clamped, note = reach.go(q_cmd, st['tip'], target)
                q_cmd[:5] = q_new[:5]
                st['tip'], st['note'], st['clamped'] = aimed, note, clamped
                arm.send_sim(q_cmd)
            else:
                arm.send_sim(q_cmd)                               # hold

            # ---- screen ----
            h, w = img.shape[:2]
            red = st['mode'] == 'follow' and st['clamped']
            if red:
                cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 0, 255), 14)
            lines = [f'mode: {st["mode"].upper()}' + ('  [DRY]' if a.dry else '')
                     + ('   MARKER LOST' if not seen else '')]
            if st['mode'] == 'wait':
                lines.append('SPACE = take reference and follow')
            if st['mode'] == 'homing' and st['after_home'] == 'ref' and not seen:
                lines.append('show the marker to take the reference')
            if p is not None:
                lines.append(f'marker cam x{p[0]*1000:6.0f} y{p[1]*1000:6.0f} z{p[2]*1000:6.0f} mm')
            if st['ref'] is not None and p is not None:
                dr = cam_to_robot(p - st['ref'], a.mirror) * 1000
                lines.append(f'move fwd {dr[0]:+5.0f}  left {dr[1]:+5.0f}  up {dr[2]:+5.0f} mm')
            tip = st['tip'] - tip_home
            lines.append(f'tip  fwd {tip[0]*1000:+5.0f}  left {tip[1]*1000:+5.0f}  up {tip[2]*1000:+5.0f} mm'
                         + (f'  ({st["note"]})' if st['mode'] == 'follow' else ''))
            if red:
                lines.append('OUT OF REACH: tip stopped at the edge')
            for k, s in enumerate(lines):
                y = 40 + 38 * k
                cv2.putText(img, s, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5)
                cv2.putText(img, s, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255) if (red and k == len(lines) - 1) else (255, 255, 255), 2)
            if w > 960:
                img = cv2.resize(img, (w * 2 // 3, h * 2 // 3))
            cv2.imshow('marker follow', img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                filt.reset()
                st['ref'], st['clamped'] = None, False
                go_home('ref')                                     # reference taken once at HOME
            elif key == ord('p'):
                if st['mode'] == 'follow':
                    st['mode'] = 'paused'
                elif st['mode'] == 'paused' and st['ref'] is not None:
                    st['mode'] = 'follow'
            elif key == ord('h'):
                st['ref'] = None
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
        arm.close()


if __name__ == '__main__':
    main()
