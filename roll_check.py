'''Line up the WRIST ROLL of the real arm with the simulation (the jaws must close across the object).

Why: the wrist_roll zero is wherever the wrist happened to be during the LeRobot calibration, so the
real jaws can be turned (e.g. 30-45 deg) from where the planner thinks they are: the gripper then
comes down diagonally on a cube. table_calib.py cannot see this (it only touches with the jaw tip).

  python roll_check.py            real arm + camera
  python roll_check.py --dry      fake arm (to try the program)

1. Put a cube (or any box) on the table, 150-280 mm from the base. Click its top in the camera window.
   The arm goes above it with the wrist turned for the measured angle, gripper open.
   The yellow line drawn on the image is where the SIMULATION thinks the jaws are.
2. Look from straight above: the jaws should be square to the cube's faces (closing straight across it).
     a / d   turn the wrist 1 deg (A / D: 5 deg) until the real jaws are square to the cube
     l       lower around the cube (check: the open jaws pass on both sides)     u  back up
3. o   save: the turn you added goes into joint_offsets.json (wrist_roll); every script uses it.
   The arm goes home. Turn the cube 30-45 deg and click it: the jaws must now be square by themselves.
   If they follow in the WRONG direction (error grows when the cube turns), tell Claude: roll sign.
h home, 0 reset the added turn (not saved), q quit.
'''
import argparse
import json
import os
import sys
import time
import numpy as np
import cv2
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "Reference"))
sys.path.insert(0, HERE)
import so101_real                                                    # noqa: E402
from so101_real import HOME, RATE, OFFSETS_FILE                     # noqa: E402
from keyboard_control import ik                                     # noqa: E402
from pick_place import BOX_SCENE, PITCH, SEEDS, GRASP_SEED, roll_for, grip_open_for, lowest_tip, smooth  # noqa: E402
from camera_pick import cloud_base, measure                         # noqa: E402
from detector import robot_mask                                     # noqa: E402
from verify_click import load_offset                                # noqa: E402
from handeye_calib import open_camera                               # noqa: E402

ABOVE = 0.025          # m: grip centre this far above the object's top
TIPS = ['fixed_jaw_sph_tip1', 'moving_jaw_sph_tip1']


def project(T, offset, intr, pts):
    '''Base-frame points (N x 3) -> image pixels.'''
    R, t = T[:3, :3], T[:3, 3] + offset
    c = (np.asarray(pts) - t) @ R
    return np.stack([c[:, 0] / c[:, 2] * intr.fx + intr.ppx, c[:, 1] / c[:, 2] * intr.fy + intr.ppy], 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--port', default=so101_real.PORT)
    ap.add_argument('--robot-id', default=so101_real.ROBOT_ID)
    a = ap.parse_args()
    so101_real.configure(a.port, a.robot_id)
    T, offset = np.load(os.path.join(HERE, 'T_base_cam.npy')), load_offset()
    import pyrealsense2 as rs
    model = mujoco.MjModel.from_xml_path(BOX_SCENE)
    d = mujoco.MjData(model)
    lo, hi = so101_real.real_limits_sim()
    lo = np.maximum(model.actuator_ctrlrange[:5, 0], lo)
    hi = np.minimum(model.actuator_ctrlrange[:5, 1], hi)
    pipe, _, _, intr = open_camera(depth=True)
    align = rs.align(rs.stream.color)
    arm = so101_real.RealArm(dry=a.dry)
    roll0 = float(so101_real.OFFSET[4])
    print(__doc__.split('1. Put')[0].split('Why:')[0])
    print(f'wrist_roll offset now: {roll0:+.1f} deg (joint_offsets.json)')
    st = {'click': None, 'q': None, 'q_up': None, 'q_low': None, 'obj': None, 'added': 0.0,
          'msg': 'click the top of a cube on the table'}

    def say(t):
        st['msg'] = t
        print(t)

    def move(q1, secs=2.5):
        q0 = arm.read_sim()
        n = int(secs * RATE)
        for k in range(1, n + 1):
            arm.send_sim(q0 + smooth(k / n) * (q1 - q0))
            time.sleep(1 / RATE)
        arm.correct_sag(q1)

    def resend():
        '''Same sim pose, new roll offset: only the wrist turns.'''
        so101_real.OFFSET[4] = roll0 + st['added']
        if st['q'] is not None:
            for _ in range(int(0.6 * RATE)):
                arm.send_sim(st['q'])
                time.sleep(1 / RATE)

    def plan(obj):
        c = obj['centre']
        g_open = grip_open_for(obj['width'])
        best = None
        for s in SEEDS:
            seed = np.append(GRASP_SEED, g_open)
            seed[0] = -np.arctan2(c[1], c[0])
            seed[1:4] = np.radians(s)
            p_up = np.array([*c, obj['height'] + ABOVE])
            q, err = ik(model, d, seed, p_up, PITCH, True, lo, hi, iters=500)
            if err < 0.004:
                best = q
                break
        if best is None:
            return 'cannot reach above it: move it to 150-280 mm from the base'
        best[4] = roll_for(model, d, best, obj['yaw'], lo, hi)
        best[5] = g_open
        q_up, _ = ik(model, d, best, np.array([*c, obj['height'] + ABOVE]), PITCH, True, lo, hi, iters=200)
        z_low = obj['height'] - 0.012
        q_low, err = ik(model, d, q_up, np.array([*c, z_low]), PITCH, True, lo, hi, iters=300)
        if err > 0.004 or lowest_tip(model, d, q_low) < 0.004:
            q_low = None
        return q_up, q_low

    def on_mouse(ev, x, y, *_):
        if ev == cv2.EVENT_LBUTTONDOWN:
            st['click'] = (x, y)

    win = 'roll_check (click cube | a/d A/D turn | l lower u up | o save | 0 reset | h home | q quit)'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    try:
        while True:
            fs = align.process(pipe.wait_for_frames())
            depth, color = fs.get_depth_frame(), fs.get_color_frame()
            img = np.asanyarray(color.get_data()).copy()
            if st['click'] is not None:
                u, v = st['click']
                st['click'] = None
                if st['q'] is not None:
                    say('the arm hides the cube: h (home) first, then click it')
                else:
                    P = cloud_base(depth, intr, T, offset)
                    q_now = arm.read_sim()
                    r = measure(P, u, v, robot=robot_mask(model, d, q_now, T, offset, intr, img.shape[:2]), bgr=img)
                    if isinstance(r, str):
                        say(r)
                    else:
                        out = plan(r)
                        if isinstance(out, str):
                            say(out)
                        else:
                            st['obj'], (st['q_up'], st['q_low']) = r, out
                            st['q'] = st['q_up']
                            c = r['centre'] * 1000
                            say(f'cube at x {c[0]:.0f} y {c[1]:.0f}, yaw {r["yaw"]:.0f} deg: moving above it...')
                            move(st['q'])
                            say('look from above: a/d until the jaws are square to the cube, then o (l = lower)')
            # what the simulation thinks: line between the two jaw tips
            q_show = st['q'] if st['q'] is not None else arm.read_sim()
            d.qpos[:6] = q_show[:6]
            mujoco.mj_kinematics(model, d)
            tips = np.array([d.geom_xpos[model.geom(g).id] for g in TIPS])
            uv = project(T, offset, intr, tips).astype(int)
            cv2.line(img, tuple(uv[0]), tuple(uv[1]), (0, 255, 255), 2)
            cv2.circle(img, tuple(uv[0]), 4, (0, 255, 255), -1)          # dot = fixed jaw
            if st['obj'] is not None:
                cv2.drawContours(img, [st['obj']['contour']], -1, (0, 255, 0), 1)
            cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
            cv2.putText(img, st['msg'][:110], (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.putText(img, f'wrist_roll offset {roll0:+.1f} + added {st["added"]:+.0f} deg   yellow = sim jaws',
                        (8, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.imshow(win, img)
            key = cv2.waitKey(1) & 0xFF
            step = {ord('a'): -1, ord('d'): 1, ord('A'): -5, ord('D'): 5}.get(key)
            if key == ord('q'):
                break
            elif step is not None:
                st['added'] += step
                resend()
                say(f'added {st["added"]:+.0f} deg: o to save when the jaws are square to the cube')
            elif key == ord('0'):
                st['added'] = 0.0
                resend()
                say('added turn reset to 0')
            elif key == ord('l') and st['q'] is st['q_up'] and st['q_up'] is not None:
                if st['q_low'] is None:
                    say('cannot lower here (fingertips would touch the table)')
                else:
                    st['q'] = st['q_low']
                    move(st['q'], 2.0)
                    say('lowered: the open jaws should pass on both sides of the cube. u = up')
            elif key == ord('u') and st['q'] is st['q_low'] and st['q'] is not None:
                st['q'] = st['q_up']
                move(st['q'], 1.5)
                say('up')
            elif key == ord('o'):
                new = roll0 + st['added']
                data = json.load(open(OFFSETS_FILE)) if os.path.exists(OFFSETS_FILE) else {}
                data['wrist_roll'] = round(new, 1)
                json.dump(data, open(OFFSETS_FILE, 'w'), indent=1)
                roll0, st['added'] = new, 0.0
                say(f'saved wrist_roll offset {new:+.1f} deg in joint_offsets.json: going home...')
                if st['q'] is st['q_low'] and st['q'] is not None:
                    move(st['q_up'], 1.5)
                st['q'] = None
                move(np.append(HOME, arm.read_sim()[5]), 3.0)
                say('saved. Turn the cube 30-45 deg and click it: the jaws must now be square to it by themselves')
            elif key == ord('h'):
                if st['q'] is st['q_low'] and st['q'] is not None:
                    move(st['q_up'], 1.5)
                st['q'] = None
                move(np.append(HOME, arm.read_sim()[5]), 3.0)
                say('home. Click a cube')
    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        arm.close()


if __name__ == '__main__':
    main()
