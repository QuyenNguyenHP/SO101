'''Table calibration: tie the camera to the robot ON THE TABLE, using touched points.

Why: handeye_calib.py uses the marker (solvePnP) high above the table, but picking uses the
DEPTH camera on the table. This calibrates exactly that: depth point on the table <-> robot.

For each pen X on the table (6-10 marks spread over the work area, 180-320 mm from the base):
  1. Hold the arm by hand (torque is OFF) and put the very END OF THE FIXED JAW (the jaw that
     does not move) exactly on the centre of the X, touching the table. Press SPACE.
  2. Move the arm away so the camera sees the X, then LEFT-CLICK the centre of the X.
  (The order of 1 and 2 does not matter.) The pair is saved to table_points.json.
Keys (click the image window first):
  SPACE  record the jaw tip from the servos        click  record the X from the camera
  u      undo the last pair                        c      compute and save T_base_cam.npy
  q      quit
Run:  python table_calib.py              (add --resume to keep table_points.json)
      python table_calib.py --solve-only (recompute from table_points.json, no camera/robot)
Then: python Reference/verify_click.py
'''
import argparse
import json
import os
import shutil
import sys
import time
import numpy as np
import cv2
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "Reference"))
sys.path.insert(0, HERE)
import so101_real                                              # noqa: E402
from handeye_calib import open_camera                          # noqa: E402
from verify_click import pixel_to_cam                          # noqa: E402

POINTS = os.path.join(HERE, 'json', 'table_points.json')
RESULT = os.path.join(HERE, 'T_base_cam.npy')
RESULT_JSON = os.path.join(HERE, 'json', 'T_base_cam.json')
BACKUP = os.path.join(HERE, 'npy', 'T_base_cam_handeye.npy')        # the marker result, kept once
JAW_TIP = 'fixed_jaw_sph_tip1'                                 # end of the fixed jaw in so101.xml
MIN_POINTS = 6
MATCH_TOL = 0.012          # m: robot and camera must agree on the distance between two marks


def jaw_tip(model, d, q):
    d.qpos[:6] = q[:6]
    mujoco.mj_kinematics(model, d)
    g = model.geom(JAW_TIP)
    return d.geom_xpos[g.id] - np.array([0, 0, g.size[0]])    # bottom of the small sphere


def fit(P_cam, P_base):
    '''Rigid transform base <- cam (Kabsch). Returns T (4x4), errors (m), best-fit scale.'''
    A, B = np.asarray(P_cam), np.asarray(P_base)
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cb - R @ ca
    err = np.linalg.norm(A @ R.T + T[:3, 3] - B, axis=1)
    scale = np.trace(np.diag(S) @ D) / ((A - ca) ** 2).sum()  # Umeyama scale, for information
    return T, err, scale


OFFSET_JOINTS = [1, 2, 3]       # shoulder_lift, elbow_flex, wrist_flex get a fine offset
PRIOR = 0.0005                   # m per degree: keeps the offsets small unless the data needs them
MAX_OFFSET = 12.0                # deg: larger means something else is wrong (servo calibration)


def tips_for(model, d, deg, base_offset, delta_deg):
    """Jaw-tip positions for raw servo readings (deg), with offsets base_offset - delta."""
    out = []
    for dg in deg:
        q = np.radians(np.asarray(dg) - base_offset)
        q[OFFSET_JOINTS] += np.radians(delta_deg)
        out.append(jaw_tip(model, d, np.append(q, 0.0)))
    return np.array(out)


def solve_joint(pairs, model, d):
    """Camera transform + fine joint offsets + table height, all together.
    Uses: camera point = robot point for every pair, and every touch is ON the table (same z)."""
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    C = np.array([p['cam'] for p in pairs])
    deg = np.array([p['deg'] for p in pairs])
    off0 = so101_real.OFFSET.copy()
    P0 = tips_for(model, d, deg, off0, np.zeros(3))
    T0, _, _ = fit(C, P0)

    def residual(z):
        R = Rotation.from_rotvec(z[:3]).as_matrix() * z[10]           # z[10]: depth scale
        P = tips_for(model, d, deg, off0, z[6:9])
        return np.concatenate([(C @ R.T + z[3:6] - P).ravel(), P[:, 2] - z[9], z[6:9] * PRIOR,
                               [(z[10] - 1) * 0.02]])
    z0 = np.concatenate([Rotation.from_matrix(T0[:3, :3]).as_rotvec(), T0[:3, 3], np.zeros(3),
                         [P0[:, 2].mean()], [1.0]])
    z = least_squares(residual, z0, loss='soft_l1', f_scale=0.005).x
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rotation.from_rotvec(z[:3]).as_matrix() * z[10], z[3:6]   # scale folded into T
    P = tips_for(model, d, deg, off0, z[6:9])
    err = np.linalg.norm(C @ T[:3, :3].T + T[:3, 3] - P, axis=1)
    new_off = off0.copy()
    new_off[OFFSET_JOINTS] -= z[6:9]
    return T, err, P, new_off, z[9], z[10]


def leave_one_out(pairs, model, d):
    '''XY error at each touch when it is predicted from the OTHER touches only (like a new click).'''
    out = []
    for i in range(len(pairs)):
        rest = pairs[:i] + pairs[i + 1:]
        T, _, _, off, _, _ = solve_joint(rest, model, d)
        saved, so101_real.OFFSET = so101_real.OFFSET, off
        tip = tips_for(model, d, [pairs[i]['deg']], off, np.zeros(3))[0]
        so101_real.OFFSET = saved
        pred = T[:3, :3] @ np.array(pairs[i]['cam']) + T[:3, 3]
        out.append(np.linalg.norm((pred - tip)[:2]))
    return np.array(out)


def compute(pairs):
    print('\n--- table calibration ---')
    use_joint = len(pairs) >= 6 and all('deg' in p for p in pairs)
    P_cam = np.array([p['cam'] for p in pairs])
    if use_joint:
        model = mujoco.MjModel.from_xml_path(so101_real.SCENE)
        d = mujoco.MjData(model)
        deg = np.array([p['deg'] for p in pairs])
        P_old = tips_for(model, d, deg, so101_real.OFFSET, np.zeros(3))
        _, err_old, sc_old = fit(P_cam, P_old)
        print(f'with the current joint offsets: mean error {err_old.mean()*1000:.1f} mm, '
              f'touch heights z {P_old[:, 2].min()*1000:.0f}..{P_old[:, 2].max()*1000:.0f} mm, scale {sc_old:.3f}')
        T, err, P_base, new_off, h, dscale = solve_joint(pairs, model, d)
        change = new_off - so101_real.OFFSET
        print('fine joint offsets found (deg, change):',
              ', '.join(f'{so101_real.JOINTS[j]} {change[j]:+.1f}' for j in OFFSET_JOINTS))
        print(f'touch heights now z {P_base[:, 2].min()*1000:.0f}..{P_base[:, 2].max()*1000:.0f} mm '
              f'(table at z {h*1000:.0f} mm), depth scale {dscale:.3f}')
        loo = leave_one_out(pairs, model, d)
        print('XY error predicting each touch from the others (= what a new click will do):')
        print('  ' + '  '.join(f'#{i + 1}: {e*1000:.1f}' for i, e in enumerate(loo)) + ' mm')
        print(f'  mean {loo.mean()*1000:.1f} mm, max {loo.max()*1000:.1f} mm  ->  '
              + ('GOOD' if loo.mean() < 0.004 else 'usable' if loo.mean() < 0.008 else
                 'not good enough: redo the flagged touches / add more spread-out marks'))
    else:
        if len(pairs) < 6 or not all('deg' in p for p in pairs):
            print('(rigid fit only: joint offsets need 6+ pairs recorded with this version)')
        P_base = np.array([p['base'] for p in pairs])
        T, err, _ = fit(P_cam, P_base)
    _, _, scale = fit(P_cam, P_base)
    for i, (pb, e) in enumerate(zip(P_base, err)):
        print(f'  point {i + 1:2d}: robot x {pb[0]*1000:5.0f} y {pb[1]*1000:5.0f} z {pb[2]*1000:4.0f} mm   '
              f'error {e*1000:4.1f} mm' + ('   <- check this one' if e > max(3 * np.median(err), 0.006) else ''))
    mean = err.mean() * 1000
    print(f'mean error {mean:.1f} mm, max {err.max()*1000:.1f} mm  ->  '
          + ('good' if mean < 4 else 'usable' if mean < 8 else 'check the flagged points / redo them'))
    print(f'depth scale vs robot: {scale:.3f}  (1.000 = perfect)')
    print('camera position in robot base frame (mm):', np.round(T[:3, 3] * 1000))
    if use_joint:
        if np.abs(change).max() > MAX_OFFSET:
            print(f'NOT saving joint offsets: a change over {MAX_OFFSET:.0f} deg means the servo calibration '
                  'itself is off. Redo lerobot-calibrate (docs/SETUP_ROBOT.md), then this.')
            return err
        json.dump({j: round(float(v), 2) for j, v in zip(so101_real.JOINTS, new_off)} |
                  {'note': 'deg; sim angle = real reading - offset. Written by table_calib.py'},
                  open(so101_real.OFFSETS_FILE, 'w'), indent=1)
        print('saved joint_offsets.json: every script (keyboard_control, verify_click, pick_place...) uses it')
    if os.path.exists(RESULT) and not os.path.exists(BACKUP):
        os.makedirs(os.path.dirname(BACKUP), exist_ok=True)
        shutil.copy(RESULT, BACKUP)
    np.save(RESULT, T)
    json.dump({'method': 'table touch points' + (' + joint offsets' if use_joint else ''), 'T_base_cam': T.tolist(),
               'n_points': len(pairs), 'mean_error_mm': float(mean), 'depth_scale': float(scale)},
              open(RESULT_JSON, 'w'), indent=1)
    print('saved T_base_cam.npy')
    return err


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=so101_real.PORT)
    ap.add_argument('--robot-id', default=so101_real.ROBOT_ID)
    ap.add_argument('--resume', action='store_true', help='keep the pairs already in table_points.json')
    ap.add_argument('--solve-only', action='store_true', help='recompute from table_points.json only')
    ap.add_argument('--dry', action='store_true', help='fake arm (camera test only)')
    a = ap.parse_args()
    so101_real.configure(a.port, a.robot_id)

    pairs = json.load(open(POINTS)) if (a.resume or a.solve_only) and os.path.exists(POINTS) else []
    if a.solve_only:
        if len(pairs) < MIN_POINTS:
            sys.exit(f'need at least {MIN_POINTS} pairs in table_points.json, have {len(pairs)}')
        compute(pairs)
        return

    import pyrealsense2 as rs
    model = mujoco.MjModel.from_xml_path(so101_real.SCENE)
    d = mujoco.MjData(model)
    pipe, _, _, intr = open_camera(depth=True)
    align = rs.align(rs.stream.color)
    if not a.dry:
        input('Torque will be turned OFF: support the arm with your hand, then press Enter...')
    arm = so101_real.RealArm(dry=a.dry)
    if not a.dry:
        arm.robot.bus.disable_torque()
    print(__doc__.split('For each')[1].split('Run:')[0])

    st = {'click': None, 'cam': None, 'base': None, 'deg': None, 'uv': None, 'msg': 'put the fixed-jaw tip on an X, press SPACE'}

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            st['click'] = (x, y)

    def say(t):
        st['msg'] = t
        print(t)

    def try_pair():
        if st['cam'] is not None and st['base'] is not None:
            # same mark seen by both? distances to the earlier marks must match (no calibration needed)
            bad = []
            for k, p in enumerate(pairs):
                dr = np.linalg.norm(st['base'] - np.array(p['base']))
                dc = np.linalg.norm(st['cam'] - np.array(p['cam']))
                if abs(dr - dc) > MATCH_TOL + 0.25 * dr:       # loose: joint offsets distort distances
                    bad.append(f'#{k + 1}: robot {dr*1000:.0f} vs camera {dc*1000:.0f} mm')
            if bad and len(bad) * 2 >= len(pairs):
                st['cam'] = st['base'] = None
                say('REJECTED: jaw tip and click are not the same X (' + '; '.join(bad[:2])
                    + '). Redo this X: SPACE + click')
                return
            if bad:
                print('  note: this pair disagrees with ' + ', '.join(b.split(':')[0] for b in bad)
                      + ' - one of those may be wrong (u removes the LAST pair only)')
            pairs.append({'cam': st['cam'].tolist(), 'base': st['base'].tolist(), 'deg': st['deg'].tolist(),
                          'uv': list(st['uv']) if st['uv'] else None})
            json.dump(pairs, open(POINTS, 'w'), indent=1)
            st['cam'] = st['base'] = None
            say(f'pair {len(pairs)} saved. Next X: jaw tip on it + SPACE, then click it'
                + ('  (or press c)' if len(pairs) >= MIN_POINTS else ''))

    win = 'table_calib  (SPACE jaw tip | click the X | u undo | c compute | q quit)'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    try:
        while True:
            ok, fs = pipe.try_wait_for_frames(2000)
            if not ok:
                continue
            f = align.process(fs)
            depth = f.get_depth_frame()
            img = np.asanyarray(f.get_color_frame().get_data()).copy()

            if st['click'] is not None:
                u, v = st['click']
                st['click'] = None
                p = pixel_to_cam(depth, intr, u, v)
                if p is None:
                    say('no depth there: click a little to the side')
                else:
                    near = [k + 1 for k, q in enumerate(pairs)
                            if q.get('uv') and np.hypot(q['uv'][0] - u, q['uv'][1] - v) < 12]
                    if near:
                        say(f'that is mark #{near[0]} again (green). Click the X you are touching now')
                        continue
                    st['cam'], st['uv'] = p, (u, v)
                    say('X recorded from the camera' + ('' if st['base'] is not None else
                                                       ': now put the jaw tip on it and press SPACE'))
                    try_pair()

            for k, p in enumerate(pairs):                # marks already done: green with number
                if p.get('uv'):
                    cv2.drawMarker(img, tuple(p['uv']), (0, 200, 0), cv2.MARKER_CROSS, 18, 2)
                    cv2.putText(img, str(k + 1), (p['uv'][0] + 6, p['uv'][1] - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)
            if st['uv'] is not None and st['cam'] is not None:
                cv2.drawMarker(img, st['uv'], (0, 0, 255), cv2.MARKER_CROSS, 25, 2)
            cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
            cv2.putText(img, st['msg'], (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(img, f'pairs: {len(pairs)}  (need {MIN_POINTS}+, 6-10 is better)'
                        + ('   jaw tip: OK' if st['base'] is not None else '')
                        + ('   X click: OK' if st['cam'] is not None else ''),
                        (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.imshow(win, img)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord(' '):
                reads, raws = [], []
                for _ in range(10):                     # average 10 servo reads (~0.3 s)
                    reads.append(jaw_tip(model, d, arm.read_sim()))
                    raws.append(arm.read_real()[0])
                    time.sleep(0.03)
                reads = np.array(reads)
                st['deg'] = np.mean(raws, axis=0)
                if np.ptp(reads, axis=0).max() > 0.002:
                    say('arm moved while recording: hold it still and press SPACE again')
                else:
                    st['base'] = reads.mean(0)
                    b = st['base'] * 1000
                    say(f'jaw tip at x {b[0]:.0f} y {b[1]:.0f} z {b[2]:.0f} mm'
                        + ('' if st['cam'] is not None else ': move the arm away and click the X'))
                    try_pair()
            elif key == ord('u'):
                if st['cam'] is not None or st['base'] is not None:
                    st['cam'] = st['base'] = None
                    say('cleared the half-done pair')
                elif pairs:
                    pairs.pop()
                    json.dump(pairs, open(POINTS, 'w'), indent=1)
                    say(f'removed last pair, {len(pairs)} left')
            elif key == ord('c'):
                if len(pairs) < MIN_POINTS:
                    say(f'need at least {MIN_POINTS} pairs, have {len(pairs)}')
                else:
                    err = compute(pairs)
                    say(f'saved T_base_cam.npy, mean error {err.mean()*1000:.1f} mm (details in terminal)')
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        arm.close()


if __name__ == '__main__':
    main()
