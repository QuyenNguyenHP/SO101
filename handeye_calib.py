'''Hand-eye calibration (eye-to-hand): find T_base_cam for a fixed RealSense D415.

An ArUco marker is stuck on the gripper's FIXED jaw. You move the arm by hand (torque off)
to 15-25 poses; at each pose the script records
  c_i            marker centre seen by the camera (OpenCV solvePnP), metres, camera frame
  T_base_grip_i  gripper pose from the real joint angles (MuJoCo forward kinematics)
and solves  R c_i + t = R_bg,i p + t_bg,i  for R, t (= T_base_cam) and p (marker in gripper frame).

Run:  python handeye_calib.py --marker-size 0.0296
      python handeye_calib.py --marker-size 0.0296 --resume   (keep the samples in samples.json)
      python handeye_calib.py --marker-size 0.0270 --solve-only  (recompute only: no camera, no robot)
Keys (click the image window first):
  SPACE  record a sample (hold the arm still)     u  undo last sample
  c      compute and save T_base_cam.npy          x  drop the worst sample (after c), then c again
  q      quit
Guide: SETUP_TABLE_2D_CALIB.md
'''
import argparse
import json
import os
import sys
import numpy as np
import cv2
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import so101_real                                     # noqa: E402

SAMPLES = os.path.join(HERE, 'samples.json')
RESULT = os.path.join(HERE, 'T_base_cam.npy')
RESULT_JSON = os.path.join(HERE, 'T_base_cam.json')
GRIP_BODY = 'gripper'          # MuJoCo body the fixed jaw belongs to
AVG_FRAMES = 10                # a sample = average of this many detections
MARKER_ID = 1                  # ArUco ID stuck on the fixed jaw (DICT_4X4_50)
MAX_SPREAD = 0.002             # m: reject the sample if the marker moved more than this meanwhile


# ---------------- camera ----------------
# Tried in order; the first one the camera accepts is used. USB 2 only allows the lower ones.
COLOR_MODES = [(1280, 720, 30), (1280, 720, 15), (640, 480, 30), (640, 480, 15)]
DEPTH_MODES = [(1280, 720, 30), (848, 480, 30), (640, 480, 30), (640, 480, 15), (640, 480, 6)]


def open_camera(depth=False):
    import pyrealsense2 as rs
    devs = rs.context().query_devices()
    if len(devs) == 0:
        sys.exit('No RealSense camera found: plug the D415 into a USB 3 (blue) port.')
    usb = devs[0].get_info(rs.camera_info.usb_type_descriptor)
    if not usb.startswith('3'):
        print(f'WARNING: camera is on USB {usb}: only low resolution / frame rate is available.\n'
              '         Use a USB 3 port (blue) and a USB 3 cable for 1280x720 @ 30 fps.')
    pipe, profile, last_err = rs.pipeline(), None, ''
    # with depth: same size for both streams (keeps aligned pixels sharp), same fps
    combos = ([(c, c) for c in COLOR_MODES if c in DEPTH_MODES] if depth else [(c, None) for c in COLOR_MODES])
    for c, dm in combos:
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, c[0], c[1], rs.format.bgr8, c[2])
        if dm:
            cfg.enable_stream(rs.stream.depth, dm[0], dm[1], rs.format.z16, dm[2])
        try:
            profile = pipe.start(cfg)
        except RuntimeError as e:
            last_err = str(e)
            if 'busy' in last_err.lower():
                sys.exit('The camera is used by another program (Device or resource busy).\n'
                         'Close it first, e.g.:  systemctl --user stop d415-lab.service\n'
                         'Find it with:  fuser -v /dev/video*')
            continue
        print(f'camera: color {c[0]}x{c[1]} @ {c[2]} fps' + (' + depth' if dm else ''))
        break
    if profile is None:
        sys.exit(f'The camera refused every mode ({last_err}): unplug and replug it, then try realsense-viewer.')
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
    dist = np.array(intr.coeffs[:5])
    return pipe, K, dist, intr


def make_detector(dictionary=cv2.aruco.DICT_4X4_50):
    d = cv2.aruco.getPredefinedDictionary(dictionary)
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.minMarkerPerimeterRate = 0.01          # accept smaller markers (default 0.03 = ~38 px side min)
    p.adaptiveThreshWinSizeMax = 53          # cope with uneven light / shadows
    p.adaptiveThreshWinSizeStep = 6
    p.perspectiveRemoveIgnoredMarginPerCell = 0.2
    p.maxErroneousBitsInBorderRate = 0.5     # tolerate a slightly damaged black border
    p.errorCorrectionRate = 0.8
    return cv2.aruco.ArucoDetector(d, p)


LAST = {'why': ''}                           # why the last frame had no usable marker (shown on screen)


def find_marker(img, detector, marker_id, size, K, dist):
    '''Marker centre in camera frame (m) and its rvec, or (None, None). Draws on img.'''
    corners, ids, rejected = detector.detectMarkers(img)
    for c in rejected:                       # red: square found but not decoded
        cv2.polylines(img, [c.astype(int)], True, (0, 0, 255), 1)
    if ids is None or marker_id not in ids.flatten():
        if ids is not None:
            LAST['why'] = f'seeing ID {ids.flatten().tolist()}, not ID {marker_id}: use --marker-id'
        elif len(rejected):
            LAST['why'] = 'red squares = found but not decoded: white border? glare? blur? bent?'
        else:
            LAST['why'] = 'nothing square in view (run marker_check.py for a diagnosis)'
        return None, None
    LAST['why'] = ''
    k = list(ids.flatten()).index(marker_id)
    h = size / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, corners[k].reshape(4, 2), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None, None
    cv2.aruco.drawDetectedMarkers(img, [corners[k]], np.array([[marker_id]]), (0, 255, 0))
    cv2.drawFrameAxes(img, K, dist, rvec, tvec, size * 0.75)
    return tvec.flatten(), rvec.flatten()


# ---------------- robot side ----------------
def gripper_pose(model, d, q):
    '''Gripper body pose in the robot base frame for sim joint angles q.'''
    d.qpos[:5] = q[:5]
    mujoco.mj_kinematics(model, d)
    b = model.body(GRIP_BODY).id
    return d.xmat[b].reshape(3, 3).copy(), d.xpos[b].copy()


# ---------------- solver ----------------
def solve(samples):
    '''Returns T_base_cam (4x4), p (marker in gripper frame), per-sample errors (m).'''
    C = np.array([s['c'] for s in samples])
    Rb = np.array([np.reshape(s['R_bg'], (3, 3)) for s in samples])
    tb = np.array([s['t_bg'] for s in samples])
    n = len(samples)

    # 1. linear: unknowns x = [R (9, row-major), t (3), p (3)]
    A, b = np.zeros((3 * n, 15)), np.zeros(3 * n)
    for i in range(n):
        for r in range(3):
            A[3 * i + r, 3 * r:3 * r + 3] = C[i]
            A[3 * i + r, 9 + r] = 1
            A[3 * i + r, 12:15] = -Rb[i][r]
            b[3 * i + r] = tb[i][r]
    x = np.linalg.lstsq(A, b, rcond=None)[0]
    U, _, Vt = np.linalg.svd(x[:9].reshape(3, 3))
    R0 = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt     # nearest rotation

    # 2. refine with R kept a true rotation
    def residual(z):
        R = Rotation.from_rotvec(z[:3]).as_matrix()
        return ((C @ R.T + z[3:6]) - (np.einsum('nij,j->ni', Rb, z[6:9]) + tb)).ravel()
    z0 = np.concatenate([Rotation.from_matrix(R0).as_rotvec(), x[9:12], x[12:15]])
    z = least_squares(residual, z0, loss='soft_l1', f_scale=0.005).x
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rotation.from_rotvec(z[:3]).as_matrix(), z[3:6]
    err = np.linalg.norm(residual(z).reshape(n, 3), axis=1)
    return T, z[6:9], err


def scaled(samples, marker_size):
    '''Samples with c rescaled to marker_size (solvePnP position is proportional to the size used).'''
    return [dict(smp, c=(np.array(smp['c']) * marker_size / smp['size']).tolist()) for smp in samples]


def size_hint(samples, T, p):
    '''Scale factor on the camera positions that fits the data best (finds a wrong --marker-size).'''
    C = np.array([smp['c'] for smp in samples])
    Rb = np.array([np.reshape(smp['R_bg'], (3, 3)) for smp in samples])
    tb = np.array([smp['t_bg'] for smp in samples])

    def residual(z):
        R = Rotation.from_rotvec(z[:3]).as_matrix()
        return ((z[9] * C) @ R.T + z[3:6] - (np.einsum('nij,j->ni', Rb, z[6:9]) + tb)).ravel()
    z0 = np.concatenate([Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3], p, [1.0]])
    return least_squares(residual, z0, loss='soft_l1', f_scale=0.005).x[9]


def compute(samples, marker_size):
    '''Solve, print the report, save T_base_cam.npy / .json. Returns (T, per-sample errors).'''
    S = scaled(samples, marker_size)
    T, p, err = solve(S)
    report(T, p, err)
    k = size_hint(S, T, p)
    if abs(k - 1) > 0.02:
        print(f'HINT: the data fits best with --marker-size {marker_size * k:.4f} (now {marker_size}).\n'
              f'      Measure the black square again with a caliper. If that is right, recompute with\n'
              f'      python handeye_calib.py --solve-only --marker-size <measured>   (no new samples needed)')
    np.save(RESULT, T)
    json.dump({'T_base_cam': T.tolist(), 'marker_in_gripper_m': p.tolist(), 'marker_size_m': marker_size,
               'mean_error_mm': float(err.mean() * 1000), 'n_samples': len(samples)},
              open(RESULT_JSON, 'w'), indent=1)
    print(f'saved T_base_cam.npy, mean error {err.mean() * 1000:.1f} mm')
    return T, err


def report(T, p, err):
    print('\n--- result ---')
    for i, e in enumerate(err):
        flag = '   <- suspicious' if e > max(2.5 * np.median(err), 0.006) else ''
        print(f'  sample {i:2d}: {e * 1000:5.1f} mm{flag}')
    mean = err.mean() * 1000
    verdict = 'good' if mean < 4 else 'usable for objects > 3 cm' if mean < 8 else 'systematic error: see SETUP_TABLE_2D_CALIB.md section 7'
    print(f'mean error {mean:.1f} mm, max {err.max() * 1000:.1f} mm  ->  {verdict}')
    print('marker centre in gripper frame (mm):', np.round(p * 1000, 1),
          '  (should be a few cm; tens of cm = wrong)')
    print('camera position in robot base frame (mm):', np.round(T[:3, 3] * 1000),
          '  (check with a ruler: x forward, y left, z up)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=so101_real.PORT)
    ap.add_argument('--robot-id', default=so101_real.ROBOT_ID)
    ap.add_argument('--marker-size', type=float, required=True, help='black square side in m, e.g. 0.0296')
    ap.add_argument('--marker-id', type=int, default=MARKER_ID)
    ap.add_argument('--resume', action='store_true', help='keep the samples already in samples.json')
    ap.add_argument('--dry', action='store_true', help='fake arm (fixed pose): test the camera only')
    ap.add_argument('--solve-only', action='store_true',
                    help='no camera/robot: recompute from samples.json (e.g. with a corrected --marker-size)')
    ap.add_argument('--recorded-size', type=float, default=0.0298,
                    help='marker size used when old samples (no stored size) were recorded')
    a = ap.parse_args()
    so101_real.configure(a.port, a.robot_id)

    samples = []
    if (a.resume or a.solve_only) and os.path.exists(SAMPLES):
        samples = json.load(open(SAMPLES))
        for smp in samples:
            smp.setdefault('size', a.recorded_size)
        print(f'Loaded {len(samples)} samples from samples.json')
    if a.solve_only:
        if len(samples) < 6:
            sys.exit('need at least 6 samples in samples.json')
        compute(samples, a.marker_size)
        return

    model = mujoco.MjModel.from_xml_path(so101_real.SCENE)
    d = mujoco.MjData(model)
    pipe, K, dist, _ = open_camera()
    detector = make_detector()

    if not a.dry:
        input('Torque will be turned OFF: support the arm with your hand, then press Enter...')
    arm = so101_real.RealArm(dry=a.dry)
    if not a.dry:
        arm.robot.bus.disable_torque()
    print(__doc__.split('Keys')[1].split('Guide')[0])

    T, worst, buf, msg = None, None, [], ''
    try:
        while True:
            img = np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data()).copy()
            c, _ = find_marker(img, detector, a.marker_id, a.marker_size, K, dist)
            if buf:                                       # collecting a sample
                if c is None:
                    buf, msg = [], 'marker lost: sample NOT recorded'
                else:
                    buf.append(c)
                    if len(buf) == AVG_FRAMES:
                        pts = np.array(buf)
                        if np.ptp(pts, axis=0).max() > MAX_SPREAD:
                            msg = 'arm was moving: sample NOT recorded, hold it still'
                        else:
                            R_bg, t_bg = gripper_pose(model, d, arm.read_sim())
                            samples.append({'c': pts.mean(0).tolist(), 'q': arm.read_sim().tolist(),
                                            'R_bg': R_bg.ravel().tolist(), 't_bg': t_bg.tolist(),
                                            'size': a.marker_size})
                            json.dump(samples, open(SAMPLES, 'w'), indent=1)
                            msg = f'sample {len(samples) - 1} recorded'
                            print(msg)
                        buf = []

            color = (0, 200, 0) if c is not None else (0, 0, 255)
            cv2.putText(img, f'samples: {len(samples)}   ' + ('marker OK' if c is not None else 'NO MARKER'),
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
            cv2.putText(img, msg, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            if c is None:
                cv2.putText(img, LAST['why'], (20, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow('handeye_calib  (SPACE record, u undo, c compute, x drop worst, q quit)', img)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord(' '):
                if c is None:
                    msg = 'no marker in view'
                else:
                    buf, msg = [c], 'recording... hold still'
            elif key == ord('u') and samples:
                samples.pop()
                json.dump(samples, open(SAMPLES, 'w'), indent=1)
                msg = f'removed last sample, {len(samples)} left'
            elif key == ord('c'):
                if len(samples) < 6:
                    msg = f'need at least 6 samples (15-25 recommended), have {len(samples)}'
                else:
                    T, err = compute(samples, a.marker_size)
                    worst = int(np.argmax(err))
                    msg = f'saved T_base_cam.npy, mean error {err.mean() * 1000:.1f} mm'
            elif key == ord('x'):
                if worst is None:
                    msg = 'press c first'
                else:
                    samples.pop(worst)
                    json.dump(samples, open(SAMPLES, 'w'), indent=1)
                    msg = f'dropped sample {worst}, press c again'
                    print(msg)
                    worst = None
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        arm.close()


if __name__ == '__main__':
    main()
