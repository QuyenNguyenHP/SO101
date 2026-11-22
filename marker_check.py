'''Why isn't my ArUco marker detected? Live diagnosis for handeye_calib.py.

Run:  python marker_check.py               live window + report every second (q to quit)
      python marker_check.py --snap 5      no window: watch 5 s, print a report, save snapshots
      python marker_check.py --depth 0.0298   compare marker distance with the depth sensor
Snapshots go to ~/robot/SO101/debug/ (last frame, and the frame with the most candidates).

What it checks:
  - which dictionary / IDs are actually seen (handeye_calib needs DICT_4X4_50, ID 1 by default)
  - marker size in pixels (under ~40 px it gets unreliable)
  - "candidates": square shapes found but not decoded -> blur, glare, no white border, bent paper
  - sharpness (blur from motion / focus) and brightness / glare
'''
import argparse
import os
import sys
import time
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from handeye_calib import open_camera, make_detector, MARKER_ID                    # noqa: E402

DICTS = {n: getattr(cv2.aruco, n) for n in
         ['DICT_4X4_50', 'DICT_5X5_50', 'DICT_6X6_50', 'DICT_7X7_50', 'DICT_ARUCO_ORIGINAL',
          'DICT_APRILTAG_36h11'] if hasattr(cv2.aruco, n)}


def analyse(img, detectors):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    out = {'sharp': cv2.Laplacian(gray, cv2.CV_64F).var(), 'bright': gray.mean(),
           'glare': (gray > 250).mean() * 100, 'found': {}, 'rejected': 0, 'boxes': [], 'rej_boxes': []}
    for name, det in detectors.items():
        corners, ids, rej = det.detectMarkers(gray)
        if name == 'DICT_4X4_50':
            out['rejected'] = len(rej)
            out['rej_boxes'] = rej
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                side = np.mean([np.linalg.norm(c[0][k] - c[0][(k + 1) % 4]) for k in range(4)])
                out['found'][(name, int(i))] = side
                out['boxes'].append((name, int(i), c))
    return out


def advice(stats, n, want=MARKER_ID):
    seen = {}
    for s in stats:
        for k, side in s['found'].items():
            seen.setdefault(k, []).append(side)
    sharp = np.median([s['sharp'] for s in stats])
    bright = np.median([s['bright'] for s in stats])
    glare = np.median([s['glare'] for s in stats])
    rej = np.median([s['rejected'] for s in stats])
    print(f'\n=== {n} frames ===')
    if not seen:
        print('No marker decoded in any dictionary.')
    for (name, i), sides in sorted(seen.items()):
        print(f'  {name:20s} ID {i:2d}: seen in {len(sides) * 100 // n:3d}% of frames, '
              f'side {np.median(sides):4.0f} px')
    print(f'  undecoded square candidates (4X4): {rej:.0f} per frame')
    print(f'  sharpness {sharp:.0f} (under ~60 = blurry)   brightness {bright:.0f}/255   '
          f'glare {glare:.1f}% of pixels saturated')
    tips = []
    if any(n_ != 'DICT_4X4_50' for n_, _ in seen) and ('DICT_4X4_50', want) not in seen:
        tips.append(f'Your marker is from ANOTHER dictionary ({next(iter(seen))[0]}): '
                    f'use a DICT_4X4_50 marker (ID {want})')
    if any(n_ == 'DICT_4X4_50' and i != want for n_, i in seen) and ('DICT_4X4_50', want) not in seen:
        ids = sorted(i for n_, i in seen if n_ == 'DICT_4X4_50')
        tips.append(f'Marker is DICT_4X4_50 but ID {ids}: run handeye_calib.py with --marker-id {ids[0]}')
    sides = [s for (n_, _), v in seen.items() for s in v]
    if sides and np.median(sides) < 40:
        tips.append('Marker is small in the image (<40 px): move the camera closer (35-50 cm) or use the 40 mm marker')
    if not seen and rej > 0:
        tips.append('Squares are found but not decoded: check the 5 mm WHITE BORDER all around, flat (not bent) '
                    'paper, matte print, no glare, and hold the arm still')
    if sharp < 60:
        tips.append('Image is blurry: hold the arm still, add light (short exposure), clean the lens')
    if bright < 60:
        tips.append('Image is dark: add light; dark images use long exposure -> motion blur')
    if glare > 2:
        tips.append('Strong glare: tilt the marker or move the lamp so it does not reflect into the camera')
    for t in tips:
        print('  -> ' + t)
    if not tips and ('DICT_4X4_50', want) in seen:
        print('  -> Looks fine here. If it drops out at some poses, the marker is turned more than ~60 deg away from the camera.')


def depth_check(marker_size, seconds=8):
    '''Compare the marker distance from solvePnP (uses marker_size) with the depth sensor (does not).
    ratio = depth / PnP. ~1.00: marker size and camera are right. ~0.90: PnP says 10% too far.'''
    import pyrealsense2 as rs
    from handeye_calib import find_marker
    pipe, K, dist, intr = open_camera(depth=True)
    align, det = rs.align(rs.stream.color), make_detector()
    ratios, t0 = [], time.time()
    print(f'Hold the marker still in view, about 30-45 cm from the camera, for {seconds} s...')
    try:
        while time.time() - t0 < seconds:
            f = align.process(pipe.wait_for_frames())
            img = np.asanyarray(f.get_color_frame().get_data()).copy()
            c, _ = find_marker(img, det, MARKER_ID, marker_size, K, dist)
            if c is None:
                continue
            u, v = (K[:2, :2] @ (c[:2] / c[2]) + K[:2, 2]).astype(int)
            dep = f.get_depth_frame()
            zs = [dep.get_distance(x, y) for y in range(v - 2, v + 3) for x in range(u - 2, u + 3)
                  if 0 <= x < dep.get_width() and 0 <= y < dep.get_height()]
            zs = [z for z in zs if z > 0]
            if len(zs) > 5:
                ratios.append(np.median(zs) / c[2])      # depth z vs PnP z (both along the optical axis)
    finally:
        pipe.stop()
    if len(ratios) < 10:
        print(f'Only {len(ratios)} good frames: marker not seen or no depth on it. Try closer / more light.')
        return
    r = float(np.median(ratios))
    print(f'{len(ratios)} frames: depth / PnP distance = {r:.3f}  (spread {np.std(ratios):.3f})')
    print(f'  -> the marker size that matches the depth sensor: {marker_size * r:.4f} m')
    if abs(r - 1) < 0.03:
        print('  -> camera and --marker-size agree: the 10% error comes from the ROBOT side (servo calibration).')
    else:
        print('  -> camera side is off: wrong --marker-size, a bent/curved marker, or wrong intrinsics.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--snap', type=float, default=0, help='seconds to watch without a window')
    ap.add_argument('--depth', type=float, metavar='MARKER_SIZE',
                    help='compare PnP distance with the depth sensor, e.g. --depth 0.0298')
    a = ap.parse_args()
    if a.depth:
        depth_check(a.depth)
        return
    os.makedirs(os.path.join(HERE, 'debug'), exist_ok=True)
    pipe, K, dist, _ = open_camera()
    detectors = {n: make_detector(d) for n, d in DICTS.items()}
    stats, best, t0, t_rep = [], (-1, None), time.time(), time.time()
    try:
        while True:
            img = np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data()).copy()
            s = analyse(img, detectors)
            stats.append(s)
            if s['rejected'] + len(s['found']) > best[0]:
                best = (s['rejected'] + len(s['found']), img.copy())
            view = img.copy()
            for c in s['rej_boxes']:
                cv2.polylines(view, [c.astype(int)], True, (0, 0, 255), 1)
            for name, i, c in s['boxes']:
                cv2.polylines(view, [c.astype(int)], True, (0, 255, 0), 2)
                cv2.putText(view, f'{name[5:]} id{i}', tuple(c[0][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
            cv2.putText(view, f'green = decoded, red = square found but NOT decoded | sharp {s["sharp"]:.0f}',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            if a.snap:
                if time.time() - t0 > a.snap:
                    break
            else:
                cv2.imshow('marker_check (q quit)', view)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                if time.time() - t_rep > 3:
                    advice(stats[-45:], len(stats[-45:]))
                    t_rep = time.time()
        cv2.imwrite(os.path.join(HERE, 'debug', 'last.jpg'), view)
        if best[1] is not None:
            cv2.imwrite(os.path.join(HERE, 'debug', 'most_candidates.jpg'), best[1])
        advice(stats, len(stats))
        print('snapshots saved in', os.path.join(HERE, 'debug'))
    finally:
        pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
