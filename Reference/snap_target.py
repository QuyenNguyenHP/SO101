'''Take reference photos of an object for hand_over.py (saved in targets/).

  python Reference/snap_target.py              one snapshot: every object on the table (from the depth image) is
                                     cut out to targets/candidates/obj_N.jpg, overview in
                                     debug/snap_overview.jpg (numbered boxes)
  python Reference/snap_target.py --keep 2 cube    keep candidate 2 as targets/cube.jpg (others are not used)
  python Reference/snap_target.py --live       camera window: click an object's box (or drag a rectangle
                                     round it) to save it to targets/; q = quit
The arm should be out of the camera's view (home), it is not masked out here.
'''
import argparse
import glob
import os
import shutil
import sys
import time
import numpy as np
import cv2

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from detector import depth_blobs                      # noqa: E402
from hand_over import Target, TARGET_DIR             # noqa: E402

CAND = os.path.join(TARGET_DIR, 'candidates')


def grab(pipe, align, intr, T, off, n=1):
    from camera_pick import cloud_base
    for _ in range(n):
        fs = align.process(pipe.wait_for_frames())
    img = np.asanyarray(fs.get_color_frame().get_data()).copy()
    P = cloud_base(fs.get_depth_frame(), intr, T, off)
    boxes = []
    for blob, _ in depth_blobs(P):
        ys, xs = np.nonzero(blob)
        boxes.append((int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())))
    return img, sorted(boxes)


def draw(img, boxes):
    out = img.copy()
    for i, (x0, y0, x1, y1) in enumerate(boxes, 1):
        cv2.rectangle(out, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(out, str(i), (x0, max(14, y0 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--keep', nargs=2, metavar=('N', 'NAME'))
    ap.add_argument('--live', action='store_true')
    a = ap.parse_args()
    os.makedirs(CAND, exist_ok=True)
    if a.keep:
        src = os.path.join(CAND, f'obj_{int(a.keep[0])}.jpg')
        if not os.path.exists(src):
            sys.exit(f'{src} not found: run python Reference/snap_target.py first')
        dst = os.path.join(TARGET_DIR, a.keep[1] + '.jpg')
        shutil.copy(src, dst)
        print(f'saved {dst}: hand_over.py uses every photo in targets/ (not the candidates folder)')
        return

    import pyrealsense2 as rs
    from handeye_calib import open_camera
    from verify_click import load_offset
    T, off = np.load(os.path.join(HERE, 'T_base_cam.npy')), load_offset()
    pipe, _, _, intr = open_camera(depth=True)
    align = rs.align(rs.stream.color)
    try:
        if not a.live:
            img, boxes = grab(pipe, align, intr, T, off, n=20)        # let the exposure settle
            for f in glob.glob(os.path.join(CAND, 'obj_*.jpg')):
                os.remove(f)
            for i, b in enumerate(boxes, 1):
                cv2.imwrite(os.path.join(CAND, f'obj_{i}.jpg'), Target.crop(img, b))
            os.makedirs(os.path.join(HERE, 'debug'), exist_ok=True)
            cv2.imwrite(os.path.join(HERE, 'debug', 'snap_full.jpg'), img)
            cv2.imwrite(os.path.join(HERE, 'debug', 'snap_overview.jpg'), draw(img, boxes))
            print(f'{len(boxes)} objects: targets/candidates/obj_1..{len(boxes)}.jpg, '
                  'overview debug/snap_overview.jpg')
            print('keep one:  python Reference/snap_target.py --keep N name')
            return
        st = {'down': None, 'rect': None, 'click': None}

        def mouse(ev, x, y, *_):
            if ev == cv2.EVENT_LBUTTONDOWN:
                st['down'], st['rect'] = (x, y), None
            elif ev == cv2.EVENT_MOUSEMOVE and st['down']:
                st['rect'] = (*st['down'], x, y)
            elif ev == cv2.EVENT_LBUTTONUP and st['down']:
                x0, y0 = st['down']
                st['click'] = (min(x0, x), min(y0, y), max(x0, x), max(y0, y))
                st['down'] = st['rect'] = None
        win = 'snap_target (click a box or drag a rectangle to save | q quit)'
        cv2.namedWindow(win)
        cv2.setMouseCallback(win, mouse)
        msg = 'click an object box, or drag a rectangle round the object'
        while True:
            img, boxes = grab(pipe, align, intr, T, off)
            if st['click']:
                x0, y0, x1, y1 = st['click']
                st['click'] = None
                if x1 - x0 > 10 and y1 - y0 > 10:                  # dragged: that rectangle
                    crop = img[y0:y1, x0:x1]
                else:
                    hit = [b for b in boxes if b[0] <= x0 <= b[2] and b[1] <= y0 <= b[3]]
                    crop = Target.crop(img, hit[0]) if hit else None
                if crop is None:
                    msg = 'no box there: click inside a green box or drag a rectangle'
                else:
                    p = os.path.join(TARGET_DIR, time.strftime('snap_%Y%m%d_%H%M%S.jpg'))
                    cv2.imwrite(p, crop)
                    msg = f'saved {os.path.relpath(p, HERE)}'
                    print(msg)
            view = draw(img, boxes)
            if st['rect']:
                cv2.rectangle(view, st['rect'][:2], st['rect'][2:], (0, 200, 255), 2)
            cv2.rectangle(view, (0, 0), (view.shape[1], 28), (0, 0, 0), -1)
            cv2.putText(view, msg[:100], (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imshow(win, view)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
