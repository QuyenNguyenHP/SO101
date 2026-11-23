'''Find and name the objects on the table: several objects at once, each with a name.

Two ways (backend):
  'depth' (default)  every object that sticks out of the table in the DEPTH image is found
                     (the arm itself is masked out using its joint angles), then CLIP names each
                     one from a crop of the colour image. Reliable outlines, works offline on the CPU.
  'yolo'             YOLO-World on the colour image only (open vocabulary). Weak on small objects
                     seen from above with the 640x480 USB 2 camera image; kept for comparison.
Both use the same CLIP text encoder (~/.cache/clip/ViT-B-32.pt), so names are free text.

    from detector import Detector, robot_mask
    det = Detector(['green cube', 'tape measure'])
    found = det.detect(bgr, P, robot=robot_mask(...))
    # [{'name', 'conf', 'box': (x0, y0, x1, y1), 'blob': bool mask or None, 'table': z}, ...]

Names: short and visual ("green cube", "white cube", "black cylinder", "tape measure").
Objects that match NONE of the names well are still listed, as "? (best guess)".

Quick test on one camera frame:  python Reference/detector.py "green cube, white cube, tape measure"
'''
import os
import numpy as np
import cv2

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YOLO_MODEL = os.path.join(HERE, 'models', 'yolov8s-worldv2.pt')
DISTRACT = ['robot arm', 'empty table']   # CLIP extra classes: blobs that look like these are dropped
TEMPLATES = ['a photo of a {}.', 'a top-down photo of a {} on a table.', 'a close-up photo of a {}.']
MIN_AREA = 120          # px: smaller blobs are noise
OBJ_MIN = 0.008         # m above the table
MAX_TALL = 0.15         # m: anything taller is not a table object (arm, person...)
REACH = (0.10, 0.42)    # m from the base: only look at this ring of the table
UNSURE = 0.35           # CLIP probability under this: shown as "? name"


def robot_mask(model, d, q, T, offset, intr, shape, dilate=12):
    '''Image mask of the arm: robot meshes posed with joint angles q (sim rad), projected into
    the camera (T = base <- camera, offset = XY correction). Grown by `dilate` px for safety.'''
    import mujoco
    d.qpos[:len(q)] = q
    mujoco.mj_kinematics(model, d)
    R, t = T[:3, :3], T[:3, 3] + offset
    mask = np.zeros(shape, np.uint8)
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or model.geom_bodyid[g] == 0:
            continue
        mid = model.geom_dataid[g]
        v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid] + model.mesh_vertnum[mid]][::15]
        world = d.geom_xpos[g] + v @ d.geom_xmat[g].reshape(3, 3).T
        cam = (world - t) @ R                                   # base -> camera frame
        cam = cam[cam[:, 2] > 0.05]
        if len(cam) < 3:
            continue
        uv = np.stack([cam[:, 0] / cam[:, 2] * intr.fx + intr.ppx, cam[:, 1] / cam[:, 2] * intr.fy + intr.ppy], 1)
        hull = cv2.convexHull(uv.astype(np.float32)).astype(np.int32)
        cv2.fillConvexPoly(mask, hull, 1)
    if dilate:
        mask = cv2.dilate(mask, np.ones((dilate * 2 + 1, dilate * 2 + 1), np.uint8))
    return mask.astype(bool)


def table_height(P):
    '''Table z (m) from the depth points in the robot's reach ring.'''
    z = P[..., 2]
    r = np.hypot(P[..., 0], P[..., 1])
    ws = np.isfinite(z) & (r > REACH[0]) & (r < REACH[1]) & (np.abs(z) < 0.03)
    return float(np.median(z[ws])) if ws.sum() > 200 else 0.0


def depth_blobs(P, robot=None):
    '''Everything that sticks out of the table: list of (bool mask, table z).'''
    z = P[..., 2]
    table = table_height(P)
    r = np.hypot(P[..., 0], P[..., 1])
    m = np.isfinite(z) & (r > REACH[0]) & (r < REACH[1]) & (z > table + OBJ_MIN) & (z < table + MAX_TALL)
    if robot is not None:
        m &= ~robot
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    return [(lab == k, table) for k in range(1, n) if stats[k, cv2.CC_STAT_AREA] >= MIN_AREA]


class Detector:
    def __init__(self, names, backend='depth', conf=0.15):
        self.names = [n.strip() for n in names if n.strip()]
        if not self.names:
            raise SystemExit('give at least one object name, e.g. --detect "green cube, cup"')
        self.backend, self.conf = backend, conf
        if backend == 'yolo':
            from ultralytics import YOLOWorld
            if not os.path.exists(YOLO_MODEL):
                raise SystemExit(f'{YOLO_MODEL} not found (YOLO-World weights).')
            self.yolo = YOLOWorld(YOLO_MODEL)
            self.yolo.set_classes(self.names)
        else:
            import torch
            import clip
            self.torch = torch
            self.clip, self.pre = clip.load('ViT-B/32', device='cpu')   # cached in ~/.cache/clip
            labels = self.names + DISTRACT
            with torch.no_grad():
                f = torch.stack([self.clip.encode_text(clip.tokenize([t.format(n) for t in TEMPLATES])).float().mean(0)
                                 for n in labels])
            self.text = f / f.norm(dim=-1, keepdim=True)

    def name_crop(self, bgr, box):
        '''CLIP: (best name, probability) for the image around box; name None = arm / table.'''
        from PIL import Image
        x0, y0, x1, y1 = box
        s = int(max(x1 - x0, y1 - y0) * 1.6) + 16          # square crop with some context
        cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
        crop = bgr[max(0, cy - s // 2):cy + s // 2, max(0, cx - s // 2):cx + s // 2, ::-1]
        with self.torch.no_grad():
            f = self.clip.encode_image(self.pre(Image.fromarray(np.ascontiguousarray(crop))).unsqueeze(0)).float()
            f /= f.norm(dim=-1, keepdim=True)
            p = (100 * f @ self.text.T).softmax(-1)[0].numpy()
        k = int(p.argmax())
        return (self.names[k] if k < len(self.names) else None), float(p[k])

    def detect(self, bgr, P=None, robot=None):
        out = []
        if self.backend == 'yolo':
            r = self.yolo.predict(bgr, imgsz=640, conf=self.conf, iou=0.5, agnostic_nms=True, verbose=False)[0]
            for xyxy, c, k in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy(),
                                  r.boxes.cls.cpu().numpy().astype(int)):
                out.append({'name': self.names[k], 'conf': float(c), 'blob': None, 'table': None,
                            'box': tuple(int(round(v)) for v in xyxy)})
        else:
            for blob, table in depth_blobs(P, robot):
                ys, xs = np.nonzero(blob)
                box = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
                name, p = self.name_crop(bgr, box)
                if name is None:                                # looks like the arm or bare table
                    continue
                out.append({'name': name if p >= UNSURE else f'? {name}', 'conf': p, 'box': box,
                            'blob': blob, 'table': table})
        return sorted(out, key=lambda d: d['box'][0])          # left to right in the image


if __name__ == '__main__':                                       # quick test on one camera frame
    import sys
    import time
    import mujoco
    import pyrealsense2 as rs
    sys.path.insert(0, HERE)
    import so101_real
    from handeye_calib import open_camera
    from camera_pick import cloud_base
    from verify_click import load_offset
    names = (sys.argv[1] if len(sys.argv) > 1 else 'cube, box, bottle, cup').split(',')
    backend = sys.argv[2] if len(sys.argv) > 2 else 'depth'
    det = Detector(names, backend)
    T, off = np.load(os.path.join(HERE, 'T_base_cam.npy')), load_offset()
    model = mujoco.MjModel.from_xml_path(so101_real.SCENE)
    pipe, _, _, intr = open_camera(depth=True)
    align = rs.align(rs.stream.color)
    try:
        for _ in range(15):
            fs = align.process(pipe.wait_for_frames())
        img = np.asanyarray(fs.get_color_frame().get_data()).copy()
        P = cloud_base(fs.get_depth_frame(), intr, T, off)
    finally:
        pipe.stop()
    q = None            # the test does not connect the arm (connecting/disconnecting changes torque);
    if len(sys.argv) > 3:   # give its joint angles (deg, 5 joints) to mask it:  ... "names" depth "0 -90 90 70 0"
        q = so101_real.real_to_sim(np.array([float(x) for x in sys.argv[3].split()]))
    robot = robot_mask(model, mujoco.MjData(model), q, T, off, intr, img.shape[:2]) if q is not None else None
    t = time.time()
    found = det.detect(img, P, robot)
    print(f'{len(found)} found in {time.time() - t:.2f} s')
    if robot is not None:
        img[robot] = (img[robot] * 0.5).astype(np.uint8)
    for i, d in enumerate(found, 1):
        print(f'  {i}. {d["name"]:18s} {d["conf"]:.2f}  box {d["box"]}')
        x0, y0, x1, y1 = d['box']
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(img, f'{i} {d["name"]} {d["conf"]:.2f}', (x0, max(12, y0 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1)
    os.makedirs(os.path.join(HERE, 'debug'), exist_ok=True)
    cv2.imwrite(os.path.join(HERE, 'debug', 'detect.jpg'), img)
    print('image saved: debug/detect.jpg (arm mask darkened)')
