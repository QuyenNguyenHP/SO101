'''Build a small square plan for the real arm (no viewer) and save it as square_plan.npy.

Same math as ~/robot/mujoco_menagerie/draw_square.py, but small and saved to a file.
Run:  python make_square_plan.py          (default 3 cm half-side = 6 cm square)
      python make_square_plan.py 0.05     (5 cm half-side = 10 cm square)
'''
import os
import sys
import numpy as np

MENAGERIE = os.path.expanduser('~/robot/mujoco_menagerie')
sys.path.insert(0, MENAGERIE)
from robot_utils import load, fk, ik, straight_line, smooth   # noqa: E402

SIZE = float(sys.argv[1]) if len(sys.argv) > 1 else 0.03   # half side (m)
STEPS = 60          # commands per side: 60 = 2 s at 30 Hz
STRAIGHT = True     # True: straight edges (IK each step). False: joint-space curves
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'square_plan.npy')

model, data, ee = load(os.path.join(MENAGERIE, 'robotstudio_so101', 'scene.xml'))
home = np.radians([0, -20, 30, 40, 0])
center = fk(model, ee, home)
corners = [center + np.array(c) for c in
           [(-SIZE, -SIZE, 0), (SIZE, -SIZE, 0), (SIZE, SIZE, 0), (-SIZE, SIZE, 0), (-SIZE, -SIZE, 0)]]

q, ok = ik(model, ee, corners[0], home)
plan = []
for a, b in zip(corners[:-1], corners[1:]):
    if STRAIGHT:
        seg = straight_line(model, ee, a, b, q, STEPS)
    else:
        q_b, ok_b = ik(model, ee, b, q)
        ok = ok and ok_b
        seg = [q + smooth(k / STEPS) * (q_b - q) for k in range(STEPS + 1)]
    plan += seg
    q = seg[-1]
plan = np.array(plan)

# Check: every point reached, and how far the tip is from the target corners
err = max(np.linalg.norm(fk(model, ee, p) - c) for p, c in zip(plan[::STEPS], corners))
jump = np.degrees(np.abs(np.diff(plan, axis=0)).max())
print(f'{len(plan)} commands ({len(plan) / 30:.1f} s at 30 Hz) | corners reached: {ok} | '
      f'max corner error {err * 1000:.1f} mm | biggest step {jump:.2f} deg')
print('joint min (deg):', np.round(np.degrees(plan.min(0)), 1))
print('joint max (deg):', np.round(np.degrees(plan.max(0)), 1))
np.save(OUT, plan)
print('saved', OUT)
