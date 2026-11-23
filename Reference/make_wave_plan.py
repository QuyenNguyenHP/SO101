'''Turn wave_hello.py into a plan file for the real arm: wave_plan.npy (N x 6, radians, 30 Hz).

Same motion as ~/robot/mujoco_menagerie/wave_hello.py, plus the gripper as a 6th column.
Differences for the real arm: the wave fades in and out over FADE seconds (no sudden start/stop),
and you can shorten it.
Run:  python Reference/make_wave_plan.py          (10 s wave)
      python Reference/make_wave_plan.py 20       (20 s, like the sim)
Then: python Reference/real_play.py npy/wave_plan.npy
'''
import os
import sys
import numpy as np
import mujoco

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
RATE = 30
FADE = 1.5          # seconds to fade the wave in and out
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.expanduser('~/robot/mujoco_menagerie/robotstudio_so101/scene.xml')

model = mujoco.MjModel.from_xml_path(SCENE)
lo = model.actuator_ctrlrange[:, 0]
hi = model.actuator_ctrlrange[:, 1]
mid = (lo + hi) / 2

plan = []
for k in range(int(SECONDS * RATE) + 1):
    t = k / RATE
    a = min(1.0, t / FADE, (SECONDS - t) / FADE)       # 0 -> 1 -> 0 envelope
    a = a * a * (3 - 2 * a)                             # smooth the envelope
    u = mid.copy()
    u[1] = mid[1] - 0.6                                 # raise the shoulder (same as sim)
    u[2] = mid[2] + a * 0.4 * np.sin(2 * t)             # elbow swings
    u[3] = mid[3] + a * 0.5 * np.sin(2 * t + 1.0)       # wrist swings, out of phase
    u[5] = mid[5] + a * 0.3 * np.sin(4 * t)             # gripper opens/closes
    plan.append(np.clip(u, lo, hi))
plan = np.array(plan)

jump = np.degrees(np.abs(np.diff(plan, axis=0)).max())
print(f'{len(plan)} commands ({len(plan) / RATE:.1f} s at 30 Hz) | biggest step {jump:.2f} deg')
print('joint min (deg):', np.round(np.degrees(plan.min(0)), 1))
print('joint max (deg):', np.round(np.degrees(plan.max(0)), 1))
out = os.path.join(HERE, 'npy', 'wave_plan.npy')
os.makedirs(os.path.dirname(out), exist_ok=True)
np.save(out, plan)
print('saved', out)
