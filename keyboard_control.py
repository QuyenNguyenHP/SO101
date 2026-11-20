'''Drive the SO101 gripper TIP with the keyboard: IK in MuJoCo, the real arm follows.

Run:   python keyboard_control.py          (real arm)
       python keyboard_control.py --dry    (no arm: sim only)
Keys are read from THIS TERMINAL: keep it focused (click it), watch the MuJoCo window.

  Arrow Up / Down     tip forward / back        (+X / -X, the way the arm points at zero)
  Arrow Left / Right  tip left / right          (+Y / -Y, seen from behind the robot)
  w / s               tip up / down             (+Z / -Z)
  i / k               tilt gripper nose up / down
  j / l               wrist roll left / right
  o / c               open / close gripper a bit     space  toggle fully open / closed
  p                   pitch lock on/off (on: gripper keeps its tilt while moving)
  1 2 3               step size 2 / 5 / 10 mm
  h                   go to home pose               q  quit
'''
import os
import select
import sys
import termios
import time
import tty
import numpy as np
import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from so101_real import RealArm, SCENE, RATE, G_LO, G_HI, real_limits_sim   # noqa: E402

SITE = 'gripperframe'            # the fingertip point in so101.xml
HOME = np.radians([0, -20, 30, 40, 0])
STEPS_MM = {'1': 2, '2': 5, '3': 10}
Z_MIN = 0.01                     # never put the tip lower than 1 cm above the table
REACH_TOL = 0.003                # IK must land within 3 mm, else the move is refused
GRIP_OPEN, GRIP_CLOSED = 1.2, -0.1


# ---------------- IK on the fingertip ----------------
def tcp(model, d, q):
    '''Tip position and approach axis (unit vector out of the jaws) for joint angles q.'''
    d.qpos[:5] = q[:5]
    mujoco.mj_kinematics(model, d)
    sid = model.site(SITE).id
    return d.site_xpos[sid].copy(), d.site_xmat[sid].reshape(3, 3)[:, 0].copy()


def pitch_of(axis):
    '''Degrees the gripper points below horizontal (90 = straight down).'''
    return np.degrees(np.arcsin(np.clip(-axis[2], -1, 1)))


def ik(model, d, q, target, pitch_deg, lock_pitch, lo, hi, iters=60, damping=0.01):
    '''Move joints 0-3 so the tip reaches `target` (and the tilt `pitch_deg` if locked).
    Joint 4 (wrist roll) and the gripper are left as they are. Returns (q, position error).'''
    q = q.copy()
    sid = model.site(SITE).id
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    az_target = -np.sin(np.radians(pitch_deg))
    for _ in range(iters):
        pos, axis = tcp(model, d, q)
        mujoco.mj_comPos(model, d)
        mujoco.mj_jacSite(model, d, jacp, jacr, sid)
        err = target - pos
        J = jacp[:, :4]
        if lock_pitch:                          # 4th row: z-component of the approach axis
            dz = np.array([np.cross(jacr[:, i], axis)[2] for i in range(4)])
            J = np.vstack([J, 0.1 * dz])        # 0.1: 1 rad of tilt "costs" like 10 cm
            err = np.append(err, 0.1 * (az_target - axis[2]))
        if np.linalg.norm(err[:3]) < 1e-4 and (not lock_pitch or abs(err[3]) < 1e-4):
            break
        dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(len(err)), err)
        q[:4] = np.clip(q[:4] + dq, lo[:4], hi[:4])
    pos, _ = tcp(model, d, q)
    return q, np.linalg.norm(target - pos)


# ---------------- keyboard (terminal, non-blocking) ----------------
ARROWS = {'\x1b[A': 'UP', '\x1b[B': 'DOWN', '\x1b[C': 'RIGHT', '\x1b[D': 'LEFT'}


def read_keys():
    '''All keys pressed since last call (arrow keys come as 3-byte sequences).'''
    keys = []
    while select.select([sys.stdin], [], [], 0)[0]:
        ch = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
        if ch == '\x1b':                        # arrow key: ESC [ A/B/C/D
            seq = ch + os.read(sys.stdin.fileno(), 2).decode(errors='ignore')
            keys.append(ARROWS.get(seq, ''))
        else:
            keys.append(ch)
    return keys


def main():
    dry = '--dry' in sys.argv
    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)
    d_ik = mujoco.MjData(model)
    ctrl_lo, ctrl_hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
    r_lo, r_hi = real_limits_sim()              # calibrated real limits, in sim radians
    lo = np.maximum(ctrl_lo[:5], r_lo)
    hi = np.minimum(ctrl_hi[:5], r_hi)

    arm = RealArm(dry=dry)
    q = arm.read_sim()                          # start exactly where the real arm is
    q[:5] = np.clip(q[:5], lo, hi)
    target, axis = tcp(model, d_ik, q)
    pitch = pitch_of(axis)
    lock_pitch, step, status, refused_until, quit_ = True, 5, 'ok          ', 0.0, False

    data.qpos[:6] = q
    data.ctrl[:] = np.clip(q, ctrl_lo, ctrl_hi)
    mujoco.mj_forward(model, data)

    print(__doc__.split('Keys are read')[1].split('\n', 1)[1])
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    steps_per_cmd = max(1, round(1 / RATE / model.opt.timestep))
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            t_next = time.perf_counter()
            while viewer.is_running() and not quit_:
                new_target, new_pitch, s = target.copy(), pitch, step / 1000
                for k in read_keys():
                    if k == 'q':
                        quit_ = True
                    elif k == 'UP':    new_target[0] += s
                    elif k == 'DOWN':  new_target[0] -= s
                    elif k == 'LEFT':  new_target[1] += s
                    elif k == 'RIGHT': new_target[1] -= s
                    elif k == 'w':     new_target[2] += s
                    elif k == 's':     new_target[2] -= s
                    elif k == 'i':     new_pitch -= 3
                    elif k == 'k':     new_pitch += 3
                    elif k == 'j':     q[4] = np.clip(q[4] + np.radians(3), lo[4], hi[4])
                    elif k == 'l':     q[4] = np.clip(q[4] - np.radians(3), lo[4], hi[4])
                    elif k == 'o':     q[5] = min(G_HI, q[5] + 0.1)
                    elif k == 'c':     q[5] = max(G_LO, q[5] - 0.1)
                    elif k == ' ':     q[5] = GRIP_CLOSED if q[5] > (GRIP_OPEN + GRIP_CLOSED) / 2 else GRIP_OPEN
                    elif k == 'p':     lock_pitch = not lock_pitch
                    elif k in STEPS_MM: step = STEPS_MM[k]
                    elif k == 'h':
                        q[:5] = HOME
                        new_target, axis = tcp(model, d_ik, q)
                        new_pitch = pitch_of(axis)
                new_target[2] = max(new_target[2], Z_MIN)
                new_pitch = float(np.clip(new_pitch, -30, 100))

                q_new, err = ik(model, d_ik, q, new_target, new_pitch, lock_pitch, lo, hi)
                if err < REACH_TOL:
                    q, target, pitch = q_new, new_target, new_pitch
                    if not lock_pitch:          # tilt follows whatever IK chose
                        pitch = pitch_of(tcp(model, d_ik, q)[1])
                else:
                    refused_until = time.perf_counter() + 1.0   # keep the last good pose
                status = 'OUT OF REACH' if time.perf_counter() < refused_until else 'ok          '

                # sim follows the IK solution, real arm too
                data.ctrl[:] = np.clip(q, ctrl_lo, ctrl_hi)
                for _ in range(steps_per_cmd):
                    mujoco.mj_step(model, data)
                arm.send_sim(q)

                # green ball = target
                viewer.user_scn.ngeom = 0
                mujoco.mjv_initGeom(viewer.user_scn.geoms[0], mujoco.mjtGeom.mjGEOM_SPHERE,
                                    np.array([0.008, 0, 0]), target, np.eye(3).flatten(),
                                    np.array([0.1, 0.9, 0.1, 0.6], dtype=np.float32))
                viewer.user_scn.ngeom = 1
                viewer.sync()

                sys.stdout.write(
                    f'\rtip x{target[0]*1000:6.0f} y{target[1]*1000:6.0f} z{target[2]*1000:6.0f} mm | '
                    f'tilt {pitch:5.1f}{" (locked)" if lock_pitch else " (free)  "} | '
                    f'grip {q[5]:5.2f} | step {step:2d} mm | {status}')
                sys.stdout.flush()
                t_next += 1 / RATE
                time.sleep(max(0.0, t_next - time.perf_counter()))
            viewer.close()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        print()
        arm.close()
    # The MuJoCo viewer can crash while Python shuts down; everything is already safe here.
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
