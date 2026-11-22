'''Record the arm's HOME pose: put the arm where you want it by hand, press Enter, it is saved.

Run:  python record_home.py            torque off: hold the arm, move it by hand, Enter = record
      python record_home.py --current  record the pose the arm holds right now (torque stays on)
      python record_home.py --test     after recording: move 3 cm up and back to the new home
      python record_home.py --show     print the saved home (arm not needed)
      python record_home.py --reset    delete the saved home (back to the built-in default)
      python record_home.py --dry      fake arm (to try the program)

Saved to home_pose.json (servo degrees). Every script uses it: the 'h' key in keyboard_control.py,
verify_click.py, camera_pick.py and goto.py, and the end of each pick_place.py run.
The previous file is kept in debug/home_pose_<time>.json.

A good home: gripper 10-20 cm above the table, out of the camera's way, not near a joint limit,
and a pose the arm can hold without straining (elbow bent, gripper pointing down/forward).
'''
import argparse
import json
import os
import select
import shutil
import sys
import time
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import so101_real                                                    # noqa: E402
from so101_real import JOINTS, HOME_FILE, HOME_DEFAULT              # noqa: E402
from keyboard_control import tcp, pitch_of                           # noqa: E402

MIN_TIP_Z = 0.05       # m: warn if home puts the gripper lower than this
EDGE = 8.0             # deg: warn if a joint is this close to its calibrated end


def describe(model, d, q_sim):
    '''Text: grip-centre position and tilt for sim joint angles.'''
    p, axis = tcp(model, d, q_sim)
    return f'grip centre x {p[0]*1000:.0f}  y {p[1]*1000:.0f}  z {p[2]*1000:.0f} mm, pitch {pitch_of(axis):.0f} deg', p


def show_saved(model, d):
    if not os.path.exists(HOME_FILE):
        print(f'No home_pose.json: the default home is used: {np.degrees(HOME_DEFAULT).round(1)} (sim deg)')
        print('  ' + describe(model, d, HOME_DEFAULT)[0])
        return
    h = json.load(open(HOME_FILE))
    print(f'Saved home ({h.get("recorded", "?")}):')
    print('  servo deg: ' + '  '.join(f'{j} {h["real_deg"][j]:.1f}' for j in JOINTS))
    print('  ' + describe(model, d, so101_real.load_home())[0])


def checks(model, d, real_deg, lo, hi):
    '''Warnings for a candidate home pose (list of strings).'''
    warn = []
    for j, v, a, b in zip(JOINTS, real_deg, lo, hi):
        if v < a + EDGE or v > b - EDGE:
            warn.append(f'{j} is {v:.0f} deg, within {EDGE:.0f} deg of its limit ({a:.0f}..{b:.0f})')
    q = so101_real.real_to_sim(real_deg)
    d.qpos[:5] = q
    mujoco.mj_kinematics(model, d)
    low = min(d.geom_xpos[model.geom(g).id][2] for g in ['fixed_jaw_sph_tip1', 'moving_jaw_sph_tip1'])
    if low < MIN_TIP_Z:
        warn.append(f'the fingertips are only {low*1000:.0f} mm above the table (moves to home could hit objects)')
    return warn


def key_pressed():
    return select.select([sys.stdin], [], [], 0)[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--current', action='store_true', help='record the pose held now, no torque off')
    ap.add_argument('--test', action='store_true', help='after recording: go 3 cm up and back to home')
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--reset', action='store_true')
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--port', default=so101_real.PORT)
    ap.add_argument('--robot-id', default=so101_real.ROBOT_ID)
    a = ap.parse_args()
    so101_real.configure(a.port, a.robot_id)
    model = mujoco.MjModel.from_xml_path(so101_real.SCENE)
    d = mujoco.MjData(model)

    if a.show:
        show_saved(model, d)
        return
    if a.reset:
        if os.path.exists(HOME_FILE):
            os.makedirs(os.path.join(HERE, 'debug'), exist_ok=True)
            bak = os.path.join(HERE, 'debug', time.strftime('home_pose_%Y%m%d_%H%M%S.json'))
            shutil.move(HOME_FILE, bak)
            print(f'home_pose.json moved to {bak}: the default home is used again.')
        else:
            print('No home_pose.json: already using the default home.')
        return

    show_saved(model, d)
    arm = so101_real.RealArm(dry=a.dry)
    lo, hi = arm.lo - so101_real.MARGIN, arm.hi + so101_real.MARGIN     # full calibrated range
    bus = None if a.dry else arm.robot.bus
    try:
        if not a.current:
            input('\nHOLD THE ARM with one hand (it goes limp), then press Enter to turn the torque off...')
            if bus:
                bus.disable_torque()
            print('Torque OFF. Move the arm to the home pose by hand. Press Enter to record it '
                  '(Ctrl+C = cancel).\n')
            while True:
                deg, g = arm.read_real()
                text = describe(model, d, so101_real.real_to_sim(deg))[0]
                print('\r  servo deg ' + ' '.join(f'{v:6.1f}' for v in deg) + f'  grip {g:4.0f}  | {text}   ',
                      end='', flush=True)
                if key_pressed():
                    sys.stdin.readline()
                    break
                time.sleep(0.1)
            print()
        reads = [arm.read_real() for _ in range(10)]                  # average 10 reads
        deg = np.mean([r[0] for r in reads], 0)
        grip = float(np.mean([r[1] for r in reads]))

        if not a.current:                     # hold here: goal = where it is, then torque back on
            arm.last_deg, arm.last_grip = deg.copy(), grip
            action = {f'{j}.pos': float(v) for j, v in zip(JOINTS, deg)}
            action['gripper.pos'] = grip
            arm.robot.send_action(action)
            if bus:
                bus.enable_torque()
            print('Torque ON: the arm holds this pose. You can let go.')

        q = so101_real.real_to_sim(deg)
        text, p = describe(model, d, q)
        print('\nRecorded home:')
        print('  servo deg: ' + '  '.join(f'{j} {v:.1f}' for j, v in zip(JOINTS, deg)) + f'   gripper {grip:.0f}')
        print('  ' + text)
        for w in checks(model, d, deg, lo, hi):
            print('  WARNING: ' + w)
        ans = input('Save it as the home pose? [Y/n] ').strip().lower()
        if ans not in ('', 'y', 'yes'):
            print('Not saved.')
            return
        if os.path.exists(HOME_FILE):
            os.makedirs(os.path.join(HERE, 'debug'), exist_ok=True)
            shutil.copy(HOME_FILE, os.path.join(HERE, 'debug', time.strftime('home_pose_%Y%m%d_%H%M%S.json')))
        json.dump({'real_deg': {j: round(float(v), 2) for j, v in zip(JOINTS, deg)},
                   'gripper': round(grip, 1),
                   'sim_deg': [round(float(v), 2) for v in np.degrees(q)],
                   'grip_centre_mm': [round(float(v) * 1000, 1) for v in p],
                   'recorded': time.strftime('%Y-%m-%d %H:%M:%S'),
                   'note': 'home pose, servo degrees; recorded by record_home.py'},
                  open(HOME_FILE, 'w'), indent=1)
        print(f'Saved {HOME_FILE}')

        if a.test:                            # up 3 cm with IK, then back to home, both smooth
            from keyboard_control import ik
            from pick_place import smooth
            r_lo, r_hi = so101_real.real_limits_sim()
            q_up, err = ik(model, d, q, p + [0, 0, 0.03], pitch_of(tcp(model, d, q)[1]), True, r_lo, r_hi)
            if err > 0.004:
                print('test skipped: cannot move 3 cm up from this pose')
            else:
                print('test: 3 cm up, then back to home...')
                g_sim = so101_real.real_gripper_to_sim(grip)
                for q0, q1 in [(q, q_up), (q_up, q)]:
                    for k in range(1, 46):
                        arm.send_sim(np.append(q0 + smooth(k / 45) * (q1[:5] - q0), g_sim))
                        time.sleep(1 / so101_real.RATE)
                time.sleep(0.5)
                back, _ = arm.read_real()
                print(f'  back at home within {np.abs(back - deg).max():.1f} deg')
    except KeyboardInterrupt:
        print('\nCancelled: nothing saved.')
        if bus and not a.current:
            deg, g = arm.read_real()                 # hold where it is before torque on
            action = {f'{j}.pos': float(v) for j, v in zip(JOINTS, deg)}
            action['gripper.pos'] = float(g)
            arm.robot.send_action(action)
            bus.enable_torque()
    finally:
        arm.close()


if __name__ == '__main__':
    main()
