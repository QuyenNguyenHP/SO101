'''Small, slow gripper test using raw servo units (works before calibration).

Moves the gripper +STEP ticks, back, -STEP ticks, back. 4096 ticks = 360 deg, so 150 ~ 13 deg.
Run:  python Reference/test_gripper.py        or   python Reference/test_gripper.py 300
'''
import sys
import time
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

PORT = '/dev/ttyACM0'
STEP = int(sys.argv[1]) if len(sys.argv) > 1 else 150
TORQUE = 400          # 0..1000: limit force to 40% so it can't squeeze hard

bus = SO101Follower(SO101FollowerConfig(port=PORT, id='test')).bus
bus._connect(handshake=False)

def pos():
    return bus.read('Present_Position', 'gripper', normalize=False, num_retry=2)

def move_to(goal, seconds=1.5):
    start = pos()
    n = int(seconds * 30)
    for k in range(1, n + 1):
        bus.write('Goal_Position', 'gripper', int(start + (goal - start) * k / n), normalize=False)
        time.sleep(1 / 30)
    time.sleep(0.5)
    p = pos()
    print(f'  goal {goal:5d} -> reached {p:5d} (error {p - goal:+d})')

try:
    home = pos()
    print('start position', home)
    bus.write('Torque_Limit', 'gripper', TORQUE, normalize=False)
    bus.write('Goal_Position', 'gripper', home, normalize=False)   # hold where it is
    bus.write('Torque_Enable', 'gripper', 1, normalize=False)
    for goal in [home + STEP, home, home - STEP, home]:
        move_to(goal)
finally:
    bus.write('Torque_Enable', 'gripper', 0, normalize=False)
    bus._close()
    print('torque off')
