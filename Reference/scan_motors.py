'''Check which SO101 servos answer on the bus (IDs 1-6 at 1,000,000 baud).

Run:  python Reference/scan_motors.py              -> list IDs that answer
      python Reference/scan_motors.py fix shoulder_lift
                                        -> give the ONE connected servo the ID of that joint
'''
import sys
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

PORT = '/dev/ttyACM0'
NAMES = {1: 'shoulder_pan', 2: 'shoulder_lift', 3: 'elbow_flex',
         4: 'wrist_flex', 5: 'wrist_roll', 6: 'gripper'}

bus = SO101Follower(SO101FollowerConfig(port=PORT, id='scan')).bus

if len(sys.argv) == 3 and sys.argv[1] == 'fix':
    motor = sys.argv[2]
    assert motor in NAMES.values(), f'joint name must be one of {list(NAMES.values())}'
    input(f"Connect ONLY the '{motor}' servo to the board (with power on), then press Enter...")
    bus.setup_motor(motor)
    print(f"'{motor}' now has ID {bus.motors[motor].id}")
    sys.exit()

bus._connect(handshake=False)
try:
    for i, name in NAMES.items():
        ok = bus.ping(i, num_retry=3) is not None
        print(f'ID {i}  {name:<14} {"OK" if ok else "-- no answer"}')
finally:
    bus._close()
