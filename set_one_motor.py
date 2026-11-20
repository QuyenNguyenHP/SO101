'''Give the NEWLY added servo its SO101 ID and 1,000,000 baud.

Works two ways:
  - only that servo is connected, or
  - you add servos one by one along the chain (1, then 2, then 3, ...):
    servos already set (lower IDs) are left alone, the one new servo gets set.

Run:  python set_one_motor.py shoulder_lift
Joint names: shoulder_pan(1) shoulder_lift(2) elbow_flex(3) wrist_flex(4) wrist_roll(5) gripper(6)
'''
import sys
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

PORT = '/dev/ttyACM0'
motor = sys.argv[1]

bus = SO101Follower(SO101FollowerConfig(port=PORT, id='setup')).bus
assert motor in bus.motors, f'joint name must be one of {list(bus.motors)}'
target = bus.motors[motor].id
done_ids = {m.id for m in bus.motors.values() if m.id < target}   # already set earlier
d = bus._definition(motor)
default = bus.default_baudrate

def clean_reads(name, n=10):
    ok, pos = 0, None
    for _ in range(n):
        try:
            pos = bus.read('Present_Position', name, normalize=False)
            ok += 1
        except Exception:
            pass
    return ok, pos

bus._connect(handshake=False)
try:
    # 1. Find every servo at every baud rate
    found = []
    for baud in d.baudrates:
        bus.set_baudrate(baud)
        found += [(baud, sid) for sid in bus._scan(d)]
    bus.set_baudrate(default)
    print('On the bus:', [f'ID {i} @ {b}' for b, i in found] or 'nothing')

    new = [(b, i) for b, i in found if not (b == default and i in done_ids)]

    # 2. Check the servos already set still answer cleanly (a duplicate ID garbles them)
    for name, m in bus.motors.items():
        if m.id in done_ids and any(i == m.id and b == default for b, i in found):
            ok, _ = clean_reads(name, 5)
            if ok < 5:
                sys.exit(f'STOP: ID {m.id} ({name}) gives garbled answers -> the new servo probably '
                         f'has the SAME ID {m.id}. Connect the new servo ALONE and run again.')

    if len(new) == 0:
        sys.exit('STOP: no new servo found. Check its cable/power, or it shares an ID with one '
                 'already set: connect it ALONE and run again.')
    if len(new) > 1:
        sys.exit(f'STOP: more than one unset servo on the bus {new}. Add only one at a time.')

    baud, sid = new[0]
    if (baud, sid) == (default, target):
        print(f'Already correct: ID {target} @ {default} baud')
    else:
        print(f'Changing ID {sid} @ {baud}  ->  ID {target} @ {default}')
        bus.setup_motor(motor, initial_baudrate=baud, initial_id=sid)

    # 3. Verify
    ok, pos = clean_reads(motor)
    volt = bus.read('Present_Voltage', motor, normalize=False) / 10
    print(f'Verify {motor} (ID {target}): {ok}/10 reads OK, position {pos}, voltage {volt} V')
    for name, m in bus.motors.items():
        if m.id in done_ids:
            o, _ = clean_reads(name, 3)
            print(f'  still OK: {name} (ID {m.id}) {o}/3')
    print('DONE' if ok == 10 else 'WARNING: unreliable reads, check the cable')
finally:
    bus._close()
