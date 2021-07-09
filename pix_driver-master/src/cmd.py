from dbc import decode_dbc
import time
import can
import os
os.system("ip link set up can0")


ms = decode_dbc('self_driving.dbc')
m = ms.get_message_by_name('Auto_control')

t1 = time.clock()
data = m.encode(
    {
        'Steering': 30,
        'Speed': 80,
        'Gear_shift': 0,
        'mode_selection': 1,
        'Speed_mode': 0,
        'self_driving_enable': 1,
        'right_light': 1,
        'left_light': 1,
        'Front_light': 1
    }
)
message = can.Message(arbitration_id=m.frame_id, data=data, is_extended_id=0)
t2 = time.clock()
print t2 - t1

with can.interface.Bus() as bus:
    while True:
        bus.send(message)
        time.sleep(0.02)


