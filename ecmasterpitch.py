import time
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect

# Connect to vehicles
master = mavutil.mavlink_connection('udp:127.0.0.1:14552')
slave_out = mavutil.mavlink_connection('udpout:192.168.56.102:14550')
slave_in = mavutil.mavlink_connection('udp:0.0.0.0:14551')

# Wait for heartbeat from master # no heartbeat if vehicle target system = 0
master.wait_heartbeat(10)
print("Master System ID: ", master.target_system, " + Master Component ID: ", master.target_component)

def send_pitch(system, pitch_value, label):
    system.mav.named_value_float_send(
        int(time.time()),    # time_boot_ms (we use epoch seconds here)
        label.encode('ascii'),
        pitch_value
    )

while True:
    # Get attitude from master autopilot
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    # print("Attitude debug = ",msg.to_dict()) # enable to debug message
    # {'mavpackettype': 'ATTITUDE', 'time_boot_ms': 4759100, 'roll': -1.699246883392334, 
    # 'pitch': -1.491559624671936, 'yaw': -2.56386399269104, 'rollspeed': -0.00011479342356324196, 
    # 'pitchspeed': 0.00012205576058477163, 'yawspeed': -0.00010708149056881666}
    if msg:
        pitch = msg.pitch
        print(f"[MASTER] Sending pitch: {pitch:.2f}")
        send_pitch(slave_out, pitch, "pitch_m")

    # Listen for pitch from slave
    try:
        msg = slave_in.recv_match(type='NAMED_VALUE_FLOAT', blocking=True, timeout=0.5).to_dict()
        # print("debug NAMED_VALUE_FLOAT = ", msg) # enable to debug message
        # example {'mavpackettype': 'NAMED_VALUE_FLOAT', 'time_boot_ms': 1748591984, 'name': 'pitch_s', 'value': -1.4927592277526855}
        if msg['name'] == 'pitch_s':
            print(f"[MASTER] Received pitch: {msg['value']:.2f}")
    except Exception as e:
        print("Master error receiving from slave:", e)

    time.sleep(1)
