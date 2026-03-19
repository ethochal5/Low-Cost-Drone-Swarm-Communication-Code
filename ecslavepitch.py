from pymavlink import mavutil
import time

# Connect to vehicles
master_in = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master_out = mavutil.mavlink_connection('udpout:192.168.56.101:14551')
slave = mavutil.mavlink_connection('udp:127.0.0.1:14553')

# Wait for heartbeat from slave # not connected if vehicle target system = 0
slave.wait_heartbeat(timeout=10)
print("Slave System ID: ", slave.target_system, " + Slave Component ID: ", slave.target_component) 

def send_pitch(system, pitch_value, label):
    system.mav.named_value_float_send(
        int(time.time()),
        label.encode('ascii'),
        pitch_value
    )

while True:
    # Get attitude from slave autopilot
    msg = slave.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    # print("debug Attitide = ", msg.to_dict()) # enable to debug message
    # example {'mavpackettype': 'ATTITUDE', 'time_boot_ms': 4760598, 'roll': 0.018877992406487465, 
    # 'pitch': 0.01843053288757801, 'yaw': -0.5327193737030029, 'rollspeed': 4.817941226065159e-05, 
    # 'pitchspeed': 8.198991417884827e-05, 'yawspeed': 0.00014846574049443007}
    if msg:
        pitch = msg.pitch
        print(f"[SLAVE] Sending pitch: {pitch:.2f}")
        send_pitch(master_out, pitch, "pitch_s")

    # Listen for pitch from master
    try:
        msg = master_in.recv_match(type='NAMED_VALUE_FLOAT', blocking=True, timeout=0.5).to_dict()
        # print("debug NAMED_VALUE_FLOAT = ", msg.to_dict()) # enable to debug message
        # example {'mavpackettype': 'NAMED_VALUE_FLOAT', 'time_boot_ms': 1748591984, 'name': 'pitch_s', 'value': -1.4927592277526855}
        if msg['name'] == 'pitch_m':
            print(f"[SLAVE] Received pitch: {msg['value']:.2f}")
    except Exception as e:
        print("Slave error receiving from master:", e)

    time.sleep(1)
