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

def send_value(system, value, label):
    system.mav.named_value_float_send(
        int(time.time()),    # time_boot_ms (we use epoch seconds here)
        label.encode('ascii'),
        value
    )

# Define Variables
lat_val = 0.0
lon_val = 0.0
rel_alt_val = 0.0

while True:
    # Get lon/lat/alt from master autopilot
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7 #convert
        rel_alt = msg.relative_alt / 1000 #convert to metres. Relative to home alt
        print(f"[MASTER] Sending lat/lon/rel_alt: {lat:.6f}, {lon:.6f}, {rel_alt:.1f}")
        send_value(slave_out, lat, "lat_m")
        send_value(slave_out, lon, "lon_m")
        send_value(slave_out, rel_alt, "rel_alt_m")

    # Listen for pitch from slave
    while True:
        msg = slave_in.recv_match(type='NAMED_VALUE_FLOAT', blocking=False)
        if msg is None:
            break  # no more messages available
        msg = msg.to_dict()
        if msg['name'] == 'lat_s':
            lat_val = msg['value']
        elif msg['name'] == 'lon_s':
            lon_val = msg['value']
        elif msg['name'] == 'rel_alt_s':
            rel_alt_val = msg['value']

    # Print the latest values on one line
    print(f"[SLAVE] Receiving lat/lon/rel_alt: {lat_val:.6f}, {lon_val:.6f}, {rel_alt_val:.1f}")

    time.sleep(0.1)
