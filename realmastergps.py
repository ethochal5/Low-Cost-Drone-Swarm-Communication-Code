import time
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect
import csv
import os
import psutil

# Connect to vehicles
master = mavutil.mavlink_connection('udp:127.0.0.1:14552')
slave_out = mavutil.mavlink_connection('udpout:192.168.2.16:14550')
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

# Logging setup
log_filename = "master_log.csv"

if not os.path.exists(log_filename):
    with open(log_filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "local_time_sec",
            "direction",    
            "msg_name",
            "value",
            "cpu_percent"
        ])

sequence = 0

while True:
    # Get lon/lat/alt from master autopilot
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7 #convert
        rel_alt = msg.relative_alt / 1000 #convert to metres. Relative to home alt

        cpu = psutil.cpu_percent()
        now = time.time_ns() / 1e9

        print(f"[MASTER] Sending lat/lon/rel_alt: {lat:.6f}, {lon:.6f}, {rel_alt:.1f}")
        send_value(slave_out, lat, "lat_m")
        send_value(slave_out, lon, "lon_m")
        send_value(slave_out, rel_alt, "rel_alt_m")

        with open(log_filename, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([now, "TX", "lat_m", lat, cpu])
            writer.writerow([now, "TX", "lon_m", lon, cpu])
            writer.writerow([now, "TX", "rel_alt_m", rel_alt, cpu])

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

        cpu = psutil.cpu_percent()
        now = time.time_ns() / 1e9
        name = msg['name'] if isinstance(msg['name'], str) else msg['name'].decode()
        value = msg['value']

        # Log RX
        with open(log_filename, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([now, "RX", name, value, cpu])

    # Print the latest values on one line
    print(f"[SLAVE] Receiving lat/lon/rel_alt: {lat_val:.6f}, {lon_val:.6f}, {rel_alt_val:.1f}")

    time.sleep(0.1)