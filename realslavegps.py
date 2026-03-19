from pymavlink import mavutil
import time
import pymavlink.dialects.v20.all as dialect
import csv
import os
import psutil

# Connect to vehicles
master_in = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master_out = mavutil.mavlink_connection('udpout:192.168.2.14:14551')
slave = mavutil.mavlink_connection('udp:127.0.0.1:14553')

# Wait for heartbeat from slave # not connected if vehicle target system = 0
slave.wait_heartbeat(timeout=10)
print("Slave System ID: ", slave.target_system, " + Slave Component ID: ", slave.target_component) 

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
log_filename = "slave_log.csv"

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
    # Get lon/lat/alt from slave autopilot
    msg = slave.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7 #convert
        rel_alt = msg.relative_alt / 1000 #convert to metres. Relative to home alt
        
        cpu = psutil.cpu_percent()
        now = time.time_ns() / 1e9

        print(f"[SLAVE] Sending lat/lon/rel_alt: {lat:.6f}, {lon:.6f}, {rel_alt:.1f}")
        send_value(master_out, lat, "lat_s")
        send_value(master_out, lon, "lon_s")
        send_value(master_out, rel_alt, "rel_alt_s")

        with open(log_filename, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([now, "TX", "lat_m", lat, cpu])
            writer.writerow([now, "TX", "lon_m", lon, cpu])
            writer.writerow([now, "TX", "rel_alt_m", rel_alt, cpu])

    # Listen for pitch from master
    while True:
        msg = master_in.recv_match(type='NAMED_VALUE_FLOAT', blocking=False)
        if msg is None:
            break  # no more messages available
        msg = msg.to_dict()
        if msg['name'] == 'lat_m':
            lat_val = msg['value']
        elif msg['name'] == 'lon_m':
            lon_val = msg['value']
        elif msg['name'] == 'rel_alt_m':
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
    print(f"[MASTER] Receiving lat/lon/rel_alt: {lat_val:.6f}, {lon_val:.6f}, {rel_alt_val:.1f}")

    time.sleep(0.1)