from pymavlink import mavutil
import time
import pymavlink.dialects.v20.all as dialect

# Connect to vehicles
master_in = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master_out = mavutil.mavlink_connection('udpout:192.168.56.101:14551')
slave = mavutil.mavlink_connection('udp:127.0.0.1:14553')

# Wait for heartbeat from slave # not connected if vehicle target system = 0
slave.wait_heartbeat(timeout=10)
print("Slave System ID: ", slave.target_system, " + Slave Component ID: ", slave.target_component) 

waypoints = {}
total_points = None

print("\nWaiting for MASTER polygon grid…")

while True:
    msg = master_in.recv_match(type="NAMED_VALUE_FLOAT", blocking=True)
    if not msg:
        continue

    m = msg.to_dict()
    raw_name = m["name"]
    if isinstance(raw_name, bytes):
        name = raw_name.decode("ascii").strip()
    else:
        name = raw_name.strip()

    if name.startswith("s_lat_"):
        idx = int(name[6:])
        if idx not in waypoints:
            waypoints[idx] = [None, None]
        waypoints[idx][0] = m["value"]

    elif name.startswith("s_lon_"):
        idx = int(name[6:])
        if idx not in waypoints:
            waypoints[idx] = [None, None]
        waypoints[idx][1] = m["value"]

    elif name == "s_total":
        total_points = int(m["value"])
        print(f"Expecting {total_points} points.")

    # stop when all points arrived
    if total_points is not None:
        if len(waypoints) == total_points:
            if all(v[0] is not None and v[1] is not None for v in waypoints.values()):
                break

print("All scan points received.")

# convert dict → ordered list
scan_points = [waypoints[i] for i in sorted(waypoints.keys())]

def send_global_target(lat, lon, alt=15):
    slave.mav.set_position_target_global_int_send(
        0,
        slave.target_system,
        slave.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        int(alt),
        0,0,0,0,0,0,0,0
    )

# Fly the scan path point by point

print("\nBeginning scan…")

for (lat, lon) in scan_points:
    print(f"→ Waypoint: {lat:.6f}, {lon:.6f}")

    t_end = time.time() + 3
    while time.time() < t_end:
        send_global_target(lat, lon, 15)
        time.sleep(0.1)


print("Scan complete.")
