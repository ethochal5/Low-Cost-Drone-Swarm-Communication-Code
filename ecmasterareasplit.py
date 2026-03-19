import math
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

R = 6378137.0  # Earth radius (m)

def offset_to_gps(lat, lon, dx, dy):
    """Return new lat/lon from dx (east), dy (north)."""
    new_lat = lat + (dy / R) * (180 / math.pi)
    new_lon = lon + (dx / (R * math.cos(math.radians(lat)))) * (180 / math.pi)
    return new_lat, new_lon

def inside_polygon(point, poly):
    """Ray casting 2D point-in-polygon test."""
    x, y = point
    inside = False
    n = len(poly)
    p1x, p1y = poly[0]

    for i in range(n + 1):
        p2x, p2y = poly[i % n]
        if min(p1y, p2y) < y <= max(p1y, p2y):
            if x <= max(p1x, p2x):
                xinters = (y - p1y) * (p2x - p1x) / ((p2y - p1y) + 1e-9) + p1x
                if p1x == p2x or x <= xinters:
                    inside = not inside
        p1x, p1y = p2x, p2y

    return inside

def send_value(system, value, label):
    """Send NAMED_VALUE_FLOAT message."""
    system.mav.named_value_float_send(
        int(time.time()),
        label.encode("ascii"),
        value
    )

########## RECORD AREA ##########
import sys
import select

perimeter = []
print("\n=== MANUALLY FLY MASTER AROUND FIELD USING MAVPROXY ===")
print("Recording perimeter points automatically.")
print("When finished, press ENTER in this terminal.\n")

master.recv_match(blocking=False)

while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        perimeter.append((lat, lon))
        print(f"Recorded point: {lat:.6f}, {lon:.6f}")

    # stop when user hits ENTER
    r,_,_ = select.select([sys.stdin], [], [], 0)
    if r:
        sys.stdin.readline()
        break

if perimeter[0] != perimeter[-1]:
    perimeter.append(perimeter[0])

print("\nPolygon recorded:")
for p in perimeter:
    print(p)

########## SPLIT AREA ##########

lats = [p[0] for p in perimeter]
lons = [p[1] for p in perimeter]
mid_lon = (min(lons) + max(lons)) / 2

slave_poly = [p for p in perimeter if p[1] >= mid_lon]

print("\nSLAVE polygon region:")
for p in slave_poly:
    print(p)

########## 1m GRID ##########

min_lat = min(p[0] for p in slave_poly)
max_lat = max(p[0] for p in slave_poly)
min_lon = min(p[1] for p in slave_poly)
max_lon = max(p[1] for p in slave_poly)

grid = []
step_m = 1.0

# Convert meter steps approx to degree steps
lat_step_deg = step_m / 111111.0

# lon conversion changes with latitude
def lon_step(lat):
    return step_m / (111111.0 * math.cos(math.radians(lat)))

lat_val = min_lat
while lat_val <= max_lat:
    lon_val = min_lon
    step_lon_deg = lon_step(lat_val)
    while lon_val <= max_lon:
        if inside_polygon((lat_val, lon_val), slave_poly):
            grid.append((lat_val, lon_val))
        lon_val += step_lon_deg
    lat_val += lat_step_deg

print(f"\nGenerated {len(grid)} inside-polygon 1m scan points.")

########## SEND GRID ##########

print("Sending points to SLAVE…")

send_value(slave_out, len(grid), "s_total")
time.sleep(1)

for i, (lat, lon) in enumerate(grid):
    send_value(slave_out, lat, f"s_lat_{i}")
    send_value(slave_out, lon, f"s_lon_{i}")
    time.sleep(0.01)

print("Slave poly size", len(slave_poly))
