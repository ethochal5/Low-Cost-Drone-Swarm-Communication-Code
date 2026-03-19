### === Vertical Orbit === ###

from pymavlink import mavutil
import time
import pymavlink.dialects.v20.all as dialect
import math

# Connect to vehicles
master_in = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master_out = mavutil.mavlink_connection('udpout:192.168.2.16:14551')
slave = mavutil.mavlink_connection('udp:127.0.0.1:14553')

# Wait for heartbeat from slave # not connected if vehicle target system = 0
slave.wait_heartbeat(timeout=10)
print("Slave System ID: ", slave.target_system, " + Slave Component ID: ", slave.target_component) 

# Wait for pre-arm checks
while True:

    # observe the SYS_STATUS messages
    message = slave.recv_match(type=dialect.MAVLink_sys_status_message.msgname, blocking=True)

    # convert to dictionary
    message = message.to_dict()

    # get sensor health
    onboard_control_sensors_health = message["onboard_control_sensors_health"]

    # get pre-arm healthy bit
    prearm_status_bit = onboard_control_sensors_health & dialect.MAV_SYS_STATUS_PREARM_CHECK
    prearm_status = prearm_status_bit == dialect.MAV_SYS_STATUS_PREARM_CHECK

    # check prearm
    if prearm_status:

        # vehicle can be armable
        print("Slave is armable")

        # break the prearm check loop
        break

# Set Guided
# desired flight mode
FLIGHT_MODE = "GUIDED"

# get supported flight modes
flight_modes = slave.mode_mapping()

# check the desired flight mode is supported
if FLIGHT_MODE not in flight_modes.keys():

    # inform user that desired flight mode is not supported by the vehicle
    print(FLIGHT_MODE, "is not supported")

    # exit the code
    exit(1)

# create change mode message
set_mode_message = dialect.MAVLink_command_long_message(
    target_system=slave.target_system,
    target_component=slave.target_component,
    command=dialect.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=flight_modes[FLIGHT_MODE],
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

# change flight mode
slave.mav.send(set_mode_message)

while True:

    # catch COMMAND_ACK message
    message = slave.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

    # convert this message to dictionary
    message = message.to_dict()

    # check is the COMMAND_ACK is for DO_SET_MODE
    if message["command"] == dialect.MAV_CMD_DO_SET_MODE:

        # check the command is accepted or not
        if message["result"] == dialect.MAV_RESULT_ACCEPTED:

            # inform the user
            print("Changing mode to", FLIGHT_MODE, "accepted from the vehicle")

        # not accepted
        else:

            # inform the user
            print("Changing mode to", FLIGHT_MODE, "failed")

        # break the loop
        break

time.sleep(2)
# Arm Throttle
# arm disarm definitions
VEHICLE_ARM = 1
VEHICLE_DISARM = 0

# vehicle arm message
vehicle_arm_message = dialect.MAVLink_command_long_message(
    target_system=slave.target_system,
    target_component=slave.target_component,
    command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
    confirmation=0,
    param1=VEHICLE_ARM,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

while True:

    # arm the vehicle
    print("Slave is arming...")

    # send arm message
    slave.mav.send(vehicle_arm_message)

    # wait COMMAND_ACK message
    message = slave.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

    # convert the message to dictionary
    message = message.to_dict()

    # check if the vehicle is armed
    if message["result"] == dialect.MAV_RESULT_ACCEPTED and message["command"] == dialect.MAV_CMD_COMPONENT_ARM_DISARM:

        # print that vehicle is armed
        print("Slave is armed!")
        break

    else:

        # print that vehicle is not armed
        print("Slave is not armed!")

    # wait some time
    time.sleep(2)
time.sleep(5)
# Takeoff
# takeoff altitude definition
TAKEOFF_ALTITUDE = 15

# create takeoff command
takeoff_command = dialect.MAVLink_command_long_message(
    target_system=slave.target_system,
    target_component=slave.target_component,
    command=dialect.MAV_CMD_NAV_TAKEOFF,
    confirmation=0,
    param1=0,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=TAKEOFF_ALTITUDE
)

# takeoff the vehicle
slave.mav.send(takeoff_command)

# inform user
print("Sent takeoff command to vehicle")

time.sleep(20)

def distance_to_latlon(lat, lon, y_m, x_m):
    R = 6378137.0 #Radius Earth
    new_lat = lat + (x_m / R) * (180.0 / math.pi)
    new_lon = lon + (y_m / (R * math.cos(math.radians(lat)))) * (180.0 / math.pi)
    return new_lat, new_lon

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
orbit_radius = 5 #m
orbit_speed = 1 #rad/s
last_time = time.time()
angle = 0.0
prev_master_alt = None
orbit_direction = 1  # +1 = CCW, -1 = CW

while True:
    # Get lon/lat/alt from slave autopilot
    msg = slave.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7 #convert
        rel_alt = msg.relative_alt / 1000 #convert to metres. Relative to home alt
        print(f"[SLAVE] Sending lat/lon/rel_alt: {lat:.6f}, {lon:.6f}, {rel_alt:.1f}")
        send_value(master_out, lat, "lat_s")
        send_value(master_out, lon, "lon_s")
        send_value(master_out, rel_alt, "rel_alt_s")

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

    # Print the latest values on one line
    print(f"[MASTER] Receiving lat/lon/rel_alt: {lat_val:.6f}, {lon_val:.6f}, {rel_alt_val:.1f}")

    # MOVEMENT BASED ON FEEDBACK
    # Time Step
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    if prev_master_alt is not None:
        if rel_alt_val > prev_master_alt:
            orbit_direction = -1   # Master going up - CLOCKWISE
        elif rel_alt_val < prev_master_alt:
            orbit_direction = 1    # Master going down - ANTICLOCKWISE

    prev_master_alt = rel_alt_val
    # Orbit Angle
    
    angle += orbit_direction * orbit_speed * dt
    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)

    # Orbit Target
    target_lat, target_lon = distance_to_latlon(lat_val, lon_val, x, y)

    # Orbit Setpoint to Slave
    slave.mav.set_position_target_global_int_send(
        0,
        slave.target_system,
        slave.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,#Position ONLY
        int(target_lat * 1e7),
        int(target_lon * 1e7),
        rel_alt_val + 5, #Altitude +5m for safety
        0, 0, 0,
        0, 0, 0,
        0,
        0
    )



    time.sleep(0.1)