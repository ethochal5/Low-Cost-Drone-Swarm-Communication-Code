### === Vertical Orbit Real === ###
### === Safety and Latency Data added === ###

import time
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect
import math

# Connect to vehicles
master = mavutil.mavlink_connection('udp:127.0.0.1:14552')
slave_out = mavutil.mavlink_connection('udpout:192.168.2.12:14550')
slave_in = mavutil.mavlink_connection('udp:0.0.0.0:14551')

# Wait for heartbeat from master # no heartbeat if vehicle target system = 0
master.wait_heartbeat(10)
print("Master System ID: ", master.target_system, " + Master Component ID: ", master.target_component)

# Wait for pre-arm checks
while True:

    # observe the SYS_STATUS messages
    message = master.recv_match(type=dialect.MAVLink_sys_status_message.msgname, blocking=True)

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
        print("Master is armable")

        # break the prearm check loop
        break

# Set Guided
# desired flight mode
FLIGHT_MODE = "GUIDED"

# get supported flight modes
flight_modes = master.mode_mapping()

# check the desired flight mode is supported
if FLIGHT_MODE not in flight_modes.keys():

    # inform user that desired flight mode is not supported by the vehicle
    print(FLIGHT_MODE, "is not supported")

    # exit the code
    exit(1)

# create change mode message
set_mode_message = dialect.MAVLink_command_long_message(
    target_system=master.target_system,
    target_component=master.target_component,
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
master.mav.send(set_mode_message)

while True:

    # catch COMMAND_ACK message
    message = master.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

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
    target_system=master.target_system,
    target_component=master.target_component,
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
    print("Master is arming...")

    # send arm message
    master.mav.send(vehicle_arm_message)

    # wait COMMAND_ACK message
    message = master.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

    # convert the message to dictionary
    message = message.to_dict()

    # check if the vehicle is armed
    if message["result"] == dialect.MAV_RESULT_ACCEPTED and message["command"] == dialect.MAV_CMD_COMPONENT_ARM_DISARM:

        # print that vehicle is armed
        print("Master is armed!")
        break

    else:

        # print that vehicle is not armed
        print("Master is not armed!")

    # wait some time
    time.sleep(2)
time.sleep(5)
# Takeoff
# takeoff altitude definition
TAKEOFF_ALTITUDE = 10

# create takeoff command
takeoff_command = dialect.MAVLink_command_long_message(
    target_system=master.target_system,
    target_component=master.target_component,
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
master.mav.send(takeoff_command)

# inform user
print("Sent takeoff command to vehicle")

time.sleep(30)

# Set Guided
# desired flight mode
FLIGHT_MODE = "LAND"

# get supported flight modes
flight_modes = master.mode_mapping()

# check the desired flight mode is supported
if FLIGHT_MODE not in flight_modes.keys():

    # inform user that desired flight mode is not supported by the vehicle
    print(FLIGHT_MODE, "is not supported")

    # exit the code
    exit(1)

# create change mode message
set_mode_message = dialect.MAVLink_command_long_message(
    target_system=master.target_system,
    target_component=master.target_component,
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
master.mav.send(set_mode_message)
