These codes are used in the Low Cost Drone Swarm Communication project to demonstrate the effectiveness of the communication system:

SITL

ecmasterpitch.py ecslavepitch.py – Basic pitch communication \
ecmastergps.py ecslavegps.py – Basic GPS position communication \
ecmastergpslog.py ecslavegpslog.py – Basic GPS position communication with data logging \
ecmasterhorbit.py ecslavehorbit.py – GPS position communication with automated take off and slave orbiting master moving slowly forwards \
ecmastervorbit.py ecslavevorbit.py – GPS position communication with automated take off and slave orbiting master moving upwards and downwards either clockwise or anticlockwise depending on which direction the master is moving \
ecmasterareasplit.py ecslaveareasplit.py – GPS points logged by master and sent as a polygon to the slave (basis for area automated area splitting) 

Real-World

realmastertakeoffland.py realslavetakeoffland.py – Basic take off and land (no communication)\
realmastergps.py realslavegps.py – Basic GPS position communication with data logging\
realmasterorbit.py realslaveorbit.py – GPS position communication with automated take off and slave orbiting master moving upwards and downwards either clockwise or anticlockwise depending on which direction the master is moving


An experimental setup guide is included as a PDF document: ExperimentalSetupDroneSwarmCommunciation&Management.pdf \
Data logs are provided for reference in the data logs folder
