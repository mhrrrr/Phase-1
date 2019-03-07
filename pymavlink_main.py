# -*- coding: utf-8 -*-
"""
Created on Mon Feb 18 15:33:00 2019

@author: sachchit
"""

######################################################################
# This is main thread which will start appropriate processes at
# appropriate time. Also this will handle termination of those
# threads. 
#
# It is allowed to create more threads in vehutil.update() if needed
######################################################################

# import necessary modules
from util.gacommonutil import recieving_loop, create_mavlink_connection
import threading
import argparse

# handle arguments
parser = argparse.ArgumentParser()
parser.add_argument("--vehicle",help="Vehicle Name", type=str, dest="vehicle")
parser.add_argument("--sitludp", action="store_true", dest="sitludp")
parser.add_argument("--sitltcp", action="store_true", dest="sitltcp")

args = parser.parse_args()

# define vehicle
vehicles = ['GA3', 'GA3T', 'GA3A', 'GA3M']
vehicle = args.vehicle

# import appropriate utility for vehicle
if vehicle == 'GA3':
    import util.ga3util as vehutil
elif vehicle == 'GA3T':
    import util.ga3tutil as vehutil
elif vehicle == 'GA3A':
    import util.ga3autil as vehutil
elif vehicle == 'GA3M':
    import util.ga3mutil as vehutil
else:
    print("Please give valid vehicle using\n")
    print("python pymavlink_main.py --vehicle <VehicleName>\n")
    print("Valid Vehicle names are")
    print(vehicles)
    exit()

# define whether it is sitl or not
sitl = None

if args.sitludp:
    sitl = 'udp'
if args.sitltcp:
    sitl = 'tcp'


# master threading lock
# try to utilise only this lock everywhere
lock = threading.Lock()

# Threads list
threads = []

# Thread Kill for infinite threads
threadKill = [[False],      # recieving_loop
              [False]]      # sensors reading

try:
    # start mavlink connection and get the mavConnection object
    mavConncection = create_mavlink_connection(sitl)

    # start read loop
    threads.append(threading.Thread(target=recieving_loop, args=(threadKill[0], mavConncection, vehutil, lock,)))
    threads[0].start()

    # start reading vehicle related sensors
    threads.append(threading.Thread(target=vehutil.handle_sensor, args=(threadKill[1], lock,)))
    threads[1].start()
	
    # Run the main calculation and updates on main thread
    # i.e. this thread
    vehutil.update(mavConncection, lock)

except KeyboardInterrupt:
    # send kill signal to all threads 
    for i in range(len(threadKill)):
        threadKill[i][0] = True

    # wait for them to finish
    for i in range(len(threads)):
        threads[i].join()
