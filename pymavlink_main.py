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

import os
os.environ['MAVLINK20'] = "1"

# import necessary modules
from util.gacommonutil import recieving_loop, create_mavlink_connection, common_init
from pymavlink import mavutil
import threading
import argparse
import importlib
import logging

# handle arguments
parser = argparse.ArgumentParser()
parser.add_argument("--vehicle",help="Vehicle Name", type=str, dest="vehicle")
parser.add_argument("--sitludp", action="store_true", dest="sitludp")
parser.add_argument("--sitltcp", action="store_true", dest="sitltcp")
parser.add_argument("--sitlcom", action="store_true", dest="sitlcom")

args = parser.parse_args()

# define vehicle
vehicles = ['GA3', 'GA3T', 'GA3A', 'GA3M']
vehicle = args.vehicle
# import appropriate utility for vehicle
if vehicle in vehicles:
    modname = "util." + vehicle.lower() + "util"
    vehutil = importlib.import_module(modname)
else:
    print("Please give valid vehicle using\n")
    print("python pymavlink_main.py --vehicle <VehicleName>\n")
    print("Valid Vehicle names are")
    print(vehicles)
    exit()

# Data Container Initialization
statusData = vehutil.Data()
statusData.mavutil = mavutil

if args.sitludp:
    statusData.sitlType = 'udp'
    statusData.isSITL = True
if args.sitltcp:
    statusData.sitlType = 'tcp'
    statusData.isSITL = True
if args.sitlcom:
    statusData.sitlType = 'com'
    statusData.isSITL = True

# master threading lock
# try to utilise only this lock everywhere
# USE THREADING LOCK PROPERLY TO PREVENT RACE CONDITION AND DEADLOCK
statusData.lock = threading.Lock()

# Threads list
threads = []

# Thread Kill for infinite threads
threadKill = [[False]]      # recieving_loop

logging.basicConfig(
    level=logging.INFO,
#    level=logging.WARN,
    format='%(asctime)s.%(msecs)03d %(name)-12s %(levelname)-8s %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
)

try:
    # start mavlink connection and get the mavConnection object
    statusData.mavConnection = create_mavlink_connection(statusData)

    # start mavlink read loop
    threads.append(threading.Thread(target=recieving_loop, args=(threadKill[0], vehutil, statusData,)))
    threads[0].start()
	
    # Initialize Common Tasks
    common_init(statusData)
    
    # Run the main calculation and updates on main thread
    # i.e. this thread
    vehutil.update(statusData)

except KeyboardInterrupt:
    # send kill signal to all threads 
    for i in range(len(threadKill)):
        threadKill[i][0] = True

    # wait for them to finish
    for i in range(len(threads)):
        threads[i].join()
