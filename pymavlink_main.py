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
from util.gacommonutil import recieving_loop, create_mavlink_connection, dataStorageCommon
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

# define whether it is sitl or not
sitl = None

if args.sitludp:
    sitl = 'udp'
    dataStorageCommon['sitlType'] = 'udp'
    dataStorageCommon['isSITL'] = True
if args.sitltcp:
    sitl = 'tcp'
    dataStorageCommon['sitlType'] = 'tcp'
    dataStorageCommon['isSITL'] = True
if args.sitlcom:
    sitl = 'com'
    dataStorageCommon['sitlType'] = 'com'
    dataStorageCommon['isSITL'] = True
    
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

# master threading lock
# try to utilise only this lock everywhere
lock = threading.Lock()

# Threads list
threads = []

# Thread Kill for infinite threads
threadKill = [[False]]      # recieving_loop

logging.basicConfig(
    level=logging.INFO,
 #   level=logging.WARN,
    format='%(asctime)s.%(msecs)03d %(name)-12s %(levelname)-8s %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
)
# list of remaining messages to be sent
msgList = []

try:
    # start mavlink connection and get the mavConnection object
    mavConnection = create_mavlink_connection(sitl)

    # start read loop
    threads.append(threading.Thread(target=recieving_loop, args=(threadKill[0], msgList, mavConnection, vehutil, lock,)))
    threads[0].start()
	
    # Run the main calculation and updates on main thread
    # i.e. this thread
    vehutil.update(msgList, mavConnection, lock)

except KeyboardInterrupt:
    # send kill signal to all threads 
    for i in range(len(threadKill)):
        threadKill[i][0] = True

    # wait for them to finish
    for i in range(len(threads)):
        threads[i].join()
