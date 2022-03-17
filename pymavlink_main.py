# -*- coding: utf-8 -*-
"""
Created on Mon Feb 18 15:33:00 2019

@author: Sachchit Vekaria
@Organization: General Aeronautics Pvt Ltd
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
import argparse
import logging

# handle arguments
parser = argparse.ArgumentParser()
parser.add_argument("--vehicle",help="Vehicle Name", type=str, dest="vehicle")
parser.add_argument("--sitludp", action="store_true", dest="sitludp")
parser.add_argument("--sitltcp", action="store_true", dest="sitltcp")
parser.add_argument("--sitlcom", action="store_true", dest="sitlcom")
parser.add_argument("--log", help="Logging Level", type=str, dest="logLevel", default="WARN")
parser.add_argument("--fileLog", action="store_true", dest="fileLogEnabled")
parser.add_argument("--tcpport", type=int, required=False, default=5760,
    help="defines the tcp port at which mavlink is connected")
parser.add_argument("--udpport", type=int, required=False, default=14551,
                    help="defines the tcp port at which mavlink is connected")
args = parser.parse_args()

# Define logging
logNumericLevel = getattr(logging, args.logLevel.upper(), None)
if not isinstance(logNumericLevel, int):
    raise ValueError('Invalid log level: %s' % args.logLevel)

if args.fileLogEnabled:
    logging.basicConfig(
        level=logNumericLevel,
        filename="temp",
        format='%(asctime)s.%(msecs)03d,%(name)-12s,%(levelname)-8s,%(message)s',
        datefmt='%Y-%m-%d %H:%M:%S')
else:
    logging.basicConfig(
        level=logNumericLevel,
        format='%(asctime)s.%(msecs)03d,%(name)-12s,%(levelname)-8s,%(message)s',
        datefmt='%Y-%m-%d %H:%M:%S')

# Handle SITL    
sitlType = None
sitlPort = None
if args.sitludp:
    sitlType = 'udp'
    if args.udpport:
        sitlPort = args.udpport
if args.sitltcp:
    sitlType = 'tcp'
    if args.tcpport:
        sitlPort = args.tcpport
if args.sitlcom:
    sitlType = 'com'


# define vehicle
vehicles = ['GA3', 'GA3A', 'Test']
vehicle = args.vehicle
# import appropriate utility for vehicle
if vehicle in vehicles:
    if vehicle == 'GA3':
        from util.ga3util import GA3CompanionComputer as CompanionComp
    elif vehicle == 'GA3A':
        from util.ga3autil import GA3ACompanionComputer as CompanionComp
    elif vehicle == 'Test':
        from util.testutil import TestCompanionComputer as CompanionComp
else:
    logging.error("Please give valid vehicle using")
    logging.error("python pymavlink_main.py --vehicle <VehicleName>")
    logging.error("Valid Vehicle names are")
    logging.error(vehicles)
    exit()

try:
    # Initialize Companion Computer
    companionComp = CompanionComp(sitlType, sitlPort)
    
    # Start Companion Computer
    companionComp.init()

# During debugging so that we can exit the loop
except Exception as e:
    logging.exception(e)
    companionComp.kill_all_threads()