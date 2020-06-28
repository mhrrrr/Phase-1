# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 14:21:00 2019

@author: sachchit
"""

######################################################################
# This file handle common tasks like mavlink connection and recieving
# or sending mavlink messeges

# Make sure MavConnection Read is only in recieving_loop() and nowhere
# else including vehicle specific libraries
# Make sure MavConnection Write is only in send_remaining_msg() and 
# nowhere else including vehicle specific libraries
######################################################################

# import necessary modules
import time
from threading import Timer
import serial
import logging
from util.npnt import NPNT

# mavlink connection create
def create_mavlink_connection(statusData):
    logging.info("Starting Mavlink Connection")
    while True:    
        try:
            time.sleep(1)
            mavConnection = None
            # Create the connection
            if statusData.sitlType == 'tcp':
                mavConnection = statusData.mavutil.mavlink_connection('tcp:127.0.0.1:5760',
                                                           source_system=1,
                                                           source_component=10)
            elif statusData.sitlType == 'udp':
                mavConnection = statusData.mavutil.mavlink_connection('udp:127.0.0.1:14551',
                                                           source_system=1,
                                                           source_component=10)
            elif statusData.sitlType == 'com':
                mavConnection = statusData.mavutil.mavlink_connection('COM28',
                                                           baud=57600,
                                                           source_system=1,
                                                           source_component=10)
            else:
                # Need to provide the serial port and baudrate
                mavConnection = statusData.mavutil.mavlink_connection('/dev/serial0',
                                                           baud=921600,
                                                           source_system=1,
                                                           source_component=10)
            break
        except KeyboardInterrupt:
            break
        except serial.SerialException:
            if mavConnection is not None:
                mavConnection.close()
            
    # wait for the heartbeat msg to find the system ID
    mavConnection.wait_heartbeat()
    logging.info("Mavlink Connection Establishe")
    return mavConnection

# Pymavlink recieveng loop
def recieving_loop(threadKill, vehutil, statusData):
    while True:
        # stop the thread if triggered by main thread
        if threadKill[0]:
            statusData.mavConnection.close()
            break
        
        # Prevent unnecessary resource usage by this program
        time.sleep(0.01)
        try:
            # Recieve the messages
            recieved = statusData.mavConnection.recv_match()

            # If empty message ignore
            if recieved is not None:
                # debug
                # logging.debug(recieved)
                
                # Start storing values
                vehutil.handle_messeges(recieved, statusData)
                # handle common mssages
                handle_common_message(recieved, statusData)
                
        except serial.SerialException:
            if statusData.mavConnection is not None:
                statusData.mavConnection.close()
            statusData.mavConnection = create_mavlink_connection(statusData.sitlType)
                
        except KeyboardInterrupt:
            # again raise keyboardinterrupt so that outer try except can also handle the clean thread exits
            raise KeyboardInterrupt


# Class for scheduling tasks
class ScheduleTask(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

# Common data for all vehicles
class CommonData:
    def __init__(self):
        # list of remaining messages to be sent
        self.msgList = []
        # MavConnection storage
        self.mavConnection = None
        # Thread Lock
        self.lock = None
        # overall pause switch for stopping companion computer to send any commands to autopilot
        self.shouldPause = False
        # mavutil Library
        self.mavutil = None
        
        self.rc6 = None
        self.isSITL = False
        self.sitlType = None
        self.isArmed = False
        self.isFlying = False
        self.lat = -200e7
        self.lon = -200e7
        self.hdop = 100
        
        self.globalTime = 0
        
        # NPNT
        self.paUploaded = True
        self.paVerified = False
        
        # Threading
        # Sceduled Task List - Make Sure tasks in task list are finite calculations
        # Will be used for repeating certain tasks at certain repeated intervals
        self.schTaskList = []

def send_remaining_msg(statusData):
    # should be called from individual vehicle utility
    if not statusData.shouldPause:
        try:
            with statusData.lock:
                # send all remaining messages one by one
                for msg in statusData.msgList:
                    if msg is not None:
                        statusData.mavConnection.mav.send(msg)
                del statusData.msgList[:]
        except serial.SerialException:
            if statusData.mavConnection is not None:
                statusData.mavConnection.close()
            statusData.mavConnection = create_mavlink_connection(statusData.sitlType)
    else:
        del statusData.msgList[:]

def check_pause(statusData):
    # check if rc 6 is more than 1800
    # if yes change the flag
    if statusData.rc6 is not None:
        if statusData.rc6 > 1800:
            statusData.shouldPause = True
        else:
            statusData.shouldPause = False  

def send_heartbeat(statusData):
    msg = statusData.mavutil.mavlink.MAVLink_heartbeat_message(18, 8, 0, 0, 0, 3)
    with statusData.lock:
        statusData.msgList.append(msg)

# schedule common tasks
def schedule_common_tasks(statusData):
    # schTaskList.append(ScheduleTask(timeInterval, functionName, functionArguments))
    statusData.schTaskList.append(ScheduleTask(0.1, check_pause, statusData))
    statusData.schTaskList.append(ScheduleTask(1, send_heartbeat, statusData))
    
    # NPNT
    npnt = NPNT()
    statusData.schTaskList.append(ScheduleTask(1, npnt.run, statusData))

# handle common messages
def handle_common_message(recievedMsg, statusData):
    with statusData.lock:
        if recievedMsg.get_type() == "RC_CHANNELS":
            statusData.rc6 = recievedMsg.chan6_raw
            return
        
        if recievedMsg.get_type() == "HEARTBEAT":
            if recievedMsg.autopilot == 3:
                sysStatus = recievedMsg.system_status
                isCurrentFlying = False
                isCurrentFlying = sysStatus == 4
                if(statusData.isFlying and not isCurrentFlying):
                    isCurrentFlying = sysStatus == 5 or sysStatus == 6
                statusData.isFlying = isCurrentFlying
                
                if (recievedMsg.base_mode & statusData.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0:
                    statusData.isArmed = True
                else:
                    statusData.isArmed = False
                return
            
        if recievedMsg.get_type() == "GPS_RAW_INT":
            statusData.lat = recievedMsg.lat
            statusData.lon = recievedMsg.lon
            statusData.hdop = recievedMsg.eph/100.
            return
        
        if recievedMsg.get_type() == "SYSTEM_TIME":
            statusData.globalTime = int(recievedMsg.time_unix_usec*1e-6)
            return
        
# handle common sensors
def handle_common_sensors(statusData):
    pass

def common_init(statusData):
    # schedule common tasks
    schedule_common_tasks(statusData)
    
    # Handle sensors that are common to all vehicles like ADSB
    handle_common_sensors(statusData)

