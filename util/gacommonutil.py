# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 14:21:00 2019

@author: sachchit
"""

######################################################################
# This file handle common tasks like mavlink connection and recieving
# mavlink messeges
######################################################################

# import necessary modules
import time
from pymavlink import mavutil
from threading import Timer
import numpy as np

# mavlink connection create
def create_mavlink_connection(sitl):
    # Create the connection
    if sitl == 'tcp':
        mavConnection = mavutil.mavlink_connection('tcp:127.0.0.1:5760',
                                                   source_system=1,
                                                   source_component=10)
    elif sitl == 'udp':
        mavConnection = mavutil.mavlink_connection('udp:127.0.0.1:14551',
                                                   source_system=1,
                                                   source_component=10)
    else:
        # Need to provide the serial port and baudrate
        mavConnection = mavutil.mavlink_connection('/dev/serial0',
                                                   baud=921600,
                                                   source_system=1,
                                                   source_component=10)
        

    # wait for the heartbeat msg to find the system ID
    mavConnection.wait_heartbeat()

    # stop data which are coming automatically to stop recieving unnecessary messeges
    # arguments are (target_system, target_component, stream_type, frequency in Hz, stop(0) or start(1))
    
    mavConnection.mav.request_data_stream_send(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 0)

    return mavConnection

# Pymavlink recieveng loop
def recieving_loop(threadKill, mavConnection, vehutil, lock):
    while True:
        # stop the thread if triggered by main thread
        if threadKill[0]:
            mavConnection.close()
            break
        
        # Prevent unnecessary resource usage by this program
        time.sleep(0.01)
        try:
            # Recieve the messages
            recieved = mavConnection.recv_match()
    
            # If empty message ignore
            if recieved is not None:
                # debug
                # print(recieved)
    
                # Start storing values
                vehutil.handle_messeges(recieved, lock)
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
dataStorageCommon={'rc6': None}

# overall pause switch for stopping companion computer to send any commands to autopilot
shouldPause = False

def send_remaining_msg(msgList, mavConnection, lock):
    global shouldPause
    # should be called from individual vehicle utility
    if not shouldPause:
        with lock:
            # send all remaining messages one by one
            for msg in msgList:
                if msg is not None:
                    mavConnection.mav.send(msg)
            del msgList[:]

def check_pause(lock):
    global shouldPause
    # check if rc 6 is more than 1800
    # if yes change the flag
    if dataStorageCommon['rc6'] is not None:
        if dataStorageCommon['rc6'] > 1800:
            shouldPause = True
        else:
            shouldPause = False  

def send_heartbeat(msgList, lock):
    msg = mavutil.mavlink.MAVLink_heartbeat_message(18, 8, 0, 0, 0, 3)
    with lock:
        msgList.append(msg)

# schedule common tasks
def schedule_common_tasks(schTaskList, msgList, lock):
    # schTaskList.append(ScheduleTask(timeInterval, functionName, functionArguments))
    schTaskList.append(ScheduleTask(0.1, check_pause, lock))
    schTaskList.append(ScheduleTask(1, send_heartbeat, msgList, lock))

# handle common messages
def handle_common_message(recieved_msg, lock):
    global dataStorageCommon
    with lock:
        if recieved_msg.get_type() == "RC_CHANNELS":
            dataStorageCommon['rc6'] = recieved_msg.chan6_raw
            return
        
# handle common sensors
def handle_common_sensors(schTaskList, lock):
    pass

