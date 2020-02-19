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
from pymavlink import mavutil
from threading import Timer
import serial

# mavlink connection create
def create_mavlink_connection(sitl):
    print 0
    while True:    
        try:
            time.sleep(1)
            mavConnection = None
            # Create the connection
            if sitl == 'tcp':
                mavConnection = mavutil.mavlink_connection('tcp:127.0.0.1:5760',
                                                           source_system=1,
                                                           source_component=10)
            elif sitl == 'udp':
                mavConnection = mavutil.mavlink_connection('udp:127.0.0.1:14551',
                                                           source_system=1,
                                                           source_component=10)
            elif sitl == 'com':
                mavConnection = mavutil.mavlink_connection('COM28',
                                                           baud=57600,
                                                           source_system=1,
                                                           source_component=10)
            else:
                # Need to provide the serial port and baudrate
                mavConnection = mavutil.mavlink_connection('/dev/ttyS0',#'/dev/ttyPixhawk',
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
        except serial.SerialException:
            if mavConnection is not None:
                mavConnection.close()
            mavConnection = create_mavlink_connection(dataStorageCommon['sitlType'])
                
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
dataStorageCommon={'rc6': None,
                   'isSITL': False,
                   'sitlType': None,
                   'vehArmed':False}

# overall pause switch for stopping companion computer to send any commands to autopilot
shouldPause = False

def send_remaining_msg(msgList, mavConnection, lock):
    global shouldPause
    # should be called from individual vehicle utility
    if not shouldPause:
        try:
            with lock:
                # send all remaining messages one by one
                for msg in msgList:
                    if msg is not None:
                        mavConnection.mav.send(msg)
                del msgList[:]
        except serial.SerialException:
            if mavConnection is not None:
                mavConnection.close()
            mavConnection = create_mavlink_connection(dataStorageCommon['sitlType'])

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
        
        if recieved_msg.get_type() == "HEARTBEAT":
            if recieved_msg.autopilot == 3:
                if (recieved_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0:
                    dataStorageCommon['vehArmed'] = True
                else:
                    dataStorageCommon['vehArmed'] = False
                return
        
# handle common sensors
def handle_common_sensors(schTaskList, lock):
    pass

