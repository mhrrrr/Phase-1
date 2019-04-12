# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 16:02:00 2019

@author: sachchit
"""

# import necessary modules
import time
import math
from gacommonutil import ScheduleTask, handle_common_message, schedule_common_tasks, send_remaining_msg, dataStorageCommon, mavutil, handle_common_sensors
import threading
import numpy as np

# Empty data stroage for reference
dataStorageGA3 ={'vx': 0,
                  'vy': 0,
                  'vz': 0,
                  'currentWP': 0,
                  'rfndDist': 0}

def set_data_stream(mavConnection):
    # data rate of more than 100 Hz is not possible as recieving loop is set to run at interval of 0.01 sec
    # don't change that as it affects other vehicles as well
    
    # request data to be sent at the given rate
    # arguments are (target_system, target_component, stream_type, frequency in Hz, stop(0) or start(1))

    #mavConnection.mav.request_data_stream_send(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
    #mavConnection.mav.request_data_stream_send(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 1, 1)
    #mavConnection.mav.request_data_stream_send(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1)
    mavConnection.mav.request_data_stream_send(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 1, 1)
    #mavConnection.mav.request_data_stream_send(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 1, 1)
    mavConnection.mav.request_data_stream_send(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
    #mavConnection.mav.request_data_stream_send(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1)
    #mavConnection.mav.request_data_stream_send(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 1, 1)
    mavConnection.mav.request_data_stream_send(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 3, 1)

def handle_sensor(schTaskList, dataStorageGA3, lock):
    # Handle sensors that are common to all vehicles like ADSB
    handle_common_sensors(schTaskList, lock)

def handle_messeges(recieved_msg, lock):
    global dataStorageGA3
    with lock:
        if recieved_msg.get_type() == "GLOBAL_POSITION_INT":
            dataStorageGA3['vx'] = 0.01*recieved_msg.vx
            dataStorageGA3['vy'] = 0.01*recieved_msg.vy
            dataStorageGA3['vz'] = 0.01*recieved_msg.vz
            return
        
        if recieved_msg.get_type() == "MISSION_CURRENT":
            dataStorageGA3['currentWP'] = recieved_msg.seq
            return
        
        if recieved_msg.get_type() == "DISTANCE_SENSOR":
            dataStorageGA3['rfndDist'] = 0.01*recieved_msg.current_distance
            return

    # handle common mssages
    handle_common_message(recieved_msg, lock)

def update(mavConnection, lock):
    ############################### Defining Variables ##############################
    
    # list of remaining messages to be sent
    msgList = []
   
    freq = float(100)
    
    # List of scheduled tasks
    schTaskList = []
    
    global dataStorageGA3

    #################################################################################
    
    # set data stream rate for this vehicle
    set_data_stream(mavConnection)

    # USE THREADING LOCK PROPERLY TO PREVENT RACE CONDITION AND DEADLOCK

    ############################# Scheduled tasks #####################################

    # make sure all scheduled tasks are finite/non-blocking tasks for clean exit
    #
    # ALSO MAKE SURE SCHEDULED TASKS TAKE MUCH LESS TIME THAN THE TIME INTERVAL 
    
    # schedule common tasks
    schedule_common_tasks(schTaskList, msgList, lock)
    
    # handle sensors
    handle_sensor(schTaskList, dataStorageGA3, lock)

    # Schedule message sending tasks
    # schTaskList.append(ScheduleTask(timeInterval, functionName, functionArguments))
    schTaskList.append(ScheduleTask(1./freq, send_remaining_msg, msgList, mavConnection, lock))

    ####################################################################################

    # Final infinite loop on tasks which requires data generated/recieved
    # from Scheduled task and takes long time to run
    # This loop should wait for all tasks to be finished before going to next iteration
    oldTime = time.time()
    persistentTime = 2
    while True:
        try:
            currTime = time.time()
            dt = currTime - oldTime
            
            # Prevent unnecessary resource usage by this program
            time.sleep(0.2)

            ############################# Sequential Tasks ###############################
            print mavConnection.flightmode
            if mavConnection.flightmode == 'AUTO':
                print "inside1"
                if dataStorageGA3['currentWP'] > 1:
                    print "inside2"
                    if dataStorageGA3['rfndDist'] < 12:
                        print 1
                        persistentTime = persistentTime - dt
                        if persistentTime < 0:
                            print 2
                            with lock:
                                mavConnection.set_mode('BRAKE')
                    else:
                        persistentTime = 2

            ##############################################################################

        except KeyboardInterrupt:
            # stop scheduled tasks
            for task in schTaskList:
                task.stop()

            # again raise keyboardinterrupt so that outer try except can also handle the clean thread exits
            raise KeyboardInterrupt
        

        

