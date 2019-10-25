# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 16:02:00 2019

@author: sachchit
"""

# import necessary modules
import time
import math
from gacommonutil import ScheduleTask, handle_common_message, schedule_common_tasks, send_remaining_msg, dataStorageCommon, mavutil

# Empty data stroage for reference
dataStorageAgri ={'vx': None,
                  'vy': None,
                  'vz': None}

def set_data_stream(mavConnection, msgList, lock):
    # data rate of more than 100 Hz is not possible as recieving loop is set to run at interval of 0.01 sec
    # don't change that as it affects other vehicles as well
    
    # request data to be sent at the given rate
    # arguments are (target_system, target_component, stream_type, frequency in Hz, stop(0) or start(1))
    
    with lock:
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1))
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 1, 1))
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1))
        msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 1, 1))
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 1, 1))
        msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1))
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1))
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 1, 1))
        msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 3, 1))


def handle_sensor(threadKill, lock):
    while True:
        # stop the thread if triggered by main thread
        if threadKill[0]:
            break

        # Prevent unnecessary resource usage by this program
        time.sleep(1)

def handle_messeges(recieved_msg, lock):
    global dataStorageAgri
    with lock:
        if recieved_msg.get_type() == "GLOBAL_POSITION_INT":
            dataStorageAgri['vx'] = 0.01*recieved_msg.vx
            dataStorageAgri['vy'] = 0.01*recieved_msg.vy
            dataStorageAgri['vz'] = 0.01*recieved_msg.vz
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
    
    global dataStorageAgri

    #################################################################################
    
    # set data stream rate for this vehicle
    set_data_stream(mavConnection, msgList, lock)

    # USE THREADING LOCK PROPERLY TO PREVENT RACE CONDITION AND DEADLOCK

    ############################# Scheduled tasks #####################################

    # make sure all scheduled tasks are finite/non-blocking tasks for clean exit
    #
    # ALSO MAKE SURE SCHEDULED TASKS TAKE MUCH LESS TIME THAN THE TIME INTERVAL 
    
    # schedule common tasks
    schedule_common_tasks(schTaskList, msgList, lock)

    # Schedule message sending tasks
    # schTaskList.append(ScheduleTask(timeInterval, functionName, functionArguments))
    schTaskList.append(ScheduleTask(1./freq, send_remaining_msg, msgList, mavConnection, lock))

    ####################################################################################

    # Final infinite loop on tasks which requires data generated/recieved
    # from Scheduled task and takes long time to run
    # This loop should wait for all tasks to be finished before going to next iteration
    
    while True:
        try:
            # Prevent unnecessary resource usage by this program
            time.sleep(0.2)

            ############################# Sequential Tasks ###############################
            

            ##############################################################################

        except KeyboardInterrupt:
            # stop scheduled tasks
            for task in schTaskList:
                task.stop()

            # again raise keyboardinterrupt so that outer try except can also handle the clean thread exits
            raise KeyboardInterrupt
        

        

