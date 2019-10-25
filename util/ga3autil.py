# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 16:02:00 2019

@author: sachchit
"""

# import necessary modules
import time
from gacommonutil import ScheduleTask, handle_common_message, schedule_common_tasks, send_remaining_msg, dataStorageCommon, mavutil, handle_common_sensors
from ga3apayloadutil import PIBStatus, AgriPayload
import numpy as np
from os import path
import sys

# Empty data stroage for reference
dataStorageAgri ={'vx': 0,
                  'vy': 0,
                  'vz': 0,
                  'currentMode': 'STABILIZE',
                  'currentWP': 0,
                  'startWP': 1,
                  'endWP': 2,
                  'NozzleConfig': 0b00111100,
                  'actualNozzRPM': 0,
                  'actualFlowRate': 0,
                  'testing': False,
                  'remainingPayload': 15,
                  'currentLat': 0,
                  'currentLon': 0,
                  'RTLLat': -200,
                  'RTLLon': -200,
                  'RTLWP': 0}

def set_data_stream(mavConnection, msgList, lock):
    print "requested"
    # data rate of more than 100 Hz is not possible as recieving loop is set to run at interval of 0.01 sec
    # don't change that as it affects other vehicles as well
    
    # request data to be sent at the given rate
    # arguments are (target_system, target_component, stream_type, frequency in Hz, stop(0) or start(1))
    
    with lock:
        # stop data which are coming automatically to stop recieving unnecessary messeges
        # arguments are (target_system, target_component, stream_type, frequency in Hz, stop(0) or start(1))
    
        msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 0))
        
        
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1))
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 1, 1))
        msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1))
        msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 2, 1))
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 1, 1))
        msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1))
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1))
        #msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 1, 1))
        msgList.append(mavutil.mavlink.MAVLink_request_data_stream_message(mavConnection.target_system, mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 1, 1))

def handle_sensor(schTaskList, dataStorageAgri, lock):
    # Handle sensors that are common to all vehicles like ADSB
    handle_common_sensors(schTaskList, lock)
    
    # Initialize Sensor handling class
    pibStatus = PIBStatus('/dev/ttyDCU')
    #flowSensor = FlowSensor()
    
    # Schedule the tasks
    schTaskList.append(ScheduleTask(0.05, pibStatus.update, dataStorageAgri, lock))
    #schTaskList.append(ScheduleTask(0.2, flowSensor.calc_flow_rate, dataStorageAgri, lock))

def handle_messeges(recieved_msg, lock):
    global dataStorageAgri
    with lock:
        if recieved_msg.get_type() == "GLOBAL_POSITION_INT":
            dataStorageAgri['vx'] = 0.01*recieved_msg.vx
            dataStorageAgri['vy'] = 0.01*recieved_msg.vy
            dataStorageAgri['vz'] = 0.01*recieved_msg.vz
            dataStorageAgri['currentLat'] = recieved_msg.lat
            dataStorageAgri['currentLon'] = recieved_msg.lon
            return
        
        if recieved_msg.get_type() == "MISSION_CURRENT":
            dataStorageAgri['currentWP'] = recieved_msg.seq
            return
            
        if recieved_msg.get_type() == "RC_CHANNELS":
            if recieved_msg.chan7_raw > 1800:
                dataStorageAgri['testing'] = True
            else:
                dataStorageAgri['testing'] = False
            return
        
        if recieved_msg.get_type() == "GA3A_PAYLOAD_COMMAND":
            print recieved_msg
            if recieved_msg.set_payload > -1:
                dataStorageAgri['remainingPayload'] = recieved_msg.set_payload
            if recieved_msg.start_wp > 0:
                dataStorageAgri['startWP'] = recieved_msg.start_wp
            if recieved_msg.end_wp > recieved_msg.start_wp:
                dataStorageAgri['endWP'] = recieved_msg.end_wp
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
    
    agriPayload = AgriPayload()

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
    
    # handle sensors
    handle_sensor(schTaskList, dataStorageAgri, lock)

    # Schedule message sending tasks
    # schTaskList.append(ScheduleTask(timeInterval, functionName, functionArguments))
    schTaskList.append(ScheduleTask(1./freq, send_remaining_msg, msgList, mavConnection, lock))

    ####################################################################################
    
    ################################ Initial set Up ####################################
    if path.exists('agri_rtl_file'):
        with open('agri_rtl_file', 'r') as f:
            for line in f:
                data = line.split()
                with lock:
                    dataStorageAgri['RTLLat'] = int(data[0])
                    dataStorageAgri['RTLLon'] = int(data[1])
                    dataStorageAgri['RTLWP'] = int(data[2])
                break
    
    ####################################################################################

    # Final infinite loop on tasks which requires data generated/recieved
    # from Scheduled task and takes long time to run
    # This loop should wait for all tasks to be finished before going to next iteration
    
    while True:
        try:
            # Prevent unnecessary resource usage by this program
            time.sleep(0.2)
            
            ############################# Sequential Tasks ###############################
            
            # Save RTL Lat long
            if mavConnection.flightmode is 'RTL' and dataStorageAgri['currentMode'] is not 'RTL':
                with lock:
                    dataStorageAgri['RTLLat'] = dataStorageAgri['currentLat']
                    dataStorageAgri['RTLLon'] = dataStorageAgri['currentLon']
                    dataStorageAgri['RTLWP'] = dataStorageAgri['currentWP']
                with open('agri_rtl_file', 'w') as f:
                    f.write('%d %d %d'%(dataStorageAgri['RTLLat'], dataStorageAgri['RTLLon'], dataStorageAgri['RTLWP']))
            
            dataStorageAgri['currentMode'] = mavConnection.flightmode
            
            
            
            # update the required flow rate to the agri payload handlere
            agriPayload.update(dataStorageAgri, mavConnection, mavutil, msgList, lock)
            
            # send the data to GCS
            data = np.zeros(64, np.uint8)
            msg = mavutil.mavlink.MAVLink_ga3a_payload_status_message(0, 0, dataStorageAgri['remainingPayload'], dataStorageAgri['RTLLat'], dataStorageAgri['RTLLon'], dataStorageAgri['RTLWP'])
            #msg = mavutil.mavlink.MAVLink_data64_message(1,64,data)
            with lock:
                msgList.append(msg)

            sys.stdout.flush()
#            print mavConnection.target_system, mavConnection.target_component
            
            #msg = mavutil.mavlink.MAVLink_data16_message(1, 5, [123]*16)
            
            #msgList.append(msg)

            ##############################################################################

        except KeyboardInterrupt:
            # stop scheduled tasks
            for task in schTaskList:
                task.stop()

            # again raise keyboardinterrupt so that outer try except can also handle the clean thread exits
            raise KeyboardInterrupt
        
