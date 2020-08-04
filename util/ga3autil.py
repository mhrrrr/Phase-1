# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 16:02:00 2019

@author: sachchit
"""

# import necessary modules
# import necessary modules
import time
from util.gacommonutil import CompanionComputer, mavutil, path, dist_between_lat_lon
import logging
from util.ga3apayloadutil import PIBStatus, AgriPayload, FlowSensor
import numpy as np
import sys
import threading

class GA3ACompanionComputer(CompanionComputer):
    def __init__(self, sitlType):
        # Initialize super class
        super().__init__(sitlType)
        
        # Threading Lock for TestCompanionComputer Class
        self.lock = threading.Lock()
        self.handleRecievedMsgThread = None
        
        # Agri Specific vehicle status 
        self.testing = False
        
    def init(self):
        super().init()
        
        # set data stream rate
        self.set_data_stream()
        
        # start our recieving message handling loop
        self.handleRecievedMsgThread = threading.Thread(target=self.handle_recieved_message)
        self.handleRecievedMsgThread.start()
        
        while True:
            time.sleep(1)
            
    def set_data_stream(self):
        # data rate of more than 100 Hz is not possible as recieving loop is set to run at interval of 0.01 sec
        # don't change that as it affects other vehicles as well
    
        # request data to be sent at the given rate
        # arguments are (target_system, target_component, stream_type, frequency in Hz, stop(0) or start(1))
    
        # stop data which are coming automatically to stop recieving unnecessary messeges
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 0))


        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1))
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 1, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 2, 1))
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 1, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1))
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 1, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2, 1))
    
        logging.info("Stream Rate have been set")
    
    def handle_recieved_message(self):
        while True:
            if self.killAllThread.is_set():
                break
            recievedMsg = self.get_new_message_from_recieving_queue()
            
            if recievedMsg is not None:
                if recieved_msg.get_type() == "RC_CHANNELS":
                    if recieved_msg.chan7_raw > 1800:
                        self.testing = True
                    else:
                        self.testing = False
                    return
        
                if recieved_msg.get_type() == "PARAM_SET":
                    if recieved_msg.param_id == "PAYLOAD":
                        if recieved_msg.param_value > 0 and recieved_msg.param_value < 17:
                            dataStorageAgri['remainingPayload'] = recieved_msg.param_value
                    if recieved_msg.param_id == "CLEARANCE_ALT":
                        if recieved_msg.param_value > 200 and recieved_msg.param_value < 4000:
                            dataStorageAgri['clearanceAlt'] = recieved_msg.param_value
                    if recieved_msg.param_id == "PESTI_PER_ACRE":
                        if recieved_msg.param_value > 1 and recieved_msg.param_value < 20:
                            dataStorageAgri['pesticidePerAcre'] = recieved_msg.param_value
                    if recieved_msg.param_id == "SWATH":
                        if recieved_msg.param_value > 1 and recieved_msg.param_value < 8:
                            dataStorageAgri['swath'] = recieved_msg.param_value
                    if recieved_msg.param_id == "MAX_FLOW_RATE":
                        if recieved_msg.param_value > 0.3 and recieved_msg.param_value < 5:
                            dataStorageAgri['maxFlowRate'] = recieved_msg.param_value
        
                if recieved_msg.get_type() == "PARAM_REQUEST_READ":
                    if recieved_msg.param_id == "PAYLOAD":
                        msg = mavutil.mavlink.MAVLink_param_value_message("PAYLOAD",
                                                                          dataStorageAgri['remainingPayload'],
                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                          5,
                                                                          1)
                    if recieved_msg.param_id == "CLEARANCE_ALT":
                        msg = mavutil.mavlink.MAVLink_param_value_message("PAYLOAD",
                                                                          dataStorageAgri['clearanceAlt'],
                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                          5,
                                                                          2)
                    if recieved_msg.param_id == "PESTI_PER_ACRE":
                        msg = mavutil.mavlink.MAVLink_param_value_message("PAYLOAD",
                                                                          dataStorageAgri['pesticidePerAcre'],
                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                          5,
                                                                          3)
                    if recieved_msg.param_id == "SWATH":
                        msg = mavutil.mavlink.MAVLink_param_value_message("PAYLOAD",
                                                                          dataStorageAgri['swath'],
                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                          5,
                                                                          4)
                    if recieved_msg.param_id == "MAX_FLOW_RATE":
                        msg = mavutil.mavlink.MAVLink_param_value_message("PAYLOAD",
                                                                          dataStorageAgri['maxFlowRate'],
                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                          5,
                                                                          5)
                    msgList.append(msg)
        
                if recieved_msg.get_type() == "GA3A_MISSION_CMD":
                    if recieved_msg.start_wp > 0 and recieved_msg.end_wp > recieved_msg.start_wp:
                        dataStorageAgri['startWP'] = recieved_msg.start_wp
                        dataStorageAgri['endWP'] = recieved_msg.end_wp
                        if recieved_msg.mission_alt > 100:
                            dataStorageAgri['missionAlt'] = recieved_msg.mission_alt
                        dataStorageAgri['missionYaw'] = recieved_msg.mission_yaw
                        dataStorageAgri['missionOn'] = True
                        dataStorageAgri['RTLLat'] = -2000000000
                        dataStorageAgri['RTLLon'] = -2000000000
                        dataStorageAgri['RTLWP'] = 0
                        write_mission_file()
                    return
        
                if recieved_msg.get_type() == "GA3A_RESUME_CMD":
                    if recieved_msg.do_resume == 1 and not dataStorageCommon['isFlying'] and not dataStorageAgri['resumeOn']:
                        dataStorageAgri['resumeState'] = 1
                        dataStorageAgri['resumeOn'] = True
                    return
        
        
                if recieved_msg.get_type() == "RANGEFINDER":
                    dataStorageAgri['terrainAlt'] = recieved_msg.distance
                    return
                super().handle_recieved_message(recievedMsg)
            else:
                time.sleep(0.01)

# Empty data stroage for reference
dataStorageAgri ={
                  'startWP': 1,
                  'endWP': 2,
                  'missionYaw': 0,
                  'NozzleConfig': 0b00111100,
                  'actualNozzRPM': 0,
                  'actualFlowRate': 0,
                  'remainingPayload': 15,
                  'currentLat': 0,
                  'currentLon': 0,
                  'relativeAlt': 0,
                  'terrainAlt': 0,
                  'RTLLat': -2000000000,
                  'RTLLon': -2000000000,
                  'RTLWP': 0,
                  'clearanceAlt': 1000, #cm
                  'missionAlt': 300,  #cm
                  'missionOn': False,
                  'resumeOn': False,
                  'resumeState': 0,
                  'pesticidePerAcre': 5,
                  'swath': 4,
                  'maxFlowRate': 1.2
                  }



def handle_sensor(schTaskList, dataStorageAgri, lock):
    # Initialize Sensor handling class
    pibStatus = PIBStatus('/dev/ttyDCU')
    flowSensor = FlowSensor()

    # Schedule the tasks
    schTaskList.append(ScheduleTask(0.15, pibStatus.update, dataStorageAgri, lock))
    schTaskList.append(ScheduleTask(0.2, flowSensor.calc_flow_rate, dataStorageAgri, lock))


def resume_mission(dataStorageAgri, mavConnection, mavutil, msgList, lock, resumeSendingCounter):
    sendCount = 10
    logging.info("Resume, %d, %d, %d"%(resumeSendingCounter[0], dataStorageAgri['resumeOn'], dataStorageAgri['resumeState']))
    if dataStorageAgri['resumeOn']:
        # First change to GUIDED mode
        if dataStorageAgri['resumeState'] == 1:
            if mavConnection.flightmode == "GUIDED":
                resumeSendingCounter[0] = 0
                dataStorageAgri['resumeState'] = 2
            else:
                if resumeSendingCounter[0] < sendCount:
                    msg = mavutil.mavlink.MAVLink_set_mode_message(mavConnection.target_system,
                                                                   mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                   4) # GUIDED
                    resumeSendingCounter[0] = resumeSendingCounter[0] + 1
                    with lock:
                        msgList.append(msg)
                return


        # TakeOff
        if dataStorageAgri['resumeState'] == 2:
            if dataStorageAgri['relativeAlt'] > 2:
                resumeSendingCounter[0] = 0
                dataStorageAgri['resumeState'] = 3
            else:
                if resumeSendingCounter[0] < sendCount:
                    msg = mavutil.mavlink.MAVLink_command_long_message(mavConnection.target_system,
                                                                       mavConnection.target_component,
                                                                       mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
                                                                       0, # confirmation
                                                                       0, # param1
                                                                       0, # param2
                                                                       0, # param3
                                                                       0, # param4
                                                                       0, # param5
                                                                       0, # param6
                                                                       3) # param7 (alt)
                    resumeSendingCounter[0] = resumeSendingCounter[0] + 1
                    with lock:
                        msgList.append(msg)
                return

        # Goto Clearance Altitude
        if dataStorageAgri['resumeState'] == 3:
            if dataStorageAgri['terrainAlt'] > (dataStorageAgri['clearanceAlt']/100-1):
                resumeSendingCounter[0] = 0
                dataStorageAgri['resumeState'] = 4
            else:
                if resumeSendingCounter[0] < sendCount:
                    msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, # Time from boot (Irrelevant)
                                                                                         mavConnection.target_system,
                                                                                         mavConnection.target_component,
                                                                                         mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # Frame
                                                                                         0b110111111000, # Command Mask, Only position command
                                                                                         dataStorageAgri['currentLat'], # LAT
                                                                                         dataStorageAgri['currentLon'], # LON
                                                                                         dataStorageAgri['clearanceAlt']/100,    # Alt
                                                                                         0,     # vx
                                                                                         0,     # vy
                                                                                         0,     # vz
                                                                                         0,     # ax
                                                                                         0,     # ay
                                                                                         0,     # az
                                                                                         0,     # yaw
                                                                                         0)     # yaw_rate
                    resumeSendingCounter[0] = resumeSendingCounter[0] + 1
                    with lock:
                        msgList.append(msg)
                return

        # Align heading to mission heading
        if dataStorageAgri['resumeState'] == 4:
            if abs(dataStorageAgri['yaw']-dataStorageAgri['missionYaw']) < 5:
                resumeSendingCounter[0] = 0
                dataStorageAgri['resumeState'] = 5
            else:
                if resumeSendingCounter[0] < sendCount:
                    msg = mavutil.mavlink.MAVLink_command_long_message(mavConnection.target_system,
                                                                       mavConnection.target_component,
                                                                       mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
                                                                       0,                               # confirmation
                                                                       dataStorageAgri['missionYaw'],   # param1 (Angle respect to North)
                                                                       30,                              # param2 (rate deg/s)
                                                                       1,                               # param3 (Clockwise)
                                                                       0,                               # param4 (Absolute frame. North is 0)
                                                                       0,                               # param5
                                                                       0,                               # param6
                                                                       0)                               # param7
                    resumeSendingCounter[0] = resumeSendingCounter[0] + 1
                    with lock:
                        msgList.append(msg)
                return

        # Guide to Top of the RTL point
        if dataStorageAgri['resumeState'] == 5:
            if distance(dataStorageAgri['RTLLat']/1e7, dataStorageAgri['RTLLon']/1e7, dataStorageAgri['currentLat']/1e7, dataStorageAgri['currentLon']/1e7) < 1:
                resumeSendingCounter[0] = 0
                dataStorageAgri['resumeState'] = 6
            else:
                if resumeSendingCounter[0] < sendCount:
                    msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, # Time from boot (Irrelevant)
                                                                                         mavConnection.target_system,
                                                                                         mavConnection.target_component,
                                                                                         mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # Frame
                                                                                         0b110111111000, # Command Mask, Only position command
                                                                                         dataStorageAgri['RTLLat'], # LAT
                                                                                         dataStorageAgri['RTLLon'], # LON
                                                                                         dataStorageAgri['clearanceAlt']/100,    # Alt
                                                                                         0,     # vx
                                                                                         0,     # vy
                                                                                         0,     # vz
                                                                                         0,     # ax
                                                                                         0,     # ay
                                                                                         0,     # az
                                                                                         0,     # yaw
                                                                                         0)     # yaw_rate
                    resumeSendingCounter[0] = resumeSendingCounter[0] + 1
                    with lock:
                        msgList.append(msg)
                return

        # Come down to Actual point
        if dataStorageAgri['resumeState'] == 6:
            if dataStorageAgri['terrainAlt'] < (dataStorageAgri['missionAlt']/100+1):
                resumeSendingCounter[0] = 0
                dataStorageAgri['resumeState'] = 7
            else:
                if resumeSendingCounter[0] < sendCount:
                    msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, # Time from boot (Irrelevant)
                                                                                         mavConnection.target_system,
                                                                                         mavConnection.target_component,
                                                                                         mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # Frame
                                                                                         0b110111111000, # Command Mask, Only position command
                                                                                         dataStorageAgri['RTLLat'], # LAT
                                                                                         dataStorageAgri['RTLLon'], # LON
                                                                                         dataStorageAgri['missionAlt']/100,    # Alt
                                                                                         0,     # vx
                                                                                         0,     # vy
                                                                                         0,     # vz
                                                                                         0,     # ax
                                                                                         0,     # ay
                                                                                         0,     # az
                                                                                         0,     # yaw
                                                                                         0)     # yaw_rate
                    resumeSendingCounter[0] = resumeSendingCounter[0] + 1
                    with lock:
                        msgList.append(msg)
                return

        # Set Current Waypoint
        if dataStorageAgri['resumeState'] == 7:
            if dataStorageAgri['currentWP'] == dataStorageAgri['RTLWP']:
                resumeSendingCounter[0] = 0
                dataStorageAgri['resumeState'] = 8
            else:
                if resumeSendingCounter[0] < sendCount:
                    msg = mavutil.mavlink.MAVLink_mission_set_current_message(mavConnection.target_system,
                                                                              mavConnection.target_component,
                                                                              dataStorageAgri['RTLWP'])
                    resumeSendingCounter[0] = resumeSendingCounter[0] + 1
                    with lock:
                        msgList.append(msg)
                return

        # Engage AUTO mission
        if dataStorageAgri['resumeState'] == 8:
            if mavConnection.flightmode == "AUTO":
                resumeSendingCounter[0] = 0
                dataStorageAgri['resumeOn'] = False
                dataStorageAgri['resumeState'] = 0
            else:
                if resumeSendingCounter[0] < sendCount:
                    msg = mavutil.mavlink.MAVLink_set_mode_message(mavConnection.target_system,
                                                                   mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                   3) # AUTO
                    resumeSendingCounter[0] = resumeSendingCounter[0] + 1
                    with lock:
                        msgList.append(msg)
                return

def write_mission_file():
    with open('agri_mission_file', 'w') as f:
        f.write('%d %d %d %d %d %d %d %d'%(int(dataStorageAgri['missionOn']),
                                       dataStorageAgri['startWP'],
                                       dataStorageAgri['endWP'],
                                       dataStorageAgri['missionAlt'],
                                       dataStorageAgri['missionYaw'],
                                       dataStorageAgri['RTLLat'],
                                       dataStorageAgri['RTLLon'],
                                       dataStorageAgri['RTLWP']))

def update(msgList, mavConnection, lock):
    ############################### Defining Variables ##############################



    global dataStorageAgri

    agriPayload = AgriPayload()



    # handle sensors
    handle_sensor(schTaskList, dataStorageAgri, lock)


    ####################################################################################

    ################################ Initial set Up ####################################
    if path.exists('agri_mission_file'):
        with open('agri_mission_file', 'r') as f:
            for line in f:
                data = line.split()
                if len(data) == 8:
                    with lock:
                        dataStorageAgri['missionOn'] = bool(int(data[0]))
                        dataStorageAgri['startWP'] = int(data[1])
                        dataStorageAgri['endWP'] = int(data[2])
                        dataStorageAgri['missionAlt'] = int(data[3])
                        dataStorageAgri['missionYaw'] = int(data[4])
                        dataStorageAgri['RTLLat'] = int(data[5])
                        dataStorageAgri['RTLLon'] = int(data[6])
                        dataStorageAgri['RTLWP'] = int(data[7])
                break
    resumeSendingCounter = [0]
    ####################################################################################

    # Final infinite loop on tasks which requires data generated/recieved
    # from Scheduled task and takes long time to run
    # This loop should wait for all tasks to be finished before going to next iteration
    while True:
        try:
            # Prevent unnecessary resource usage by this program
            time.sleep(0.2)

            ############################# Sequential Tasks ###############################
            # Rewrite mission file in case of mission is over
            if dataStorageAgri['currentWP'] > dataStorageAgri['endWP'] and (dataStorageAgri['RTLWP']>0):
                if dataStorageAgri['missionOn']:
                    dataStorageAgri['RTLLat'] = -2000000000
                    dataStorageAgri['RTLLon'] = -2000000000
                    dataStorageAgri['RTLWP'] = 0
                    write_mission_file()


            # if mode changes to RTL from AUTO then store the current (Lat Lon) as RTL (Lat Lon)
            with lock:
                if mavConnection.flightmode is 'RTL' and dataStorageAgri['currentMode'] is 'AUTO':
                    dataStorageAgri['RTLLat'] = dataStorageAgri['currentLat']
                    dataStorageAgri['RTLLon'] = dataStorageAgri['currentLon']
                    dataStorageAgri['RTLWP'] = dataStorageAgri['currentWP']
                    write_mission_file()
                    
                # Update Mode
                dataStorageAgri['currentMode'] = mavConnection.flightmode
                

            # Resume Mission Handling
            resume_mission(dataStorageAgri, mavConnection, mavutil, msgList, lock, resumeSendingCounter)

            if dataStorageAgri['resumeOn'] and mavConnection.flightmode is not 'GUIDED' and dataStorageAgri['resumeState'] > 1:
                resumeSendingCounter[0] = 0
                dataStorageAgri['resumeOn'] = False
                dataStorageAgri['resumeState'] = 0

            # update the required flow rate to the agri payload handlere
            agriPayload.update(dataStorageAgri, mavConnection, mavutil, msgList, lock)

            # send the data to GCS
            resumeButtonEnable = False
            if not dataStorageCommon['isFlying'] and dataStorageAgri['missionOn'] and dataStorageAgri['RTLWP']>dataStorageAgri['startWP'] and dataStorageAgri['RTLWP']<=dataStorageAgri['endWP']:
                resumeButtonEnable = True
            msg = mavutil.mavlink.MAVLink_ga3a_payload_status_message(0,
                                                                      0,
                                                                      dataStorageAgri['remainingPayload'],
                                                                      int(resumeButtonEnable))
            with lock:
                msgList.append(msg)


            ##############################################################################
    def kill_all_threads(self):
        logging.info("GA3ACompanionComputer killing all threads")
        super().kill_all_threads()
#        self.killAllThread.set()
#        
#        for task in self.scheduledTaskList:
#            task.stop()
#        
        self.handleRecievedMsgThread.join()
        logging.info("GA3ACompanionComputer joined all threads")