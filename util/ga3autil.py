# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 16:02:00 2019

@author: sachchit
"""

# import necessary modules
# import necessary modules
import time
from util.gacommonutil import CompanionComputer, mavutil, path, dist_between_lat_lon, ScheduleTask, State
import logging
from util.ga3apayloadutil import AgriPayload#, PIBStatus, FlowSensor
import numpy as np
import sys
import threading

class GA3ACompanionComputer(CompanionComputer):
    def __init__(self, sitlType):
        # Initialize super class
        super().__init__(sitlType)

        # Threading Lock for TestCompanionComputer Class
        self.handleRecievedMsgThread = None

        # Spraying Mission Related
        self.startWP = 1
        self.endWP = 2
        self.missionYaw = 0
        self.missionAlt = 3       # m
        self.missionOn = False
        self.previousMode = "UNKNOWN"

        # Resume Mission Related
        self.RTLLat = -200
        self.RTLLon = -200
        self.RTLWP = 0
        self.clearanceAlt = 10    # m
        self.resumeOn = False
        self.resumeState = 0
        self.resumeSendingCounter = 0

        # Payload Related
        self.agriPayload = AgriPayload(self.isSITL)

        # Agri Specific vehicle status
        self.testing = False

        # Read Agri Mission File
        self.read_mission_file()

    def init(self):
        super().init()

        # set data stream rate
        self.set_data_stream()

        # start our recieving message handling loop
        self.handleRecievedMsgThread = threading.Thread(target=self.handle_recieved_message)
        self.handleRecievedMsgThread.start()

        # Start Agri Loop
        self.scheduledTaskList.append(ScheduleTask(0.2, self.update))

        # Record Home Location
#        if not self.isSITL:
#            # Initialize Sensor handling class
#            pibStatus = PIBStatus('/dev/ttyDCU')
#            flowSensor = FlowSensor(self.isSITL)
#
#            # Schedule the tasks
#            self.scheduledTaskList.append(ScheduleTask(0.15, pibStatus.update))
#            self.scheduledTaskList.append(ScheduleTask(0.2, flowSensor.calc_flow_rate))
#
#        else:
#            pass

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
                if recievedMsg.get_type() == "RC_CHANNELS":
                    if recievedMsg.chan7_raw > 1800:
                        self.testing = True
                    else:
                        self.testing = False

                if recievedMsg.get_type() == "PARAM_SET":
                    #paramId = recievedMsg.param_id.strip(b"\x00").decode()
                    paramId = recievedMsg.param_id
                    try:
                        paramId = recievedMsg.param_id.replace(b'\x00',b'')
                        paramId = paramId.decode()
                    except:
                        pass
                    paramValue = recievedMsg.param_value
                    if paramId == "PAYLOAD":
                        print("Payload param received",paramValue)
                        if paramValue > 0 and paramValue < 17:
                            self.agriPayload.remainingPayload = paramValue
                    if paramId == "CLEARANCE_ALT":
                        if paramValue > 200 and paramValue < 4000:
                            self.agriPayload.clearanceAlt = paramValue/100.
                    if paramId == "PESTI_PER_ACRE":
                        if paramValue > 1 and paramValue < 20:
                            self.agriPayload.pesticidePerAcre = paramValue
                    if paramId == "SWATH":
                        if paramValue > 1 and paramValue < 8:
                            self.agriPayload.swath = paramValue
                    if paramId == "MAX_FLOW_RATE":
                        if paramValue > 0.3 and paramValue < 5:
                            self.agriPayload.maxFlowRate = paramValue

                if recievedMsg.get_type() == "PARAM_REQUEST_READ":
                    paramId = recievedMsg.param_id
                    try:
                        paramId = recievedMsg.param_id.replace(b'\x00',b'')
                        paramId = paramId.decode()
                    except:
                        pass
                    if paramId == "PAYLOAD":
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_param_value_message("PAYLOAD".encode(),
                                                                                                          self.agriPayload.remainingPayload,
                                                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                                                          5,
                                                                                                          1))
                    if paramId == "CLEARANCE_ALT":
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_param_value_message("CLEARANCE_ALT".encode(),
                                                                                                          int(self.agriPayload.clearanceAlt*100),
                                                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                                                          5,
                                                                                                          2))
                    if paramId == "PESTI_PER_ACRE":
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_param_value_message("PESTI_PER_ACRE".encode(),
                                                                                                          self.agriPayload.pesticidePerAcre,
                                                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                                                          5,
                                                                                                          3))
                    if paramId == "SWATH":
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_param_value_message("SWATH".encode(),
                                                                                                          self.agriPayload.swath,
                                                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                                                          5,
                                                                                                          4))
                    if paramId == "MAX_FLOW_RATE":
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_param_value_message("MAX_FLOW_RATE".encode(),
                                                                                                          self.agriPayload.maxFlowRate,
                                                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                                                          5,
                                                                                                          5))

                if recievedMsg.get_type() == "GA3A_MISSION_CMD":
                    if recievedMsg.start_wp > 0 and recievedMsg.end_wp > recievedMsg.start_wp:
                        self.startWP = recievedMsg.start_wp
                        self.endWP = recievedMsg.end_wp
                        if recievedMsg.mission_alt > 100:
                            self.missionAlt = recievedMsg.mission_alt/100.
                        self.missionYaw = recievedMsg.mission_yaw
                        self.missionOn = True
                        self.RTLLat = -200
                        self.RTLLon = -200
                        self.RTLWP = 0
                        self.write_mission_file()

                if recievedMsg.get_type() == "GA3A_RESUME_CMD":
                    if recievedMsg.do_resume == 1 and not self.isFlying and not self.resumeOn:
                        self.resumeState = 1
                        self.resumeOn = True

                super().handle_recieved_message(recievedMsg)
            else:
                time.sleep(0.01)

    def resume_mission(self):
        sendCount = 10
        logging.info("Resume, %d, %d, %d"%(self.resumeSendingCounter, self.resumeOn, self.resumeState))
        if self.resumeOn:
            # First change to GUIDED mode
            if self.resumeState == 1:
                if self.currentMode == "GUIDED":
                    self.resumeSendingCounter = 0
                    self.resumeState = 2
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                       mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                                       4) # GUIDED
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return


            # TakeOff
            if self.resumeState == 2:
                if self.relativeAlt > 2:
                    self.resumeSendingCounter = 0
                    self.resumeState = 3
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_command_long_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                           self.mavlinkInterface.mavConnection.target_component,
                                                                                                           mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
                                                                                                           0, # confirmation
                                                                                                           0, # param1
                                                                                                           0, # param2
                                                                                                           0, # param3
                                                                                                           0, # param4
                                                                                                           0, # param5
                                                                                                           0, # param6
                                                                                                           3) # param7 (alt)
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Goto Clearance Altitude
            if self.resumeState == 3:
                if self.terrainAlt > (self.clearanceAlt-1):
                    self.resumeSendingCounter = 0
                    self.resumeState = 4
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, # Time from boot (Irrelevant)
                                                                                                                             self.mavlinkInterface.mavConnection.target_system,
                                                                                                                             self.mavlinkInterface.mavConnection.target_component,
                                                                                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # Frame
                                                                                                                             0b110111111000, # Command Mask, Only position command
                                                                                                                             int(self.lat*1e7), # LAT
                                                                                                                             int(self.lon*1e7), # LON
                                                                                                                             self.clearanceAlt,    # Alt
                                                                                                                             0,     # vx
                                                                                                                             0,     # vy
                                                                                                                             0,     # vz
                                                                                                                             0,     # ax
                                                                                                                             0,     # ay
                                                                                                                             0,     # az
                                                                                                                             0,     # yaw
                                                                                                                             0)     # yaw_rate
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Align heading to mission heading
            if self.resumeState == 4:
                if abs(self.yaw-self.missionYaw) < 5:
                    self.resumeSendingCounter = 0
                    self.resumeState = 5
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_command_long_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                           self.mavlinkInterface.mavConnection.target_component,
                                                                                                           mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
                                                                                                           0,                               # confirmation
                                                                                                           self.missionYaw,   # param1 (Angle respect to North)
                                                                                                           30,                              # param2 (rate deg/s)
                                                                                                           1,                               # param3 (Clockwise)
                                                                                                           0,                               # param4 (Absolute frame. North is 0)
                                                                                                           0,                               # param5
                                                                                                           0,                               # param6
                                                                                                           0)                               # param7
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Guide to Top of the RTL point
            if self.resumeState == 5:
                if dist_between_lat_lon(self.RTLLat, self.RTLLon, self.lat, self.lon) < 1:
                    self.resumeSendingCounter = 0
                    self.resumeState = 6
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, # Time from boot (Irrelevant)
                                                                                                                             self.mavlinkInterface.mavConnection.target_system,
                                                                                                                             self.mavlinkInterface.mavConnection.target_component,
                                                                                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # Frame
                                                                                                                             0b110111111000, # Command Mask, Only position command
                                                                                                                             int(self.RTLLat*1e7), # LAT
                                                                                                                             int(self.RTLLon*1e7), # LON
                                                                                                                             self.clearanceAlt,    # Alt
                                                                                                                             0,     # vx
                                                                                                                             0,     # vy
                                                                                                                             0,     # vz
                                                                                                                             0,     # ax
                                                                                                                             0,     # ay
                                                                                                                             0,     # az
                                                                                                                             0,     # yaw
                                                                                                                             0)     # yaw_rate
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Come down to Actual point
            if self.resumeState == 6:
                if self.terrainAlt < (self.missionAlt+1):
                    self.resumeSendingCounter = 0
                    self.resumeState = 7
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, # Time from boot (Irrelevant)
                                                                                                                             self.mavlinkInterface.mavConnection.target_system,
                                                                                                                             self.mavlinkInterface.mavConnection.target_component,
                                                                                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # Frame
                                                                                                                             0b110111111000, # Command Mask, Only position command
                                                                                                                             int(self.RTLLat*1e7), # LAT
                                                                                                                             int(self.RTLLon*1e7), # LON
                                                                                                                             self.missionAlt,    # Alt
                                                                                                                             0,     # vx
                                                                                                                             0,     # vy
                                                                                                                             0,     # vz
                                                                                                                             0,     # ax
                                                                                                                             0,     # ay
                                                                                                                             0,     # az
                                                                                                                             0,     # yaw
                                                                                                                             0)     # yaw_rate
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Set Current Waypoint
            if self.resumeState == 7:
                if self.currentWP == self.RTLWP:
                    self.resumeSendingCounter = 0
                    self.resumeState = 8
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_mission_set_current_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                                  self.mavlinkInterface.mavConnection.target_component,
                                                                                                                  self.RTLWP)
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Engage AUTO mission
            if self.resumeState == 8:
                if self.currentMode == "AUTO":
                    self.resumeSendingCounter = 0
                    self.resumeOn = False
                    self.resumeState = 0
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                       self.mavlinkInterface.mavConnection.target_component,
                                                                                                       3) # AUTO
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

    def write_mission_file(self):
        with open('agri_mission_file', 'w') as f:
            f.write('%d %d %d %d %d %d %d %d'%(int(self.missionOn),
                                               self.startWP,
                                               self.endWP,
                                               int(self.missionAlt*100),
                                               self.missionYaw,
                                               int(self.RTLLat*1e7),
                                               int(self.RTLLon*1e7),
                                               self.RTLWP))

    def read_mission_file(self):
        if path.exists('agri_mission_file'):
            with open('agri_mission_file', 'r') as f:
                for line in f:
                    data = line.split()
                    if len(data) == 8:
                        with self.lock:
                            self.missionOn = bool(int(data[0]))
                            self.startWP = int(data[1])
                            self.endWP = int(data[2])
                            self.missionAlt = int(data[3])*0.01
                            self.missionYaw = int(data[4])
                            self.RTLLat = int(data[5])*1e-7
                            self.RTLLon = int(data[6])*1e-7
                            self.RTLWP = int(data[7])
                    return

    def update(self):
        # Rewrite mission file in case of mission is over
        if self.currentWP > self.endWP and (self.RTLWP>0):
            if self.missionOn:
                self.RTLLat = -200
                self.RTLLon = -200
                self.RTLWP = 0
                self.write_mission_file()


        # if mode changes to RTL from AUTO then store the current (Lat Lon) as RTL (Lat Lon)
        if self.previousMode == 'RTL' and self.currentMode == 'AUTO':
            self.RTLLat = self.lat
            self.RTLLon = self.lon
            self.RTLWP = self.currentWP
            self.write_mission_file()

        # Resume Mission Handling
        self.resume_mission()

        if self.resumeOn and self.currentMode != 'GUIDED' and self.resumeState > 1:
            self.resumeSendingCounter = 0
            self.resumeOn = False
            self.resumeState = 0

        # update the required flow rate to the agri payload handlere
        self.agriPayload.update(self.testing)

        # send the data to GCS
        resumeButtonEnable = False
        if not self.isFlying and self.missionOn and self.RTLWP>self.startWP and self.RTLWP<=self.endWP:
            resumeButtonEnable = True

        # Payload Status send to GCS
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_ga3a_payload_status_message(0,
                                                                                                  0,
                                                                                                  self.agriPayload.remainingPayload,
                                                                                                  int(resumeButtonEnable)))

        # Update Mode
        self.previousMode = self.currentMode

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
        