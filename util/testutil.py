# -*- coding: utf-8 -*-
"""
Created on Wed Jul  8 14:07:56 2020

@author: Sachchit Vekaria
@Organization: General Aeronautics Pvt Ltd
"""

# import necessary modules
import time
from unittest import FunctionTestCase
from util.gacommonutil import CompanionComputer, mavutil, ScheduleTask
import threading
import logging


# # # Another module
import math
import numpy as np
import util.VectorMath as vmath
import util.SAADriver as driver
import util.SAADataHandling as estimation
import util.SAAController as control
import matplotlib.pyplot as plt #yes


class TestCompanionComputer(CompanionComputer):
    def __init__(self, sitlType, sitlPort):
        # Initialize super class
        super().__init__(sitlType, sitlPort)
        
        # Threading Lock for TestCompanionComputer Class
        self.lock = threading.Lock()
        self.handleRecievedMsgThread = None

        self.lidar = driver.SensorDriver('SITL')
        self.lidar.connect_and_fetch()
        
        self.front_sensor = estimation.Sensor(1,0.03098,40,1,-0.976)
        self.coordinate_transform = estimation.DataPreProcessor()

        self.navigation_controller = control.ObstacleAvoidance()

        

        
    def init(self):
        super().init()
        


        # set data stream rate
        self.set_data_stream()
        
        # start our recieving message handling loop
        self.handleRecievedMsgThread = threading.Thread(target=self.handle_recieved_message)
        self.handleRecievedMsgThread.start()

        self.scheduledTaskList.append(ScheduleTask(0.12, self.lidar.update))
        self.scheduledTaskList.append(ScheduleTask(0.1, self.update_vars))
        
        self.scheduledTaskList.append(ScheduleTask(0.12,self.front_sensor.handle_raw_data))
        self.scheduledTaskList.append(ScheduleTask(0.01, self.coordinate_transform.update_vehicle_states))
        self.scheduledTaskList.append(ScheduleTask(0.01, self.coordinate_transform.convert_body_to_inertial_frame))

        self.scheduledTaskList.append(ScheduleTask(0.01, self.navigation_controller.predict_pos_vector))
        self.scheduledTaskList.append(ScheduleTask(0.1, self.navigation_controller.basic_stop))
        self.scheduledTaskList.append(ScheduleTask(0.1, self.handbrake))
        

        # self.scheduledTaskList.append(ScheduleTask(0.1, self.debug))
        
        while True:
            time.sleep(1)
    def handbrake(self):
        if self.brake:
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           17))    

    def debug(self):
        print(self.coordinate_transform.obstacle_vector_inertial)


    def update_vars(self):
        #6.198883056640625e-06 seconds
        self.front_sensor.data = self.lidar.raw_data
        self.coordinate_transform.roll = self.roll
        self.coordinate_transform.pitch = self.pitch
        self.coordinate_transform.yaw = self.yaw
        self.coordinate_transform.px = self.px
        self.coordinate_transform.py = self.py
        self.coordinate_transform.pz = self.relativeAlt
        self.coordinate_transform.x = self.front_sensor.X
        self.coordinate_transform.y = self.front_sensor.Y
        self.navigation_controller.obstacle_map = self.coordinate_transform.obstacle_vector_inertial.T
        self.navigation_controller.px = self.px
        self.navigation_controller.py = self.py
        self.navigation_controller.vx = self.vx
        self.navigation_controller.vy = self.vy
        self.brake = self.navigation_controller.brake
         
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
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1))
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 1, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2, 1))
    
        logging.info("Stream Rate have been set")
    
    def handle_recieved_message(self):
        while True:
            if self.killAllThread.is_set():
                break
            recievedMsg = self.get_new_message_from_recieving_queue()
            if recievedMsg is not None:
                super().handle_recieved_message(recievedMsg)
            else:
                time.sleep(0.01)
            
    def kill_all_threads(self):
        logging.info("TestCompanionComputer killing all threads")
        super().kill_all_threads()
#        self.killAllThread.set()
#        
#        for task in self.scheduledTaskList:
#            task.stop()
#        
        self.handleRecievedMsgThread.join()
        logging.info("TestCompanionComputer joined all threads")