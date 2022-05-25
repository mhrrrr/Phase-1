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


class TestCompanionComputer(CompanionComputer):
    def __init__(self, sitlType, sitlPort):
        # Initialize super class
        super().__init__(sitlType, sitlPort)
        
        # Threading Lock for TestCompanionComputer Class
        self.lock = threading.Lock()
        self.handleRecievedMsgThread = None

        #Initialise SITL driver
        self.lidar = driver.SensorDriver('SITL')

        #Connect to the listener - ensure the listener is running in background!!
        self.lidar.connect_and_fetch()

        #Front sensor
        # self.front_sensor = estimation.Sensor(1,1*math.pi/180,12,0.01,0)
        #SITL
        self.front_sensor = estimation.Sensor(1,0.03098,40,1,-0.976)

        #Initialise pre processor
        self.coordinate_transform = estimation.DataPreProcessor()

        #initialise navigation controller
        self.navigation_controller = control.ObstacleAvoidance(max_obs=40)

        self.navigation_map = estimation.DataPostProcessor()


        #Brake
        self.brake = 0
        self.alreadybraked = 0
        self.initvar=1

        

        
    def init(self):
        super().init()

        # set data stream rate
        self.set_data_stream()
        
        # start our recieving message handling loop
        self.handleRecievedMsgThread = threading.Thread(target=self.handle_recieved_message)
        self.handleRecievedMsgThread.start()

        ### Starting reading threads as they are while loops ###
        # t1 = threading.Thread(target=self.lidar.give_scan_values)
        # t1.start()
        # t2 = threading.Thread(target=self.lidar.read_fast)
        # t2.start()
        
        
        # set data stream rate
        self.set_data_stream()
        
        # start our recieving message handling loop
        self.handleRecievedMsgThread = threading.Thread(target=self.handle_recieved_message)
        self.handleRecievedMsgThread.start()

        #Scheduled the threads
        self.scheduledTaskList.append(ScheduleTask(0.0006, self.lidar.update_sitl_sensor))
        self.scheduledTaskList.append(ScheduleTask(0.0001, self.update_vars))
        #self.scheduledTaskList.append(ScheduleTask(0.000000000000000001, self.lidar.give_scan_values))
        #self.give_scan_values()
        # self.scheduledTaskList.append(ScheduleTask(0.00002,self.lidar.update_rplidar))
        self.scheduledTaskList.append(ScheduleTask(0.02,self.front_sensor.handle_raw_data))
        self.scheduledTaskList.append(ScheduleTask(0.02, self.coordinate_transform.update_vehicle_states))
        self.scheduledTaskList.append(ScheduleTask(0.02, self.coordinate_transform.convert_body_to_inertial_frame))

        self.scheduledTaskList.append(ScheduleTask(0.01, self.navigation_controller.predict_pos_vector))
        self.scheduledTaskList.append(ScheduleTask(0.01, self.navigation_controller.basic_stop))
        self.scheduledTaskList.append(ScheduleTask(0.01, self.handbrake))
        self.scheduledTaskList.append(ScheduleTask(0.005,self.navigation_stack))
        self.scheduledTaskList.append(ScheduleTask(0.005,self.navigation_map.forget_far_obstacles))
        # self.scheduledTaskList.append(ScheduleTask(0.05,self.debug))



        while True:
            if self.navigation_controller.obstacle_map is None:
                pass
            else:
                time.sleep(0.01)

    def handbrake(self):
        if self.initvar==1:
            print("Handbrake Initialize")
            self.initvar = 0
        #Only brake when previously brake command is not given, altitude is greater than 1
        if self.brake and not self.alreadybraked and self.relativeAlt>1:
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           17))    
            self.alreadybraked = 1

    def debug(self):
        """Debugger if you want to print something
        """
        print(f"New:  {self.navigation_controller.obstacle_map}")
        


    def update_vars(self):
        """This function acts as a bridge between different class to transfer data. Part of the requirements for 
        developing the algorithm
        """
        #6.198883056640625e-06 seconds
        self.front_sensor.data = self.lidar.raw_data

        #Gazebo axes are different from my 
        # axis, thus I have to handle it. 
        if self.lidar.drivername == 'SITL':
            self.coordinate_transform.roll = self.roll + math.pi
        else:
            self.coordinate_transform.roll = self.roll
        self.coordinate_transform.pitch = self.pitch
        #Yaw wrapper to keep it compatible with my transformations
        self.coordinate_transform.yaw = math.atan2(math.sin(self.yaw),math.cos(self.yaw))
        self.coordinate_transform.px = self.px
        self.coordinate_transform.py = self.py
        self.coordinate_transform.pz = self.relativeAlt
        self.navigation_controller.px = self.px
        self.navigation_controller.py = self.py
        self.navigation_controller.vx = self.vx
        self.navigation_controller.vy = self.vy
        self.brake = self.navigation_controller.brake
        self.coordinate_transform.x = self.front_sensor.X
        self.coordinate_transform.y = self.front_sensor.Y
        self.navigation_controller.mode = self.currentMode
        self.navigation_map.px = self.px
        self.navigation_map.py = self.py


    def navigation_stack(self):
        """Update vars function was getting filled....
        I am making another thread. 
        """

        #Angular inertial obstacle vector to grid based reading and that is stored in global map
        self.navigation_map.convert_rel_obstacle_to_inertial(self.navigation_map.grid(self.coordinate_transform.obstacle_vector_inertial.T))

        #Relative obstacle is sent to the navigation algorithm
        self.navigation_controller.obstacle_map = self.navigation_map.convert_inertial_to_rel()


         
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