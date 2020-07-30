# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 14:21:00 2019

@author: sachchit
"""

# import necessary modules
import time
from threading import Timer
import serial
import logging
from util.npnt import NPNT
import threading
from pymavlink import mavutil
import queue

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

class CompanionComputer(object):
    def __init__(self, sitlType):
        # Instance Mavlink Communication class
        self.mavlinkInterface = MavlinkInterface(sitlType)
        
        # Vehicle Status
        self.rc6 = 0
        self.isFlying = False
        self.isArmed = False
        self.lat = -200e7
        self.lon = -200e7
        self.hdop = 100
        self.globalTime = 0
        self.globalAlt = -1000
        
        # NPNT
        self.npnt = NPNT()
        
        # Variable to kill all threads cleanly
        self.killAllThread = threading.Event()
        self.scheduledTaskList = []
        
    def init(self):
        # Start the main connection to autopilot
        self.mavlinkInterface.create_mavlink_connection()
        
        # Schedule tasks which needs to be done at particular interval
        self.scheduledTaskList.append(ScheduleTask(0.1, self.check_pause))
        self.scheduledTaskList.append(ScheduleTask(1, self.send_heartbeat))
        
        # Update NPNT related Variables
        self.scheduledTaskList.append(ScheduleTask(1, self.update_npnt))
        
    # handle common messages
    def handle_recieved_message(self, recievedMsg):
        if recievedMsg.get_type() == "RC_CHANNELS":
            self.rc6 = recievedMsg.chan6_raw
            return
        
        if recievedMsg.get_type() == "HEARTBEAT":
            if recievedMsg.autopilot == 3:
                sysStatus = recievedMsg.system_status
                isCurrentFlying = False
                isCurrentFlying = sysStatus == 4
                if(self.isFlying and not isCurrentFlying):
                    isCurrentFlying = sysStatus == 5 or sysStatus == 6
                self.isFlying = isCurrentFlying
                
                if (recievedMsg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0:
                    self.isArmed = True
                else:
                    self.isArmed = False
                return
            
        if recievedMsg.get_type() == "GPS_RAW_INT":
            self.lat = recievedMsg.lat
            self.lon = recievedMsg.lon
            self.globalAlt = recievedMsg.alt/1000.
            self.hdop = recievedMsg.eph/100.
            return
        
        if recievedMsg.get_type() == "SYSTEM_TIME":
            self.globalTime = int(recievedMsg.time_unix_usec*1e-6)
            return
        
        if recievedMsg.get_type() == "NPNT_UIN_REGISTER":
            self.npnt.uinChangeRequested = recievedMsg.uin
            return
        
        if recievedMsg.get_type() == "NPNT_KEY_ROTATION":
            self.npnt.keyRotationRequested = True
            return
        
        if recievedMsg.get_type() == "NPNT_REQ_LOGS":
            self.npnt.handle_log_request(recievedMsg.datetime)
            return
        
    def check_pause(self):
        # check if rc 6 is more than 1800
        # if yes change the flag
        if self.rc6 is not None:
            if self.rc6 > 1800:
                self.mavlinkInterface.sendingBlocked = True
            else:
                self.mavlinkInterface.sendingBlocked = False

    def send_heartbeat(self):
        msg = mavutil.mavlink.MAVLink_heartbeat_message(18, 8, 0, 0, 0, 3)
        self.add_new_message_to_sending_queue(msg)
        
    def update_npnt(self):
        self.npnt.update(self.lat, self.lon, self.globalAlt, self.hdop, self.globalTime, self.isArmed)
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_status_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                          self.mavlinkInterface.mavConnection.target_component,
                                                                                          self.npnt.get_npnt_allowed(),
                                                                                          self.npnt.get_npnt_not_allowed_reason()))
        
        # Acknoledgement Messages
        if self.npnt.keyRotationRequested:
            self.npnt.keyRotationRequested = False
            if self.npnt.key_rotation():
                self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_key_rotation_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                        self.mavlinkInterface.mavConnection.target_component,
                                                                                                        1))
            else:
                self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_key_rotation_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                        self.mavlinkInterface.mavConnection.target_component,
                                                                                                        0))
                
        if self.npnt.uinChangeRequested:
            self.npnt.update_uin()
            emptyBytes = [0]*30
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_uin_register_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                    self.mavlinkInterface.mavConnection.target_component,
                                                                                                    1,
                                                                                                    30,
                                                                                                    emptyBytes))
            self.npnt.uinChangeRequested = None
    
    def add_new_message_to_sending_queue(self, msg):
        # Only this method to be used to send the messages
        if self.mavlinkInterface.connected:
            # This if loop to make sure no garbage in queue when there is no 
            # connection
            self.mavlinkInterface.pendingSendMsgList.put(msg)
            
    def get_new_message_from_recieving_queue(self):
        # Only this method to be used to recieve messages
        if not self.mavlinkInterface.recievedMsgQueue.empty():
            return self.mavlinkInterface.recievedMsgQueue.get()
        else:
            return None
    
    def kill_all_threads(self):
        logging.info("CompanionComputer killing all threads")
        self.mavlinkInterface.kill_all_threads()
        self.npnt.kill_all_threads()
        self.killAllThread.set()
        
        for task in self.scheduledTaskList:
            task.stop()
            
        logging.info("CompanionComputer joined all threads")
        
class MavlinkInterface(object):
    # This class handles all the Mavlink Messaging
    def __init__(self, sitlType):
        # Handling for SITL
        self.isSITL = False
        if sitlType is not None:
            self.sitlType = sitlType
            self.isSITL = True
        
        # Initialize connection variables 
        self.mavConnection = None
        self.connected = False
        
        # Threading Lock for Mavlink Class
        self.lock = threading.Lock()
        self.recievingThread = None
        self.sendingThread = None
        
        # Message list
        self.recievedMsgQueue = queue.Queue()
        self.pendingSendMsgList = queue.Queue()
        
        # Blocker for sending message
        self.sendingBlocked = False
        
        # Start AutoReconnect signal slot mechanism
        self.disconnectEvent = threading.Event()
        self.autoReconnectThread = threading.Thread(target=self.auto_reconnect)
        self.autoReconnectThread.start()  
        
        # Variable to kill all threads cleanly
        self.killAllThread = threading.Event()
        
        
    def auto_reconnect(self):
        # If we recieve disconnection signal, set the connected status to
        # False and try restarting the connection
        logging.info("Auto Reconnection Functionality Started")
        while True:
            try:
                self.disconnectEvent.wait()
                if self.killAllThread.is_set():
                    break
                self.connected = False
                self.create_mavlink_connection()
                self.disconnectEvent.clear()
                
            # During debugging so that we can exit the loop
            except KeyboardInterrupt:
                self.kill_all_threads()
                break
            
    def create_mavlink_connection(self):
        # Close already existing connection
        if self.mavConnection is not None:
            self.mavConnection.close()
        
        logging.info("Starting Mavlink Connection")
        while not self.connected:
            if self.killAllThread.is_set():
                break
            try:
                time.sleep(1)
                # Create the connection
                if self.sitlType == 'tcp':
                    self.mavConnection = mavutil.mavlink_connection('tcp:127.0.0.1:5760',
                                                               source_system=1,
                                                               source_component=10)
                elif self.sitlType == 'udp':
                    self.mavConnection = mavutil.mavlink_connection('udp:127.0.0.1:14551',
                                                               source_system=1,
                                                               source_component=10)
                elif self.sitlType == 'com':
                    self.mavConnection = mavutil.mavlink_connection('COM28',
                                                               baud=57600,
                                                               source_system=1,
                                                               source_component=10)
                else:
                    # Need to provide the serial port and baudrate
                    self.mavConnection = mavutil.mavlink_connection('/dev/serial0',
                                                               baud=921600,
                                                               source_system=1,
                                                               source_component=10)
            
            # During debugging so that we can exit the loop
            except KeyboardInterrupt:
                self.kill_all_threads()
                break
            
            except Exception as e:
                logging.exception(e)
                if self.mavConnection is not None:
                    self.mavConnection.close()
            else:
                # wait for the heartbeat msg to find the system ID
                self.mavConnection.wait_heartbeat()
                logging.info("Mavlink Connection Established")
            
                self.connected = True
                self.recievingThread = threading.Thread(target=self.recieving_loop)
                self.sendingThread = threading.Thread(target=self.sending_loop)
                self.recievingThread.start()
                self.sendingThread.start()
    
    # Mavlink recieveng loop
    def recieving_loop(self):
        logging.info("starting recieving loop")
        while self.connected:
            if self.killAllThread.is_set():
                break
            try:
                # Recieve the messages
                time.sleep(0.001)
                recieved = self.mavConnection.recv_match()
    
                # If empty message ignore
                if recieved is not None:
                    self.recievedMsgQueue.put(recieved)
            
            # During debugging so that we can exit the loop
            except KeyboardInterrupt:
                self.kill_all_threads()
                break
            
            except:
                if not self.disconnectEvent.is_set():
                    self.disconnectEvent.set()
    
    # Mavlink Sending Loop                
    def sending_loop(self):
        logging.info("starting sending loop")
        while self.connected:
            if self.killAllThread.is_set():
                break
            if not self.pendingSendMsgList.empty():
                try:
                    msg = self.pendingSendMsgList.get()
                    if not self.sendingBlocked:
                        self.mavConnection.mav.send(msg)
                        
                # During debugging so that we can exit the loop
                except KeyboardInterrupt:
                    self.kill_all_threads()
                    break
                
                except:
                    if not self.disconnectEvent.is_set():
                        self.disconnectEvent.set()
            else:
                # If there is no pending message to be sent wait for 0.05 seconds
                time.sleep(0.05)
                
    def kill_all_threads(self):
        logging.info("MavlinkInterface killing all threads")
        self.disconnectEvent.set()
        self.killAllThread.set()
        self.autoReconnectThread.join()
        self.recievingThread.join()
        self.sendingThread.join()
        logging.info("MavlinkInterface joined all threads")
