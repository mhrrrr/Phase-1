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
from util.npnt import NPNT, listdir, path
import threading
from pymavlink import mavutil
import queue
import struct

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
        
        # FTP
        self.ftp = FTP()
        
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
            self.npnt.uinChangeRequested = ''.join(map(chr,recievedMsg.uin[0:recievedMsg.size]))
            return
        
        if recievedMsg.get_type() == "NPNT_KEY_ROTATION":
            self.npnt.keyRotationRequested = True
            return
        
        if recievedMsg.get_type() == "NPNT_REQ_LOGS":
            self.npnt.logDownloadRequest = True
            self.npnt.logDownloadDateTime = ''.join(map(chr,recievedMsg.date_time[0:15]))
            return
        
        if recievedMsg.get_type() == "NPNT_RFM_DETAIL":
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_rfm_detail_message(0,
                                                                                                  0,
                                                                                                  self.npnt.firmwareVersion,
                                                                                                  len(self.npnt.firmwareVersion),
                                                                                                  self.npnt.firmwareHash,
                                                                                                  len(self.npnt.firmwareHash),
                                                                                                  self.npnt.rpasId,
                                                                                                  len(self.npnt.rpasId),
                                                                                                  self.npnt.rpasModelId,
                                                                                                  len(self.npnt.rpasModelId),
                                                                                                  self.uin,
                                                                                                  len(self.uin)))
            return
        
        if recievedMsg.get_type() == "FILE_TRANSFER_PROTOCOL":
            replyPayload = self.ftp.handle_ftp_message(recievedMsg.payload)
            if replyPayload:
                self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_file_transfer_protocol_message(0,
                                                                                                             255,
                                                                                                             0,
                                                                                                             replyPayload))
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
        self.npnt.update(self.lat, self.lon, self.globalAlt, self.hdop, self.globalTime, self.isArmed, self.ftp.latestUploadedPAFile)
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_status_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                          self.mavlinkInterface.mavConnection.target_component,
                                                                                          self.npnt.get_npnt_allowed(),
                                                                                          self.npnt.get_npnt_not_allowed_reason()))
        
        # Acknoledgement Messages
        if self.npnt.keyRotationRequested:
            self.npnt.keyRotationRequested = False
            confirm = int(self.npnt.key_rotation())
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_key_rotation_message(255,
                                                                                                    0,
                                                                                                    confirm))
                
        if self.npnt.uinChangeRequested:
            self.npnt.update_uin()
            emptyBytes = [0]*30
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_uin_register_message(255,
                                                                                                    0,
                                                                                                    1,
                                                                                                    30,
                                                                                                    emptyBytes))
            self.npnt.uinChangeRequested = None
            
        if self.npnt.logDownloadRequest:
            confirm = 0
            self.npnt.logDownloadRequest = False
            if len(self.npnt.logDownloadDateTime) == 15:
                self.npnt.start_bundling()
                self.npnt.logDownloadDateTime = None
                confirm = 1
                
            emptyBytes = [0]*15
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_req_logs_message(255,
                                                                                                0,
                                                                                                confirm,
                                                                                                emptyBytes))
    
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
                recieved = self.mavConnection.recv_match()
    
                # If empty message ignore
                if recieved is not None:
                    self.recievedMsgQueue.put(recieved)
                else:
                    time.sleep(0.001)
            
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
                time.sleep(0.01)
                
    def kill_all_threads(self):
        logging.info("MavlinkInterface killing all threads")
        self.disconnectEvent.set()
        self.killAllThread.set()
        self.autoReconnectThread.join()
        self.recievingThread.join()
        self.sendingThread.join()
        logging.info("MavlinkInterface joined all threads")

class FTP(object):
    def __init__(self):
        self.recievedFile = None
        self.latestUploadedPAFile = None
        
        self.sentFile = None
        self.sentFileSize = -1
    
    def update_recieved_file_status(self):
        if "NPNT/Permission_Artefact" in self.recievedFile:
            self.latestUploadedPAFile = self.recievedFile.split("/")[-1]
            
        self.recievedFile = None
        
    def update_sent_file_status(self):
        self.sentFile = None
        self.sentFileSize = -1
        
    def handle_ftp_message(self, payload):
        sessionid =  payload[2]
        opcode = payload[3]
        size = payload[4]
        reqopcode = payload[5]
        burstComplete = payload[6]
        padding = payload[7]
        contentOffset = struct.unpack('I',bytearray([payload[8],payload[9],payload[10],payload[11]]))[0]
        data = ''.join(map(chr,payload[12:12+size]))

        replyPayload = None
    
        if(opcode == 6 or opcode == 7):
            replyPayload = self.save_file(opcode, contentOffset, data)
        if(opcode == 1):
            replyPayload = self.terminate_session()
        if(opcode==3):
            replyPayload = self.send_directory_list(contentOffset, data)
        if(opcode==4 or opcode==5):
            replyPayload = self.send_file(opcode, contentOffset, size, data)
            
        return replyPayload
    
    def remove_file(self):
        pass
    
    def save_file(self, opcode, contentOffset, data):
        payloadVal = [0]*251
        payloadVal[3] = 128
        if (opcode == 6):
            self.recievedFile=data
            
            with open(self.recievedFile,'w') as f:
                pass
            payloadVal[5] = 6
        if (opcode == 7):
            with open(self.recievedFile,'a') as f:
                f.seek(contentOffset)
                f.write(data)
            payloadVal[5] = 7
            
        return payloadVal
    
    def send_file(self, opcode, contentOffset, size, data):
        payloadVal = [0]*251
        payloadVal[3]=128
        if (opcode==4):
            self.sentFile=data
            if path.isfile(self.sentFile):
                with open(self.sentFile,'r') as f:
                    self.sentFileSize = len(f.read())
            else:
                self.sentFileSize=0
                payloadVal[3]=129
            payloadVal[4]=4
            payloadData = struct.pack('I',self.sentFileSize)
            for j in range(0,len(payloadData)):
                payloadVal[12+j] = payloadData[j]#struct.unpack('B', payloadData[j])[0]
            payloadVal[5]=4
        if (opcode==5):
            if(contentOffset < self.sentFileSize):
                charCount=size
                if(contentOffset+size > self.sentFileSize):
                   charCount = self.sentFileSize - contentOffset
                with open(self.sentFile,'r') as f:
                    fileString = f.read(contentOffset)
                    fileString = f.read(charCount)
                payloadVal[5]=5
                payloadVal[4]=charCount
                payloadData  = list(fileString.encode())
                for j in range(0,charCount):
                    payloadVal[12+j] = payloadData[j]#struct.unpack('B', payloadData[j])[0]
    
    
            else:
                payloadVal[3]=129
                payloadVal[5]=5
    
        return payloadVal
    
    def terminate_session(self):
        payloadVal = [0]*251
        payloadVal[3] = 128
        payloadVal[5] = 1
        # Update Which file is uploaded/downloaded for other codes to handle 
        # the relevant actions
        if self.recievedFile:
            self.update_recieved_file_status()
        if self.sentFile:
            self.update_sent_file_status()
            
        return payloadVal
    
    def send_directory_list(self, contentOffset, data):
        # Sending only list of files
        payloadVal = [0]*251
        payloadVal[5]=3
        onlyfiles=[]
        if path.isdir(data):
            onlyfiles = [f for f in listdir(data) if path.isfile(path.join(data, f))]
        dirListString=''
        if(contentOffset<len(onlyfiles)):
            for i in range(contentOffset,len(onlyfiles)):
                if(len(dirListString)==0):
                    if(len(dirListString) + len(onlyfiles[i]) <=200):
                        dirListString = dirListString + onlyfiles[i]
                else:
                    if(len(dirListString) + len(':') + len(onlyfiles[i]) <=200):
                        dirListString = dirListString + ':' + onlyfiles[i]
            payloadVal[3]=128
            payloadData  = list(dirListString.encode())
            size=len(payloadData)
            payloadVal[4]=size
            for j in range(0,size):
                payloadVal[12+j] = payloadData[j]#struct.unpack('B', payloadData[j])[0]
        else:
            payloadVal[3]=129
            size=1
            payloadVal[4]=size
            payloadVal[12]=6
            
        return payloadVal

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