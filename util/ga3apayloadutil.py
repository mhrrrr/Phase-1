# -*- coding: utf-8 -*-
"""
Created on Fri Mar  8 12:22:57 2019

@author: sachchit
"""

import numpy as np
import time
import struct
import serial

class AgriPayload:
    def __init__(self):        
        # PIB Input
        self.pumpPWM = 0
        self.micromiserPWM = 0
        
        # Status
        self.remainingPayload = 0.                   # in liters
        self.flowRate = 0.                           # in ltr/min
        self.shouldSpraying = False
        
        # pump parameters
        self.pumpMaxPWM = 1900
        self.pumpMinPWM = 1250
        self.pumpMaxFlowRate = 2.5                  # in ltr/min
        self.pumpMinFlowRate = 0.9                  # in ltr/min
        self.pumpPWM = 0
        
        # Nozzle parameters
        self.nozzMaxPWM = 2000
        self.nozzMinPWM = 1000
        self.nozzMinPS = 85                 # Particle Size in micrometer
        self.nozzPWM = 0                    # Current PWM of Nozzle
        self.nozzNum = 4                    # number of nozzles
        self.nozzMaxFlowRate = 0.3          # in ltr/min
        self.nozzMinFlowRate = 0.1          # in ltr/min
        self.nozzType = 'Micromiser'
        self.nozzP = 0.01
        
        # Overall Parameters
        self.perAcrePestiscide = 5          # in ltr
        self.swath = 2.4                    # in m
        self.maxFlowRate = 1.2              # in ltr/min
        self.maxSpeedSentCount = 0
        self.sprayDensity = self.perAcrePestiscide/4047.
        self.maxSpeed = self.maxFlowRate/60./self.swath/self.sprayDensity + 0.15
        
        # Time related
        self.time = time.time()
        
    def calc_remaining_payload(self):
        if self.remainingPayload > 0:
            self.remainingPayload = self.remainingPayload - self.dt * (self.flowRate/60.)
            
    def update_time(self):
        currTime = time.time() 
        self.dt = currTime - self.time
        self.time = currTime
        
    def update(self, dataStorageAgri, mavConnection, mavutil, msgList, lock):        
        # update timer
        self.update_time()
        
        # Set correct top speed of vehicle
        self.set_vehicle_max_speed(mavConnection, mavutil, msgList, lock)
        
        # calculate speed of the vehicle
        speed = calc_speed(dataStorageAgri, lock)
        
        # RPM
        actualRPM = dataStorageAgri['actualNozzRPM']
        
        # Are we testing
        testing = dataStorageAgri['testing']
        
        # check whether we should be spraying
        if dataStorageAgri['currentWP'] <= dataStorageAgri['endWP'] and dataStorageAgri['currentWP'] >= dataStorageAgri['startWP'] and dataStorageAgri['endWP']>1:
            self.shouldSpraying = True
        else:
            self.shouldSpraying = False
        
        # update max speed of vehicle
        self.set_vehicle_max_speed(mavConnection, mavutil, msgList, lock)
        
        if self.shouldSpraying:
            # calculate flow rate
            self.calc_flow_rate(speed)
            
            # calculate pwm for pump and nozzle
            self.calc_pump_pwm()
            self.calc_nozz_pwm(actualRPM)
            
        else:
            if testing:
                self.nozzPWM = 1500
                self.pumpPWM = 1500
            else:
                self.nozzPWM = self.nozzMinPWM
                self.pumpPWM = self.pumpMinPWM
                
        # update the pwm in FCS
        self.update_pwm(mavConnection, mavutil, msgList, lock)
        
    def calc_flow_rate(self, speed):
        # sanity check of speed
        if (speed < 0):
            speed = 0
        if (speed > (self.maxSpeed+1)):
            speed = self.maxSpeed+1
        
        # Calculate flow rate
        self.flowRate = 60*self.swath*self.sprayDensity*speed

        
        # check flow rate limit
        if self.flowRate > self.maxFlowRate:
            self.flowRate = self.maxFlowRate
        if self.flowRate < self.pumpMinFlowRate:
            self.flowRate = self.pumpMinFlowRate
            
    def calc_pump_pwm(self):
        self.pumpPWM = self.pumpMinPWM + int(float(self.pumpMaxPWM - self.pumpMinPWM)*float(self.flowRate - self.pumpMinFlowRate)/float(self.pumpMaxFlowRate - self.pumpMinFlowRate))
        
    def calc_nozz_pwm(self, actualRPM):
        
        if self.nozzType == 'Micromiser':
            # handle garbage value
            if actualRPM < 0:
                actualRPM = 0
            if actualRPM > 20000:
                actualRPM = 20000
            
            # Calculate Target RPM
            nozzFlow = self.flowRate/self.nozzNum
            if nozzFlow < self.nozzMinFlowRate:
                nozzFlow = self.nozzMinFlowRate
            if nozzFlow > self.nozzMaxFlowRate:
                nozzFlow = self.nozzMaxFlowRate
            
            # curve fit equation RPM = A * (PS - C) ^ B from Micromiser 10 Datasheet
            equationCoeefsA = 89248 + (nozzFlow - 0.3) / (0.1 - 0.3) * (180412 - 89248)
            equationCoeefsB = -0.732 + (nozzFlow - 0.3) / (0.1 - 0.3) * (-0.858 + 0.732)
            equationCoeefsC = 80 + (nozzFlow - 0.3) / (0.1 - 0.3) * (42 - 80)
            
            if (self.nozzMinPS - equationCoeefsC) > 1:
                nozzTargetRPM = equationCoeefsA * (self.nozzMinPS - equationCoeefsC)**equationCoeefsB
            else:
                nozzTargetRPM = 0
                
            # P control for maintaing target PWM
            self.nozzPWM = int(self.nozzPWM + self.nozzP * (nozzTargetRPM - actualRPM))
            
            if self.nozzPWM > self.nozzMaxPWM:
                self.nozzPWM = self.nozzMaxPWM
            if self.nozzPWM < self.nozzMinPWM:
                self.nozzPWM = self.nozzMinPWM
    
    def set_vehicle_max_speed(self, mavConnection, mavutil, msgList, lock):
        # send the message 3 times to be sure message reaches autopilot
        if (self.maxSpeedSentCount < 3):
            # calculate max speed of vehicle
            self.sprayDensity = self.perAcrePestiscide/4047.
            self.maxSpeed = self.maxFlowRate/60./self.swath/self.sprayDensity + 0.15
            
            if self.maxSpeed > 8:
                self.maxSpeed = 8

            # create message
            msg = mavutil.mavlink.MAVLink_param_set_message(mavConnection.target_system, mavConnection.target_component, 'WPNAV_SPEED', self.maxSpeed*100, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            
            with lock:
                # append message to send list
                msgList.append(msg)
            
            # update counter
            self.maxSpeedSentCount = self.maxSpeedSentCount + 1
            
    
    def update_pwm(self, mavConnection, mavutil, msgList, lock):
        # Redundant check to prevent unrealistic value to go to the FCS
        if self.nozzPWM > 2000:
            self.nozzPWM = 2000
        if self.nozzPWM < 1000:
            self.nozzPWM = 1000
        if self.pumpPWM > 2000:
            self.pumpPWM = 2000
        if self.pumpPWM < 1000:
            self.pumpPWM = 1000
            
        # create message
        pumpPWMMsg = mavutil.mavlink.MAVLink_command_long_message(mavConnection.target_system,
                                                                  mavConnection.target_component,
                                                                  mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                                                  0,
                                                                  9,
                                                                  self.pumpPWM,
                                                                  0,
                                                                  0,
                                                                  0,
                                                                  0,
                                                                  0)
        nozzPWMMsg = mavutil.mavlink.MAVLink_command_long_message(mavConnection.target_system,
                                                                  mavConnection.target_component,
                                                                  mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                                                  0,
                                                                  10,
                                                                  self.nozzPWM,
                                                                  0,
                                                                  0,
                                                                  0,
                                                                  0,
                                                                  0)
        
        with lock:
            # append message to send list
            msgList.append(pumpPWMMsg)
            msgList.append(nozzPWMMsg)
        
###############################################################################
        
class PIBStatus:
    # This class store the current status of the PIB board
    #
    # Also, it will have additional functionalities like storing for future
    # data analysis
    def __init__(self, serialPort):
        self.serial = serialPort
        
        # All data recieved from PIB is stored in this dictionary
        self.status = {'ATOMIZER_RPM': np.zeros(6),
                       'ATOMIZER_CURRENT': np.zeros(6),
                       'PUMP_CURRENT': np.zeros(2),
                       'FLOW_METER_READING': np.zeros(2)}
        
        # Status of the error
        # Whether currently in error or not
        self.errorNow =  {'INVALID_START_BIT': False,
                          'INVALID_FUNCTION_CODE': False,
                          'INVALID_LRC_PIB': False,         # PIB side
                          'HEALTH_CHECK_FAIL': False,
                          'WRONG_DATA': False,              # RPi side
                          'INVALID_LRC_RPI': False}        # RPi side
        self.errorList = {'INVALID_START_BIT': 5,
                          'INVALID_FUNCTION_CODE': 6,
                          'INVALID_LRC_PIB': 7,         # PIB side
                          'HEALTH_CHECK_FAIL': 8,
                          'WRONG_DATA': 9,              # RPi side
                          'INVALID_LRC_RPI': 10}        # RPi side
        self.errorStartTime = 0 # start time of error
        self.errorTime = 0  # Time since error without resolvement
        
        # definition of function codes
        self.functionCode = {'MSG1': 1,         # Have data from PIB
                             'MSG2': 2,         # Acknowledgement from PIB for recieving messege from RPi
                             'MSG3': 3}         # Nozzle selection
        
        # data not recieved counter
        self.dataNotRecievedStartTime = 0 # start time after which no data is coming
        self.dataNotRecievedTime = 0      # time till no data is recieved
        
        # Timer to count timing of events
        self.timer = time.time()
        
        # Nozzle configuration
        self.nozzleConfiguration = 0b11110000
        
        # Initialize
        self.init()
        
    def init(self):
        while True:
            # keep trying  to open port unitl succesful
            try:
                time.sleep(1)
                self.ser = serial.Serial(port=self.serial,
                                         baudrate=115200,
                                         parity=serial.PARITY_ODD,
                                         stopbits=serial.STOPBITS_ONE,
                                         bytesize=serial.EIGHTBITS,
                                         timeout=0.1)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
                
    def update(self, dataStorageAgri, lock):
        # read the data
        data = self.ser.readline()
        
        # update the status
        self.decode_data(data)
        
        # transfer data to dataStorageAgri
        self.update_veh_data(dataStorageAgri, lock)

        # Send the nozzle configuration
        if dataStorageAgri['NozzleConfig'] != self.nozzleConfiguration:
            self.set_nozzle_config(dataStorageAgri['NozzleConfig'])
            self.send_nozzle_config()
            
    def update_veh_data(dataStorageAgri, lock):
        with lock:
            pass

    def decode_data(self, data):
        
        # update the timer
        self.timer = time.time()
        
        # if data is not recieved, store how long it has been without data
        if not data:
            if self.dataNotRecievedStartTime == 0:
                self.dataNotRecievedStartTime = int(self.timer)
            self.dataNotRecievedTime = int(self.timer() - self.dataNotRecievedStartTime)
            return
        else:
            # if data is recieved reset the counters
            self.dataNotRecievedStartTime = 0
            self.dataNotRecievedTime = 0
        
        # extract data
        extractedData = self.extract_data(data)
        
        # process data only if something is there in the data
        if extractedData:
            
            if (len(extractedData) is not 3):
                
                # Check LRC
                lrc = self.calc_lrc(data)
                if (lrc is extractedData[-2]):
                    self.errorNow['INVALID_LRC_RPI'] = False
                    
                    self.update_data(extractedData)
                else:
                    self.errorNow['INVALID_LRC_RPI'] = True
            else:
                if extractedData[1] == 2:
                    print("message reached PIB")
                else:
                    for key in self.errorList.keys():
                        if self.errorList[key] == extractedData[1]:
                            self.errorNow[key] = True

        
    def extract_data(self, data):
        acceptedLengths = [32,3]
        dataLength = len(data)
        
        # Reject data with
        # first character not being $
        # length of data not matching to any of the packet size expected by us
        # second character is out of bound of acceptable function codes
        print("1")
        if (data[0] == '$') and (dataLength not in acceptedLengths) and (ord(data[1])<8):
            print("2")
            self.errorNow['WRONG_DATA'] = False
            
            if dataLength == 3:
                extractedData = struct.unpack('cbc', data)
                return extractedData
            else:
                extractedData = struct.unpack('cbhhhhhhhhhhhhhhbc', data)
                return extractedData
                
        else:
            print("3")
            self.errorNow['WRONG_DATA'] = True
            return None
        
    def calc_lrc(self, data):
        lrc = 0
        if type(data) == type('str'):
            for char in data[:-2]:
                lrc  ^= ord(char)
        else:
            # we are assuming integers
            for char in data[:-2]:
                lrc  ^= char
        
        return lrc
    
    def update_data(self, extractedData):
        # Recheck data length
        if len(extractedData) is 18:
            self.status['ATOMIZER_RPM'] = extractedData[2:8]
            self.status['ATOMIZER_CURRENT'] = extractedData[8:14]
            self.status['FLOW_METER_READING'] = extractedData[14:16]
            
    def set_nozzle_config(self, val):
        # check value is withing 1 byte limit of int
        if (val<256):
            self.nozzleConfiguration = val
    
    def send_nozzle_config(self):
        # check serial port is properly open
        if self.ser:
            if self.ser.isOpen():
                data = [ord('$'), self.functionCode['MSG3'], self.nozzleConfiguration]
                lrc = self.calc_lrc(data)
                packedData = struct.pack('cbbbc', '$', self.functionCode['MSG3'], self.nozzleConfiguration, lrc, '\n')
                self.ser.write(packedData)
        
        
class OnlineHealthMonitor:
    def __init__(self):
        pass        
        
def calc_speed(dataStorageAgri, lock):
    with lock:
        speed = np.sqrt(dataStorageAgri['vx']**2+dataStorageAgri['vy']**2)
    return speed
        