# -*- coding: utf-8 -*-
"""
Created on Fri Mar  8 12:22:57 2019

@author: sachchit
"""

import numpy as np
import time
import struct
import serial
from gacommonutil import dataStorageCommon
import copy
if not dataStorageCommon['isSITL']:
    import pigpio

class AgriPayload:
    def __init__(self):        
        # PIB Input
        self.pumpPWM = 0
        self.micromiserPWM = 0
        
        # Status
        self.remainingPayload = 15                   # in liters
        self.reqFlowRate = 0.                           # in ltr/min
        self.shouldSpraying = False
        
        # pump parameters
        self.pumpMaxPWM = 1999
        self.pumpMinPWM = 1100
        self.pumpAbsMinPWM = 1000
        self.pumpMaxFlowRate = 1.4                  # in ltr/min
        self.pumpMinFlowRate = 0                  # in ltr/min
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
        self.swath = 4.0                    # in m
        self.maxFlowRate = 1.2              # in ltr/min
        self.maxSpeedSentCount = 0
        self.sprayDensity = self.perAcrePestiscide/4047.
        self.maxSpeed = self.maxFlowRate/60./self.swath/self.sprayDensity + 0.15

        # GPIO handling
        if not dataStorageCommon['isSITL']:
            self.pi = pigpio.pi()
        self.pumpPin = 18
        self.nozzPin = 19

        # PID Pump
        self.pumpP = 50.
        self.pumpI = 5.
        self.pumpIMax = 1.
        self.pumpD = 10.
        self.pumpOldError = 0.
        self.pumpIntError = 0.
        
        # Time related
        self.time = time.time()

        # Debug Timer
        self.debugTime = time.time()
        
    def calc_remaining_payload(self, actualFlowRate):
        if self.remainingPayload > 0:
            self.remainingPayload = self.remainingPayload - self.dt * (actualFlowRate/60.)
            
    def update_time(self):
        currTime = time.time() 
        self.dt = currTime - self.time
        self.time = currTime
        
    def update(self, dataStorageAgri, mavConnection, mavutil, msgList, lock):        
        # update timer
        self.update_time()
        
        # calculate speed of the vehicle
        speed = calc_speed(dataStorageAgri, lock)
        
        # RPM
        actualRPM = dataStorageAgri['actualNozzRPM']
        
        # actual flow rate
        actualFlowRate = dataStorageAgri['actualFlowRate']
        
        # Are we testing
        testing = dataStorageAgri['testing']
        #print testing
        # check whether we should be spraying
        print "WP", dataStorageAgri['startWP'], dataStorageAgri['currentWP'], dataStorageAgri['endWP'], mavConnection.flightmode
        if dataStorageAgri['currentWP'] <= dataStorageAgri['endWP'] and dataStorageAgri['currentWP'] > dataStorageAgri['startWP'] and dataStorageAgri['endWP']>1 and mavConnection.flightmode == 'AUTO':
            # Allow sending max speed again to vehicle if the shouldSpraying state have changed
            if not self.shouldSpraying:
                self.maxSpeedSentCount = 0
                
            self.shouldSpraying = True
            # Set correct top speed of vehicle
            self.set_vehicle_max_speed(mavConnection, mavutil, msgList, lock)
        else:
            # Allow sending max speed again to vehicle if the shouldSpraying state have changed
            if self.shouldSpraying:
                self.maxSpeedSentCount = 0
                
            self.shouldSpraying = False
            # send the message 3 times to be sure message reaches autopilot
            if (self.maxSpeedSentCount < 3):
                # create message
                msg = mavutil.mavlink.MAVLink_command_long_message(mavConnection.target_system, 
                                                                   mavConnection.target_component,
                                                                   mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # command
                                                                   0, # confirmation
                                                                   0, # param1 
                                                                   9, # param2 
                                                                   0, # param3
                                                                   0, # param4
                                                                   0, # param5
                                                                   0, # param6
                                                                   0) # param7
                
                with lock:
                    # append message to send list
                    msgList.append(msg)
                
                # update counter
                self.maxSpeedSentCount = self.maxSpeedSentCount + 1
        
        if self.shouldSpraying:
            # calculate flow rate
            self.calc_flow_rate(speed)
            
            # calculate pwm for pump and nozzle
            self.calc_pump_pwm(actualFlowRate)
            self.calc_nozz_pwm(actualRPM)
            
        else:
            if testing:
                self.nozzPWM = 1999
                #self.pumpPWM = 1400
##                currTime = time.time()
##                deltaTime = currTime - self.debugTime
##                if deltaTime < 50:
##                    self.reqFlowRate = min(1.2, 0.6+ 0.2*deltaTime)
##                elif deltaTime < 30:
##                    self.reqFlowRate = 0.4
##                elif deltaTime < 50:
##                    self.reqFlowRate = 1.2
##                elif deltaTime < 60:
##                    self.reqFlowRate = 0.6
##                else:
##                    self.debugTime = currTime
                self.reqFlowRate = 1.2
                    
                self.calc_pump_pwm(actualFlowRate)

                #self.pumpPWM = 1400
            else:
                self.nozzPWM = self.nozzMinPWM
                self.pumpPWM = self.pumpAbsMinPWM
                self.reqFlowRate = 0
                
        # update the pwm in DCU
        self.update_pwm(mavConnection, mavutil, msgList, lock)
##        actualFlowRate = self.reqFlowRate
        # update the remaining payload
        print "FlowRate", ",",dataStorageAgri['actualFlowRate'], ",", self.reqFlowRate, ",", self.nozzPWM, ",", self.pumpPWM 
        print "RemPayload", ",", dataStorageAgri['remainingPayload']
        self.remainingPayload = dataStorageAgri['remainingPayload']
        self.calc_remaining_payload(actualFlowRate)
        with lock:
            dataStorageAgri['remainingPayload'] = self.remainingPayload
        
    def calc_flow_rate(self, speed):
        # sanity check of speed
        if (speed < 0):
            speed = 0
        if (speed > (self.maxSpeed+1)):
            speed = self.maxSpeed+1
        
        # Calculate flow rate
        self.reqFlowRate = 60*self.swath*self.sprayDensity*speed

        
        # check flow rate limit
        if self.reqFlowRate > self.maxFlowRate:
            self.reqFlowRate = self.maxFlowRate
        if self.reqFlowRate < self.pumpMinFlowRate:
            self.reqFlowRate = self.pumpMinFlowRate
            
    def calc_pump_pwm(self, actualFlowRate):
##        if abs(self.reqFlowRate - actualFlowRate) > 0.25:
##            pumpPWM = int(self.pumpPWM + 100 * (self.reqFlowRate - actualFlowRate))
##        elif abs(self.reqFlowRate - actualFlowRate) > 0.1:
##            pumpPWM = int(self.pumpPWM + 50 * (self.reqFlowRate - actualFlowRate))
##        elif abs(self.reqFlowRate - actualFlowRate) > 0.01:
##            pumpPWM = int(self.pumpPWM + 50 * (self.reqFlowRate - actualFlowRate))
##        else:
##            pumpPWM = self.pumpPWM
#        pumpPWM = self.pumpMinPWM + int(float(self.pumpMaxPWM - self.pumpMinPWM)*float(self.reqFlowRate - self.pumpMinFlowRate)/float(self.pumpMaxFlowRate - self.pumpMinFlowRate))
        # P
        error = (self.reqFlowRate - actualFlowRate)
        pumpPWM = int(self.pumpPWM + self.pumpP * error)

        # D
        dError = (error - self.pumpOldError) / self.dt
        self.pumpOldError = error
        pumpPWM = int(pumpPWM + self.pumpD * dError)

        # I
        self.pumpIntError = self.pumpIntError + error
        if (self.pumpI * self.pumpIntError) > self.pumpIMax:
            self.pumpIntError = self.pumpIMax/self.pumpI

        if (self.pumpI * self.pumpIntError) < -self.pumpIMax:
            self.pumpIntError = -self.pumpIMax/self.pumpI
        pumpPWM = int(pumpPWM + self.pumpI * self.pumpIntError)

        print "PID", ",", error, ",", self.pumpIntError , ",", self.pumpP * error, ",", self.pumpI * self.pumpIntError, ",", self.pumpD * dError
        
        
        self.pumpPWM = pumpPWM
        #print self.pumpPWM, self.reqFlowRate, actualFlowRate
##        with open('flow_log1', 'a') as f:
##            f.write('%f, %f, %f\n'%(self.pumpPWM, self.reqFlowRate, actualFlowRate))

        if self.pumpPWM > self.pumpMaxPWM:
            self.pumpPWM = self.pumpMaxPWM
        if self.pumpPWM < self.pumpMinPWM:
            self.pumpPWM = self.pumpMinPWM
        
    def calc_nozz_pwm(self, actualRPM):
        
        if self.nozzType == 'Micromiser':
            self.nozzPWM = self.nozzMaxPWM - 500*float(self.nozzMaxFlowRate - self.reqFlowRate/4)/float(self.nozzMaxFlowRate - self.nozzMinFlowRate)
##            # handle garbage value
##            if actualRPM < 0:
##                actualRPM = 0
##            if actualRPM > 20000:
##                actualRPM = 20000
##            
##            # Calculate Target RPM
##            nozzFlow = self.reqFlowRate/self.nozzNum
##            if nozzFlow < self.nozzMinFlowRate:
##                nozzFlow = self.nozzMinFlowRate
##            if nozzFlow > self.nozzMaxFlowRate:
##                nozzFlow = self.nozzMaxFlowRate
##            
##            # curve fit equation RPM = A * (PS - C) ^ B from Micromiser 10 Datasheet
##            equationCoeefsA = 89248 + (nozzFlow - 0.3) / (0.1 - 0.3) * (180412 - 89248)
##            equationCoeefsB = -0.732 + (nozzFlow - 0.3) / (0.1 - 0.3) * (-0.858 + 0.732)
##            equationCoeefsC = 80 + (nozzFlow - 0.3) / (0.1 - 0.3) * (42 - 80)
##            
##            if (self.nozzMinPS - equationCoeefsC) > 1:
##                nozzTargetRPM = equationCoeefsA * (self.nozzMinPS - equationCoeefsC)**equationCoeefsB
##            else:
##                nozzTargetRPM = 0
##                
##            # P control for maintaing target PWM
##            self.nozzPWM = int(self.nozzPWM + self.nozzP * (nozzTargetRPM - actualRPM))
##            
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
            msg = mavutil.mavlink.MAVLink_command_long_message(mavConnection.target_system, 
                                                                   mavConnection.target_component,
                                                                   mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # command
                                                                   0, # confirmation
                                                                   0, # param1 
                                                                   self.maxSpeed, # param2 
                                                                   0, # param3
                                                                   0, # param4
                                                                   0, # param5
                                                                   0, # param6
                                                                   0) # param7
            
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

        if not dataStorageCommon['isSITL']:
            self.pi.hardware_PWM(self.pumpPin, 50, 50*self.pumpPWM)
            self.pi.hardware_PWM(self.nozzPin, 50, 50*self.nozzPWM)
                
##        # create message
##        pumpPWMMsg = mavutil.mavlink.MAVLink_command_long_message(mavConnection.target_system,
##                                                                  mavConnection.target_component,
##                                                                  mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
##                                                                  0,
##                                                                  9,
##                                                                  self.pumpPWM,
##                                                                  0,
##                                                                  0,
##                                                                  0,
##                                                                  0,
##                                                                  0)
##        nozzPWMMsg = mavutil.mavlink.MAVLink_command_long_message(mavConnection.target_system,
##                                                                  mavConnection.target_component,
##                                                                  mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
##                                                                  0,
##                                                                  10,
##                                                                  self.nozzPWM,
##                                                                  0,
##                                                                  0,
##                                                                  0,
##                                                                  0,
##                                                                  0)
##        
##        with lock:
##            # append message to send list
##            msgList.append(pumpPWMMsg)
##            msgList.append(nozzPWMMsg)
        
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
        self.nozzleConfiguration = 0b00000000
        
        # Initialize
        self.init()
        
    def init(self):
        if not dataStorageCommon['isSITL']:
            while True:
                # keep trying  to open port unitl succesful
                try:
                    time.sleep(1)
                    self.ser = serial.Serial(port=self.serial,
                                             baudrate=115200,
                                             parity=serial.PARITY_ODD,
                                             stopbits=serial.STOPBITS_ONE,
                                             bytesize=serial.EIGHTBITS,
                                             timeout=1)
                    break
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
            self.send_nozzle_config()
                
    def update(self, dataStorageAgri, lock):
        if not dataStorageCommon['isSITL']:
            # read the data
            data = self.ser.readline()
            self.ser.reset_input_buffer()


            # update the status
            self.decode_data(data)
            
            # transfer data to dataStorageAgri
            self.update_veh_data(dataStorageAgri, lock)
    
            # Send the nozzle configuration
            #if dataStorageAgri['NozzleConfig'] != self.nozzleConfiguration:
            self.set_nozzle_config(dataStorageAgri['NozzleConfig'])
            self.send_nozzle_config()
            
    def update_veh_data(self, dataStorageAgri, lock):
        with lock:
            #print ((self.status['FLOW_METER_READING'][1] - 9.3438)/84.413)
            dataStorageAgri['actualFlowRate'] = ((self.status['FLOW_METER_READING'][1] - 9.3438)/84.413) * 2./5. - 0.07
            if dataStorageAgri['actualFlowRate'] < 0:
                dataStorageAgri['actualFlowRate'] = 0

    def decode_data(self, data):
        
        # update the timer
        self.timer = time.time()
        
        # if data is not recieved, store how long it has been without data
        if not data:
            if self.dataNotRecievedStartTime == 0:
                self.dataNotRecievedStartTime = int(self.timer)
            self.dataNotRecievedTime = int(self.timer - self.dataNotRecievedStartTime)
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
                if (lrc == extractedData[-2]):
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
        acceptedLengths = [38,3]
        dataLength = len(data)
        
        # Reject data with
        # first character not being $
        # length of data not matching to any of the packet size expected by us
        # second character is out of bound of acceptable function codes
        if (data[0] == '$') and (dataLength in acceptedLengths) and (int(data[1])<8):
            self.errorNow['WRONG_DATA'] = False
            
            if dataLength == 3:
                extractedData = struct.unpack('cbc', data)
                return extractedData
            else:
                #extractedData = struct.unpack('cchhhhhhhhhhhhhhhhhBc', data)
                extractedData = struct.unpack('ccHHHHHHHHHHHHHHHHHBc', data)
                return extractedData
                
        else:
            self.errorNow['WRONG_DATA'] = True
            return None
        
    def calc_lrc(self, data):
        lrc = 0
        if type(data) == type('str'):
            for char in data[:-2]:
                lrc  ^= ord(char)
        else:
            # we are assuming integers
            for char in data:
                lrc  ^= char
        
        return lrc
    
    def update_data(self, extractedData):
        # Recheck data length
        if len(extractedData) is 21:
            self.status['ATOMIZER_RPM'] = extractedData[2:8]
            self.status['ATOMIZER_CURRENT'] = np.asarray(extractedData[8:14])/100.
            self.status['PUMP_CURRENT'] = np.asarray(extractedData[14:16])/100.
            self.status['FLOW_METER_READING'] = extractedData[16:18]
            print "temp,",extractedData[18]
            print "rpm,",self.status['ATOMIZER_RPM']
            print "acurr,",self.status['ATOMIZER_CURRENT']
            print "pcurr,",self.status['PUMP_CURRENT']
            #print "fmreading", self.status['FLOW_METER_READING'][1]
            
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

class FlowSensor:
    def __init__(self, pin=2):
        self.pin = pin
        self.count = 0
        self.countList60 = [0]*300
        self.countList = [0]*10
        self.fileName = str(time.time())
        if not dataStorageCommon['isSITL']:
            self.pi = pigpio.pi()
            self.pi.set_mode(self.pin, pigpio.INPUT)
            self.pi.callback(self.pin, pigpio.RISING_EDGE, self.counter)
    
    def counter(self,g,l,t):
        self.count = self.count + 1
        
    def calc_flow_rate(self, dataStorageAgri, lock):
        count = self.count
        self.count = 0
        actualFlowrate = 0
        if count > 6:
            self.countList60[:-1] = self.countList60[1:]
            self.countList60[-1] = count
            self.countList[:-1] = self.countList[1:]
            self.countList[-1] = count
            #print sum(self.countList60), sum(self.countList), self.countList
            
##            with open(self.fileName, 'a') as f:
##                f.write('%d'%sum(self.countList60))
##                f.write('\n')
            a5Hz = 55.448
            a1Hz = a5Hz/5
            a05Hz = a5Hz/10
            b = -68.483
            flowRate5Hz = a5Hz*self.countList[-1] + b
            
            flowRate1Hz = a1Hz*(sum(self.countList[5:])) + b
            flowRate05Hz = a05Hz*(sum(self.countList[:])) + b
            
            sortedCountList1Hz = copy.deepcopy(self.countList[5:])
            sortedCountList05Hz = copy.deepcopy(self.countList[:])
            #print sum(sortedCountList05Hz)
            sortedCountList1Hz.sort()
            sortedCountList05Hz.sort()
            #print self.countList
            actualFlowrate = flowRate5Hz
            if abs(sortedCountList1Hz[0]-sortedCountList1Hz[-1]) < 3:
                actualFlowrate = flowRate1Hz
            if abs(sortedCountList05Hz[0]-sortedCountList05Hz[-1]) < 3:
                actualFlowrate = flowRate05Hz
            #print "flowrate", flowRate5Hz/1000., flowRate1Hz/1000., flowRate05Hz/1000., self.countList
        with lock:
            dataStorageAgri['actualFlowRate'] = 1.04*actualFlowrate/1000.
                
        
class OnlineHealthMonitor:
    def __init__(self):
        pass        
        
def calc_speed(dataStorageAgri, lock):
    with lock:
        speed = np.sqrt(dataStorageAgri['vx']**2+dataStorageAgri['vy']**2)
    return speed
        
