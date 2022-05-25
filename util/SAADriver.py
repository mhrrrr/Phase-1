"""
@author: Dhruv Parikh
@organisation: General Aeronautics Pvt. Ltd.
@date: 1-03-2022
"""

"""
@Description:
    Sensor driver for lidar/radar
    @Todo: Add UART driver support
           Add CAN driver support
"""


from util.RPLidar import RPLidar
import math
import socket
import math
import logging
import threading
import copy
import serial
import time
import struct

class SensorDriver():
    def __init__(self,drivertype):
        self.HOST = None
        self.raw_data = [40]*360
        self.drivername = drivertype
        
        if drivertype == 'SITL':
            self.HOST, self.PORT = "localhost", 8080
            self.raw_data = [0]*10
            self.lid = []

        else:
            self.PORT = '/dev/ttyUSB0'
            self.START_FLAG = b"\xA5"
            self.HEALTH_CMD = b"\x52"
            self.GET_INFO = b"\x50"
            self.RESET = b"\x40"
            
            self.STOP = b"\x25"
            self.START_SCAN = b"\x20"
            self.HEALTH = 1
            self.INFO = 2
            self.SCAN = 3
            self.RESPONSE_FLAG = b"\x5A"
            
            self.master_angle = []
            self.master_distance = []
            
            self.scan_data = None
            

            

    def connect_and_fetch(self):
        if self.HOST != None:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.HOST, self.PORT))
            logging.info("Bridge initialised")
        else:
            self.lidar_connection = serial.Serial(self.PORT,baudrate=115200,parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,timeout=1)
            # self.send_reset_request()
            time.sleep(2)
            
            self.set_pwm(866)
            self.lidar_connection.read_all()
            time.sleep(1)
            self.start_scan_request()
            self.read_response()
    
    def read_fast(self):
        while True:
            if self.lidar_connection.inWaiting()>300:
                self.scan_data = self.pass_raw_data(4000)

    def give_scan_values(self):
        while True:
            if self.scan_data is not None:
                data = self.scan_data
                for i in range(0,len(data),5):

                    new_scan, angle, distance = self.parse_scan_readings(data[i:i+5])
                    if new_scan:
                        #print("yay")
                        self.master_angle = []
                        self.master_distance = []
                        self.master_angle.append(angle)
                        self.master_distance.append(distance) 
                    elif new_scan == -1:
                        print("problem")
                        pass
                    else:
                        self.master_angle.append(angle)
                        self.master_distance.append(distance)
                    #self.t2 = time.time()
                
    def update_rplidar(self):
        """
        @Description: 
        Populates the lidar data in the raw_data var
        If lidar data outputs more than 10 measurements per scan, then we will consider the output
        The data is downsampled to resolution of 1 degree to be fed to the SAADataHandler
        """
#        self.lidar.reset()
#        

        lid = []
        ang = []
        mag = [40]*360
        #self.give_scan_values()

        
        #Saving the values so they dont get updated
        if len(self.master_angle)>2:
            angles = self.master_angle
            distance = self.master_distance
        else:
            #Wait for some time to populate a few more readings
            time.sleep(0.0001)
            angles = self.master_angle
            distance = self.master_distance
        
        
        #Reset the vars
        mag = [40]*360

        #Number of measurement in this scan
        no_of_scans = len(angles)
        #If scan is valid
        if no_of_scans>2:
            #Append the data to lidar and magnitude
            for j in range(no_of_scans-1):
                #Sometimes the index error occurs
                    # lid.append(float(scan[j][2]))
                    #Downsampling the angular reading
                    #if ang = 1.2, it will be converted to 1
                    # ang.append(int(math.floor(scan[j][1])))
                ang = (math.floor(angles[j]))
                if ang>359 or ang<0:
                    ang = 0
                try:
                    mag[ang] = float(distance[j])/1000
                except:
                    print(len(distance),j)

            # for j in range(len(ang)):
            #     #Assigning the output to the vector based on init angle as 0
            #     #This will give lidar mag @ each int angle in meters
            #     mag[int(ang[j])-1] = lid[j]/1000
            #Assign to the global var once the data is populated
            self.raw_data = mag

        else:
            #If no of scans are less, then assign the default unusable reading
            self.raw_data = mag
        
    def return_readings(self):
        
        return self.raw_data


    def update_sitl_sensor(self):
        #0.1259 seconds highest - sometimes
        #0.00025 seconds lowest 
        #0. 15 seconds 

        #Empty the array for new fetch
        lid = []

        #empty the data
        data = None

        #Keep fetching until the data is filled with 64 indices
        while len(lid)<=64:
            data = self.s.recv(4).decode("utf-8") 
            #If we recieve a new packet of data
            if data == 'new ':
                if len(lid) == 64:
                    self.raw_data = lid
                    lid = []            
            elif data != None:   
                lid.append(float(data))
                
                
    
    #Function Dump
    def send_health_request(self):
        if self.lidar_connection.is_open:
            buf = self.START_FLAG + self.HEALTH_CMD
            self.lidar_connection.write(buf)
            self.request = self.HEALTH

    def send_getinfo_request(self):
        self.send_cmd(self.GET_INFO)
        self.request = self.INFO
    def send_reset_request(self):

        self.send_cmd(self.RESET)
        # _ = self.lidar_connection.read_all()
        # self.lidar_connection.flushInput
        # self.lidar_connection.flushOutput
    
    
    def send_stopscan_request(self):
        self.send_cmd(self.STOP)
    
    def start_scan_request(self):
        self.lidar_connection.setDTR(True)
        self.send_cmd(self.START_SCAN)
        self.request = self.SCAN
        
    def send_cmd(self,cmd):
        if self.lidar_connection.is_open:
            buf = self.START_FLAG + cmd
            self.lidar_connection.write(buf)
        
    def read_response(self):
        if self.lidar_connection.is_open:
            if self.request == self.HEALTH:
                data_len = 3
                data = self.lidar_connection.read(7)
            if self.request ==self.INFO:
                data_len = 20
                data = self.lidar_connection.read(7)
            if self.request == self.SCAN:
                data_len = 0
                data = self.lidar_connection.read(7)
                
            return self.check_data_header(data,data_len)
            
    def check_data_header(self,data,data_len):
        if len(data) != 7:
            print("Descriptor length mismatch")
            return None
        elif not data.startswith(self.START_FLAG + self.RESPONSE_FLAG):
            print("Incorrect descriptor starting bytes")
            return None
        else:
            if data_len!=0:
                data = (self.lidar_connection.read(data_len))
                return data
        
    def interprete_data(self,data):
        if len(data) > 0:
            if self.request == self.HEALTH:
                self.device_health = data[0]
            # if self.request == self.motor
        print(data)

    def _send_payload_cmd(self, cmd: bytes, payload: bytes) -> None:
        size = struct.pack("B", len(payload))
        req = self.START_FLAG + cmd + size + payload
        
        checksum = 0
        for v in struct.unpack("B" * len(req), req):
            checksum ^= v
        req += struct.pack("B", checksum)
        self.lidar_connection.write(req)
        print('Command sent: %s' % self._showhex(req))
        
        
        
    def set_pwm(self, pwm: int) -> None:
        self.lidar_connection.setDTR(False)
        payload = struct.pack("<H", pwm)
        yo = b"\xF0"
        self._send_payload_cmd(yo, payload)
             
    
    def _showhex(self,signal):
        '''Converts string bytes to hex representation (useful for debugging)'''
        return [format((b), '#02x') for b in signal]
    
    
    def get_scan_data(self):
        self.check_data_header()
        self.lidar_connection.read()
        
    def pass_raw_data(self,bytes):
        return self.lidar_connection.read(bytes)
            


    def parse_scan_readings(self,raw:bytes):

        if len(raw)<5:
            print("Length not enough")
            return -1,0,0
        else:
            new_scan = bool(raw[0] & 0b1)
            inversed_new_scan = bool((raw[0] >> 1) & 0b1)
            quality = raw[0] >> 2
            if new_scan == inversed_new_scan:
                print("New scan flags mismatch")
                # self.send_stopscan_request()
                self.lidar_connection.flushInput()
                
                return -1,0,0
            else:
                angle = ((raw[1] >> 1) + (raw[2] << 7)) / 64.0
                distance = (raw[3] + (raw[4] << 8)) / 4.0
                return new_scan,angle,distance


    def _process_scan(self,raw: bytes):
        """Processes input raw data and returns measurement data"""
        if len(raw)==5:
            new_scan = bool(raw[0] & 0b1)
            inversed_new_scan = bool((raw[0] >> 1) & 0b1)
            quality = raw[0] >> 2
            if new_scan == inversed_new_scan:
                print("New scan flags mismatch")
                return 0,0,0,0
            check_bit = raw[1] & 0b1
            if check_bit != 1:
                print("Check bit not equal to 1")
                return 0,0,0,0
            
            angle = ((raw[1] >> 1) + (raw[2] << 7)) / 64.0
            distance = (raw[3] + (raw[4] << 8)) / 4.0
            return new_scan, quality, angle, distance
 
