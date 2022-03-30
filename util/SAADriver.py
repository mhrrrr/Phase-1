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

import socket
from util.RPLidar import RPLidar
import math
import logging

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
            self.PORT = '/dev/ttyUSB2'
            

            

    def connect_and_fetch(self):
        if self.HOST != None:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.HOST, self.PORT))
            logging.info("Bridge initialised")
        else:
            self.lidar = RPLidar(self.PORT)
            info = self.lidar.get_info()

            #@TODO: use this health for monitoring at each time step
            health = self.lidar.get_health()
            logging.info(health)
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.clean_input()

    def update_rplidar(self):
        """
        @Description: 
        Populates the lidar data in the raw_data var
        If lidar data outputs more than 10 measurements per scan, then we will consider the output
        The data is downsampled to resolution of 1 degree to be fed to the SAADataHandler
        """

        lid = []
        ang = []
        mag = [40]*360

        for scan in self.lidar.iter_scans():
            #Reset the vars
            mag = [40]*360

            #Number of measurement in this scan
            no_of_scans = len(scan)
            # print(no_of_scans)
            #If scan is valid
            if no_of_scans>10:
                #Append the data to lidar and magnitude
                for j in range(no_of_scans):
                    #Sometimes the index error occurs
                        # lid.append(float(scan[j][2]))
                        #Downsampling the angular reading
                        #if ang = 1.2, it will be converted to 1
                        # ang.append(int(math.floor(scan[j][1])))
                    ang = (math.floor(scan[j][1]))
                    if ang>359 or ang<0:
                        ang = 0
                    mag[ang] = float(scan[j][2])/1000                    

                # for j in range(len(ang)):
                #     #Assigning the output to the vector based on init angle as 0
                #     #This will give lidar mag @ each int angle in meters
                #     mag[int(ang[j])-1] = lid[j]/1000
                #Assign to the global var once the data is populated
                self.raw_data = mag
            else:
                #If no of scans are less, then assign the default unusable reading
                self.raw_data = mag
        


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

