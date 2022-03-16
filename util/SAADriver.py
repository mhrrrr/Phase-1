"""
@author: Dhruv Parikh
@organisation: General Aeronautics Pvt. Ltd.
"""

"""
@Description:
    Sensor driver for lidar
    @Todo: Add UART driver support
           Add CAN driver support
"""

import socket

class SensorDriver():
    def __init__(self,drivertype):
        if drivertype == 'SITL':
            self.HOST, self.PORT = "localhost", 8080


            self.lid = []

        else:
            """UART DRIVER"""
            pass

    def connect_and_fetch(self):
        if self.HOST != None:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.HOST, self.PORT))
            print("Bridge initialised")
    
    def update(self):
        lid = []
        data = None
        while len(lid)<=64:
            data = self.s.recv(4).decode("utf-8") 
            #If we recieve a new packet of data
            if data == 'new ':
                if len(lid) ==64:
                    return lid            
            elif data != None:   
                lid.append(float(data))    

