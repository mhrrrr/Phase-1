"""
@author: Dhruv Parikh
@organisation: General Aeronautics Pvt. Ltd.
"""

"""
@Description:
    Data handling for the obstacle avoidance sensor data. 

    @Todo: Handle multiple sensor 
    Pack all the data in one stream
    Handle the estimation errors and filtering portion 
    Account for offsets (Least priority)
"""

import math
from turtle import update
import numpy as np
import util.VectorMath as vmath


class Sensor():
    #This class handles single instance of reading and has no memory

    def __init__(self,id,delta_angle,max_range,min_range,init_angle):

        init_angle += math.pi/2

        self.id = id
        self.delta_angle = delta_angle
        self.max_range = max_range
        self.min_range = min_range
        self.init_angle = init_angle

        self.init_index = round(init_angle/delta_angle)
        print(self.init_index)
        print(self.init_angle*180/3.14)

        self.master_array_length = round(2*math.pi/delta_angle)

        print(self.master_array_length)

        self.X = np.ones([self.master_array_length])*self.max_range
        self.Y = np.ones([self.master_array_length])*self.max_range

        self.data = [0]



    def handle_raw_data(self):
        #5.888938903808594e-05 seconds
        data = self.data
        if len(data)<=10:
            pass
        else:
            sensor_binary_mask = [1.1*self.min_range<=abs(i)<=0.9*self.max_range for i in data]
            self.X = np.ones([self.master_array_length])*self.max_range
            self.Y = np.ones([self.master_array_length])*self.max_range

            if any(sensor_binary_mask) == True:
                for i in range(len(data)):
                    if sensor_binary_mask[i] == True:
                        #Piecing out magnitude to X-Y coordinates
                        self.X[i + self.init_index] = float(data[i]*math.cos(self.delta_angle*i + self.init_angle))
                        self.Y[i + self.init_index] = float(data[i]*math.sin(self.delta_angle*i + self.init_angle))
        #return self.X, self.Y

    def combine_multiple_readings(self,x2,y2):
        for i in range(self.master_array_length):
            if x2[i] <40 or y2[i] <40:
                self.X[i] = x2[i]
                self.Y[i] = y2[i]

        return self.X, self.Y

class DataPreProcessor():
    """
    Handle obstacle vector - 1 shot scan
    """
    def __init__(self):
        self.transformations = vmath.AngularTransformation()
        self.x = [40]
        self.y = [40]
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.px = 0
        self.py = 0
        self.pz = 0



    def update_vehicle_states(self):
        #0.00023102760314941406 seconds
        self.transformations.calc_trig_values(self.roll,self.pitch,self.yaw)
        self.b2i_matrix = np.linalg.inv(self.transformations.euler_zyx())



    def convert_body_to_inertial_frame(self):
        #5.7697296142578125e-05 seconds

        x = self.x
        y = self.y
        z = np.zeros([len(x)])

        obstacle_vector_body = np.array([x,y,z])
        self.obstacle_vector_inertial = np.dot(self.b2i_matrix,obstacle_vector_body)

class DataPostProcessor():
    """
    Handle the entire obstacle information
    """
    def __init__(self):
        self.map_x = [0]
        self.map_y = [0]
        self.px = 0
        self.py = 0


    def cleanup(self):
        k = 0
        dist = [(self.px-b[i][0])**2 + (self.py-b[i][1])**2 for i in range(np.size(b,axis=0))]
        for i in range(len(dist)):
            j = dist[i]
            if j>1600:
                b = np.delete(b, i-k,0)
                k = k+1
        return b
