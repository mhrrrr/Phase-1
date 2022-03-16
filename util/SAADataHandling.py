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
import numpy as np
import VectorMath as vmath


class Sensor():
    #This class handles single instance of reading and has no memory

    def __init__(self,id,delta_angle,max_range,min_range,init_angle):
        self.id = id
        self.delta_angle = delta_angle
        self.max_range = max_range
        self.min_range = min_range
        self.init_angle = init_angle

        self.init_index = init_angle/delta_angle

        self.master_array_length = round(360/delta_angle)


    def handle_raw_data(self,data):
        sensor_binary_mask = [1.1*self.min_range<=abs(i)<=0.9*self.max_range for i in data]
        self.X = np.ones([self.master_array_length])*self.max_range
        self.Y = np.ones([self.master_array_length])*self.max_range

        if any(sensor_binary_mask) == True:
            for i in range(len(data)):
                if sensor_binary_mask[i] == True:
                    #Piecing out magnitude to X-Y coordinates
                    self.X[i + self.init_index] = float(data[i]*math.cos(self.delta_angle*i + self.init_angle))
                    self.Y[i + self.init_index] = float(data[i]*math.sin(self.delta_angle*i + self.init_angle))
        return self.X, self.Y

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



    def update_vehicle_states(self,roll,pitch,yaw,px,py,pz):
        self.pitch = pitch
        self.roll = roll
        self.yaw = math.atan2(math.sin(yaw),math.cos(yaw)) 
        
        self.px = px
        self.py = py
        self.pz = pz

        self.transformations.calc_trig_values(roll,pitch,yaw)
        self.b2i_matrix = np.linalg.inv(self.transformations.euler_zyx())


    def convert_body_to_inertial_frame(self,x,y):
        z = np.zeros([len(x)])

        obstacle_vector_body = np.array([x,y,z]).T
        obstacle_vector_inertial = np.dot(self.b2i_matrix,obstacle_vector_body)

        return obstacle_vector_inertial

class DataPostProcessor():
    """
    Handle the entire obstacle information
    """
    def __init__(self):
        pass

