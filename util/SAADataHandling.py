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
import util.VectorMath as vmath

import time
class Sensor():
    #This class handles single instance of reading and has no memory

    def __init__(self,id,delta_angle,max_range,min_range,init_angle):
        """
        @Params: id: sensor id for future
        delta_angle: for the steps in lidar data (dtheta)
        max_range: maximum range of sensor
        min_range: minimum range of sensor
        init_angle: Angle from which the sensor will start recording (offset)
        """

        #Assign to class params
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
        #If data is too less
        if len(data)<=10:
            print("Data not populated in other thread!!")
        else:

            #Create a bit mask for taking trusty readings of 90% 
            #@TODO: This should be based on CC code params because we can't afford to loose data in low range sensors
            sensor_binary_mask = [1.1*self.min_range<=abs(i)<=0.95*self.max_range for i in data]

            #Recreating the X and Y vectors 
            self.X = np.ones([self.master_array_length])*self.max_range
            self.Y = np.ones([self.master_array_length])*self.max_range

            #If any reading falls in range
            if any(sensor_binary_mask) == True:
                for i in range(len(data)):
                    #Populate only those readings where the range is true else don't bother calculating
                    if sensor_binary_mask[i] == True:
                        #Piecing out magnitude to X-Y coordinates
                        self.X[i + self.init_index] = float(data[i]*math.cos(self.delta_angle*i + self.init_angle))
                        self.Y[i + self.init_index] = float(data[i]*math.sin(self.delta_angle*i + self.init_angle))

                        print("Obstacle detected")
                        print(time.time())

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

        #x and y are the mags
        self.x = [40]
        self.y = [40]

        #Vehicle states
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.px = 0
        self.py = 0
        self.pz = 0
        
        #Initialise the body to inertial matrix as 3x3 identity
        self.b2i_matrix = np.eye(3)


    def update_vehicle_states(self):
        #0.00023102760314941406 seconds

        #Calculate trigs and matrices before hand to reduce computational load
        self.transformations.calc_trig_values(self.roll,self.pitch,self.yaw)
        self.b2i_matrix = np.linalg.inv(self.transformations.euler_zyx())



    def convert_body_to_inertial_frame(self):
        #5.7697296142578125e-05 seconds

        #These x and y are updated in the mainloop
        x = self.x
        y = self.y
        z = np.zeros([len(x)])
        
        #Obstacle vector in the body frame
        obstacle_vector_body = np.array([x,y,z])

        #Convert from body to inertial frame
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
