"""
@author: Dhruv Parikh
@organisation: General Aeronautics Pvt. Ltd.
"""

"""
@Description:
    Sense and Avoid Algorithm
    @Todo: I don't know
"""

import numpy as np
import math
import util.VectorMath as vmath
import logging

class ObstacleHandle():
    def __init__(self) -> None:
        pass

    def downsampler(self):
        pass
    
    def forget_far_obstacles(self):
        pass
    
    

class ObstacleAvoidance(ObstacleHandle):
    def __init__(self,max_obs = 10):
        self.vec = vmath.vector()
        self.vx = 0
        self.vy = 0
        self.px = 0
        self.py = 0
        self.obstacle_map_2 = None
        self.obstacle_map = None
        self.brake = 0
        self.pos_vector = [0,0]

        self.engaging_distance = max_obs

        self.mode = "UNKNOWN"


    def predict_pos_vector(self):
        """
        Predicting the next position of drone. 
        @TODO: This seems to be a very sensitive vector
        Adding an estimator based on position and velocity will be better
        """
        dt = 0.5 # Doesn't matter as long it is positive scalar
        self.pos_vector = [self.vx*dt,self.vy*dt]

    def basic_stop(self):
        """
        Basic Stopping Class
        """
        if self.mode != 'AUTO' or self.obstacle_map is None:
            pass
        #Only move forward if obstacle map is defined
        else:
            for i in range(np.size(self.obstacle_map,axis=0)):
                #Compute vector
                obstacle_vector = [self.obstacle_map[i,0],self.obstacle_map[i,1]]
                
                #If drone is not moving or obstacle is beyond the specified limits -> don't engage 
                if(self.vec.mag2d(self.pos_vector)==0 or self.vec.mag2d(obstacle_vector)==0 or self.vec.mag2d(obstacle_vector)>=self.engaging_distance):                    
                    obstacle_angle = 1000
                
                #Compute the angle between the predicted position and obstacle on the field
                else:
                    obstacle_angle = round(math.acos(np.dot(self.pos_vector,obstacle_vector)/(self.vec.mag2d(self.pos_vector)*self.vec.mag2d(obstacle_vector)))*180/math.pi,2)

                #If angle is in range, engage the brakes                        
                #@TODO: Range specified by params
                if abs(obstacle_angle)<5:
                    self.brake = 1
                    logging.info(f"Brake {obstacle_angle} ---  {self.pos_vector} --- {obstacle_vector}")

