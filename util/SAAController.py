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
        self.obstacle_map_ = None
        self.obstacle_map = None
        self.brake = 0
        self.pos_vector = [0,0]

        self.engaging_distance = max_obs

        self.mode = "UNKNOWN"

        #Mavlink required to update this stuff
        self.waypoints = np.dot(self.scale(1.01),np.array([[0,0],[0,-188],[150,-188]]).T).T


    def predict_pos_vector(self):
        """
        Predicting the next position of drone. 
        @TODO: This seems to be a very sensitive vector
        Adding an estimator based on position and velocity will be better
        """
        dt = 0.5 # Doesn't matter as long it is positive scalar
        self.pos_vector = [self.vx*dt,self.vy*dt]
    
    
    def scale(self,val):
        return np.eye(2)*val

    def if_inside_triangle(self,point):
        point += np.array([self.px,self.py])
        a1 = self.vec.area_of_triangle(self.waypoints[0,:],self.waypoints[1,:],point)
        a2 = self.vec.area_of_triangle(self.waypoints[2,:],self.waypoints[1,:],point)
        a3 = self.vec.area_of_triangle(self.waypoints[0,:],self.waypoints[2,:],point)
        A = self.vec.area_of_triangle(self.waypoints[0,:],self.waypoints[2,:],self.waypoints[1,:])
#        print(f"{A} vs {a1+a2+a3}")
        return abs(A - (a1+a2+a3))<=5

    def basic_stop(self):
        """
        Basic Stopping Class
        """
        if self.mode != 'AUTO' or self.obstacle_map is None:
            pass
        #Only move forward if obstacle map is defined
        else:
            # print("Won't ignore this obstacle")
            #Protect the var in for loop
            for i in range(np.size(self.obstacle_map,axis=0)):
                #Compute vector
                obstacle_vector = [self.obstacle_map[i,0],self.obstacle_map[i,1]]
                #Scale the triangle using transformation matrix - tomorrow
                if self.if_inside_triangle(obstacle_vector):
                    
                    #If drone is not moving or obstacle is beyond the specified limits -> don't engage 
                    if(self.vec.mag2d(self.pos_vector)==0 or self.vec.mag2d(obstacle_vector)==0 or self.vec.mag2d(obstacle_vector)>=self.engaging_distance):                    
                        obstacle_angle = 1000
                    
                    #Compute the angle between the predicted position and obstacle on the field
                    else:
                        obstacle_angle = round(math.acos(np.dot(self.pos_vector,obstacle_vector)/(self.vec.mag2d(self.pos_vector)*self.vec.mag2d(obstacle_vector)))*180/math.pi,2)
                        # print(obstacle_angle)

                    #If angle is in range, engage the brakes                        
                    #@TODO: Range specified by params
                    if abs(obstacle_angle)<5:
                        self.brake = 1
                        print(f"Brake {obstacle_angle} ---  {self.pos_vector} --- {obstacle_vector}")
                else:
                    pass
                    # print("Ignoring obstacle! Good Luck")

