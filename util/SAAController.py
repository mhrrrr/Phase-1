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

class ObstacleHandle():
    def __init__(self) -> None:
        pass

    def downsampler(self):
        pass
    
    def forget_far_obstacles(self):
        pass

class ObstacleAvoidance(ObstacleHandle):
    def __init__(self):
        self.vec = vmath.vector()
        self.vx = 0
        self.vy = 0
        self.px = 0
        self.py = 0
        self.obstacle_map = None
        self.brake = 0

    def predict_pos_vector(self):
        dt = 0.5
        self.pos_vector = [self.vx*dt,self.vy*dt]

    def basic_stop(self):
        if self.obstacle_map is None:
            pass
        else:
            for i in range(np.size(self.obstacle_map,axis=0)):
                # obstacle_vector = [self.obstacle_map[i,0]-self.px,self.obstacle_map[i,1]-self.py]
                obstacle_vector = [self.obstacle_map[i,0],self.obstacle_map[i,1]]
                
                if(self.vec.mag2d(self.pos_vector)==0 or self.vec.mag2d(obstacle_vector)==0 or self.vec.mag2d(obstacle_vector)>=50):
                    
                    obstacle_angle = 1000
                else:
                    obstacle_angle = round(math.acos(np.dot(self.pos_vector,obstacle_vector)/(self.vec.mag2d(self.pos_vector)*self.vec.mag2d(obstacle_vector)))*180/math.pi,2)
                # if self.vec.mag2d(obstacle_vector)<=50:
                        
                if abs(obstacle_angle)<15:
                    import time
                    print("Stopped")
                    print(time.time())
                    self.brake = 1
                    print(f"Brake {obstacle_angle} ---  {self.pos_vector} --- {obstacle_vector}")

