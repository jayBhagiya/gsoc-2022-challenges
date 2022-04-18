
"""
Robotics Academy Exercise

    Name: Vacuum Cleaner
    Solution: Solved using "Random Coverage Algorithm"
"""

from GUI import GUI
from HAL import HAL

import math
import time
import random

def getBumperState():
    """
    For getting current state for bumper data

    Parameters:
        None
    Returns:
        Bumper status (1 of if collided else 0)
    """
    return HAL.getBumperData().state
    
def getBumberBump():
    """
    For getting data of collision to particular side

    Parameters:
        None
    Returns:
        Bumper status (0: if left side, 1: if front side, 2: if right side)
    """
    return HAL.getBumperData().bumper

curr_state = 1 # In free region (robot will move in spiral)
bumper_bump = -1 # not collided with any wall

vel = 0
ang_vel = 2

while True:
    # while there is not collision move in spiral
    if(curr_state == 1 and getBumperState() == 0):
        HAL.setV(vel)
        HAL.setW(ang_vel)
        vel += 0.0125
    
    # If robot collides 
    elif(getBumperState() == 1):
        # Change the free region to collided (robot will rotate to random angle)
        curr_state = 2
        
        # setting linear vel to 0
        # and robot will move for random duration at fixed angular vel
        vel = 0
        ang_vel = 3
        
        # setting liner vel 
        HAL.setV(vel)
        
        # If the collision is with left wall ang_vel will be positive else negative
        if(getBumberBump() == 0):
            HAL.setW(ang_vel)
        else:
            HAL.setW(-1 * ang_vel)
        
        # Waiting for random duarion to rotate the robot at random angle 
        time.sleep(random.random() * math.pi)
        
        # All velocity set to zero
        HAL.setV(0)
        HAL.setW(0)
        time.sleep(0.8)
        
        # state changed to 3 (robot will move straight)
        curr_state = 3
    
    # if robot has rotated to free region then state 3 (robot will move forward for random duration)
    elif(curr_state == 3):
        # Just setting linear vel
        vel = 3
        ang_vel = 0
        HAL.setV(vel)
        HAL.setW(ang_vel)
        
        # wait for random duration (meanwhile robot will move forward)
        time.sleep(random.random() * 4)
        
        # changing the state to 1 (again robot will move in spiral)
        curr_state = 1
        vel = 0
        ang_vel = 2