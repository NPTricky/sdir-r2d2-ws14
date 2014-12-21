import numpy as np
import time
import Kinematics as kin
from sympy.mpmath import *
import math

def PTPtoConfiguration(start_cfg, target_cfg, motiontype):
    """PTP path planning
    
    :param start_cfg: Current axis angle of the robot
    :type start_cfg: array of floats
    :param target_cfg: Target angle of the robot
    :type target_cfg: array of floats
    :param motiontype: Type of motion (asynchronous, synchronous, fully synchronous)
    :type motiontype: int
    :returns: Array containing the axis angles of the interpolated path
    :rtype: matrix of floats
    """
    
    trajectory = np.empty([100, 6])

    #TODO: Implement PTP (Replace pseudo implementation with your own code)! Consider the max. velocity and acceleration of each axis
    
    diff = target_cfg - start_cfg
    delta = diff / 100.0  
    
    for i in xrange(100):
        trajectory[i] = start_cfg + (i*delta)
        
    trajectory[99] = target_cfg
    
    return trajectory   
    
def Move(robot, trajectory):
    for i in range(trajectory.shape[0]):
        robot.SetDOFValues(trajectory[i])
        kin.forward(robot)
        time.sleep(0.01)

    
"""
@type pose: position and rotation of a point
@param pose: point in robot coordinate system were move the robot
@rtype: configurations of angles, angle in radiant
@return: configuration for each joint of the robot
"""
def get_fastest_invese_solution(robot, configurations):

    limit_angles = robot.GetDOFLimits()
    current_angles = robot.GetDOFValues()
    velocity_limits = robot.GetDOFVelocityLimits()
       
    # calculate the movement time for each configuration and give back
    # the fastet configuration
    # the function checked, if the angle for this robot possible
    t_min = 999999
    ret_value = []
    for config in configurations:
    
        t_conf = 0
        possible = True
        for i in range(0, len(config)):
            if config[i] < limit_angles[0][i] or limit_angles[1][i] < config[i]:
                possible = False
                break
            
            t_conf += np.abs(config[i] - current_angles[i]) / velocity_limits[i]
        
        if possible and t_conf < t_min:
            t_min = t_conf
            ret_value = config
        
    return ret_value
    
def get_possible_inverse_solution(robot, configurations):
    
    angle_limits = robot.GetDOFLimits()
    
    ret_value = []
    for config in configurations:
    
        check = True
        for i in range(0, len(config)):
            if config[i] < angle_limits[0][i] or angle_limits[1][i] < config[i]:
                check = False
                break
    
        if check:
            ret_value.append(config)
    
    return ret_value
    
def velocity(robot, pose):
    
    
    return