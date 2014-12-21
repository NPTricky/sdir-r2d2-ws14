import numpy as np
import time
import Kinematics as kin
from sympy.mpmath import *
import math

def PTPtoConfiguration(robot, target_cfg, motiontype):
    """PTP path planning
    :param robot: robot instance
    :type robot: model of the robot
    :param target_cfg: Target angle of the robot
    :type target_cfg: array of floats
    :param motiontype: Type of motion (asynchronous, synchronous, fully synchronous)
    :type motiontype: int
    :returns: Array containing the axis angles of the interpolated path
    :rtype: matrix of floats
    """
    # current axis angles of the robot - array of floats
    start_cfg = robot.GetDOFValues();
        
    trajectory = np.empty([100, 6])

    #TODO: Implement PTP (Replace pseudo implementation with your own code)! Consider the max. velocity and acceleration of each axis
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times(robot, start_cfg, target_cfg, motiontype)
    
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
def get_fastest_inverse_solution(robot, configurations):

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

def limits_and_times(robot, start_cfg, target_cfg, motiontype):
    # given
    distance = np.fabs(target_cfg - start_cfg)
    
    # calculate
    velocity_limit = np.zeros(robot.GetDOF())
    acceleration_limit = np.zeros(robot.GetDOF())
    times_acc = np.zeros(robot.GetDOF()) # acceleration stop time
    times_dec = np.zeros(robot.GetDOF()) # deceleration start time
    times_end = np.zeros(robot.GetDOF()) # total motion time
    
    if motiontype == 0: # asynchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    elif motiontype == 1: # synchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_synchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)    
    elif motiontype == 2: # full synchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_full_synchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    else: # default to asynchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
                    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end



def limits_and_times_asynchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    for i in range(0, robot.GetDOF()):
        if (distance[i] < 1e-12): continue
        
        # maximum amplitude of velocity ramp (triangle situation)
        v_limit = mp.sqrt(distance[i]) * robot.GetDOFAccelerationLimits()[i]
        velocity_limit[i] = min(v_limit,robot.GetDOFVelocityLimits()[i])
        
        # maximum acceleration
        acceleration_limit[i] = robot.GetDOFAccelerationLimits()[i]
        
        # points in time
        times_acc[i] = velocity_limit[i] / robot.GetDOFAccelerationLimits()[i]
        times_end[i] = (distance[i] / velocity_limit[i]) / times_acc[i]
        times_dec[i] = times_end[i] - times_acc[i]
        
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end



def limits_and_times_synchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    
    # latest total motion time
    temax_idx = np.where(times_end == times_end.max())
    temax = times_end[temax_idx][0]
    
    for i in range(0, robot.GetDOF()):
        if (distance[i] < 1e-12): continue
        
        velocity_limit[i] = (temax * robot.GetDOFAccelerationLimits()[i] / 2) - mp.sqrt((math.pow(temax, 2) * math.pow(robot.GetDOFAccelerationLimits()[i], 2) / 4) - robot.GetDOFAccelerationLimits()[i] * distance[i])
        
        # points in time
        times_acc[i] = velocity_limit[i] / robot.GetDOFAccelerationLimits()[i]
        times_end[i] = temax
        times_dec[i] = times_end[i] - times_acc[i]
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end
   
   

def limits_and_times_full_synchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    
    # latest total motion time
    temax_idx = np.where(times_end == times_end.max())
    temax = times_end[temax_idx][0]
    
    # latest deceleration start time
    tdmax_idx = np.where(times_dec == times_dec.max())
    tdmax = times_dec[tdmax_idx][0]
    
    # latest acceleration stop time
    tamax_idx = np.where(times_acc == times_acc.max())
    tamax = times_acc[tamax_idx][0]
    
    for i in range(0, robot.GetDOF()):
        if (distance[i] < 1e-12): continue
        
        velocity_limit[i] = distance[i] / tdmax
        acceleration_limit[i] = velocity_limit[i] / tamax
    
        # points in time
        times_acc[i] = velocity_limit[i] / acceleration_limit[i]
        times_end[i] = temax
        times_dec[i] = times_end[i] - times_acc[i]
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end
