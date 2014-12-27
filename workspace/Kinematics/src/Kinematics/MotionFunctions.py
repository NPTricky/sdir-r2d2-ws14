import numpy as np
import time
import Kinematics as kin
import math
import sys

# e.g. 0.1, 0.01 or 0.001
_SAMPLE_RATE = 0.01
# length of the string minus 2, to subtract the two letters "0."
_SAMPLE_RATE_DECIMAL = len(str(_SAMPLE_RATE)) - 2
# requirement to simulate a np.ceil() by np.round()
_SAMPLE_RATE_CEIL_OFFSET = _SAMPLE_RATE / 2

_EPS = 1e-12
 
def PTPtoConfiguration(robot, target_cfg, motiontype):
    """PTP path planning
    :param robot: robot instance
    :type robot: model of the robot
    :param target_cfg: Target angle of the robot
    :type target_cfg: array of floats
    :param motiontype: Type of motion (asynchronous, synchronous, fully synchronous, linear movement)
    :type motiontype: int
    :returns: Array containing the axis angles of the interpolated path
    :rtype: matrix of floats
    """
    # current axis angles of the robot - array of floats
    start_cfg = robot.GetDOFValues();
    distance = np.fabs(target_cfg - start_cfg)
    distance = distance.clip(min = _EPS)
    
    #TODO: Implement PTP (Replace pseudo implementation with your own code)! Consider the max. velocity and acceleration of each axis
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times(robot, distance, motiontype)
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = discretize(velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
        
    trajectory = generate_trajectory(start_cfg, target_cfg, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    
    return trajectory   



def Move(robot, trajectory):
    for i in range(trajectory.shape[0]):
        robot.SetDOFValues(trajectory[i])
        kin.forward(robot)
        time.sleep(_SAMPLE_RATE)



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
    # the fastest configuration
    # the function checked, if the angle for this robot possible
    t_min = sys.maxsize
    ret_value = []
    for config in configurations:
    
        t_conf = 0
        possible = True
        for i in range(0, len(config)):
            if config[i] < limit_angles[0][i] or limit_angles[1][i] < config[i]:
                possible = False
                break
            
            t_conf += np.fabs(config[i] - current_angles[i]) / velocity_limits[i]
        
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



def generate_trajectory(start_cfg, target_cfg, velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    # calculate the number of discrete time steps
    time_acc = np.ones((1,6))[0] * times_acc
    time_dec = np.ones((1,6))[0] * times_dec
    time_end = np.ones((1,6))[0] * times_end
    
     # latest total motion time
    time_steps_max = np.int(time_end.max() / _SAMPLE_RATE)
    
    # improvised
    trajectory = np.empty([time_steps_max, 6])
    diff = target_cfg - start_cfg
    
    sign = np.sign(diff)
    delta = np.zeros(len(sign))
    
    # determine of the trajectory
    for i in xrange(0, time_steps_max):
        # calculate of time t
        t = i * _SAMPLE_RATE 
        
        # calculate of the angle difference at time t for a trapeziod velocity profile
        for j in xrange(0, len(delta)):
            if t < time_acc[j]:   #       t < ta
                delta[j] = 0.5 * t**2 * acceleration_limit[j]
            elif t < time_dec[j]: # ta <= t < td
                delta[j] = velocity_limit[j] * ( t - 0.5 * time_acc[j])
            elif t < time_end[j]: # td <= t < te
                delta[j] = velocity_limit[j] * time_dec[j] - 0.5 * acceleration_limit[j] * ( time_end[j] - t)**2                
            else:                 # te <= t
                delta[j] = np.abs(diff[j])
        
        # calculate the angle position at time t 
        # delta * sign, why the angle difference is an absolute value
        trajectory[i] = start_cfg + delta * sign
    
    trajectory[ time_steps_max - 1] = target_cfg
    
    return trajectory



def discretize(velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    
    # adjust times_acc, times_dec, times_end to satisfy
    #  - n = number of discrete time steps
    #  - h = _SAMPLE_RATE
    #
    # times_acc = n_acc * h, n_acc element of N+
    # times_end = n_end * h, n_end element of N+
    # times_dec = n_dec * h, n_dec element of N+

    # acceleration stop time
    times_acc_discrete = np.round(times_acc + _SAMPLE_RATE_CEIL_OFFSET, _SAMPLE_RATE_DECIMAL)

    # cross-multiplication to scale the acceleration limit to the new times_acc
    scalar = (1 / times_acc_discrete) * times_acc
    acceleration_limit_discrete = acceleration_limit * scalar
 
    # total motion time
    times_end_discrete = np.round(times_end + (times_acc_discrete - times_acc) + _SAMPLE_RATE_CEIL_OFFSET, _SAMPLE_RATE_DECIMAL)
    
    # cross-multiplication to scale the velocity limit to the new times_end
    scalar = (1 / times_end_discrete) * times_end
    velocity_limit_discrete = velocity_limit * scalar
    
    # deceleration start time
    times_dec_discrete = times_end_discrete - times_acc_discrete

    return velocity_limit_discrete, acceleration_limit_discrete, times_acc_discrete, times_dec_discrete, times_end_discrete



def limits_and_times(robot, distance, motiontype):
    # calculate
    velocity_limit = np.zeros(robot.GetDOF())
    acceleration_limit = np.zeros(robot.GetDOF())
    times_acc = np.zeros(robot.GetDOF()) # acceleration stop time
    times_dec = np.zeros(robot.GetDOF()) # deceleration start time
    times_end = np.zeros(robot.GetDOF()) # total motion time
    
    if motiontype == "A": # asynchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    elif motiontype == "S": # synchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_synchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)    
    elif motiontype == "F": # full synchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_full_synchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    else: # default to asynchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end



def limits_and_times_asynchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    # maximum acceleration
    acceleration_limit = robot.GetDOFAccelerationLimits()
    
    # maximum amplitude of velocity ramp (triangle situation)
    v_limit = np.sqrt(distance * acceleration_limit)
    velocity_limit = np.minimum(v_limit,robot.GetDOFVelocityLimits())

    # points in time
    times_acc = velocity_limit / acceleration_limit
    times_end = (distance / velocity_limit) + times_acc
    times_dec = times_end - times_acc
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end



def limits_and_times_synchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    
    # latest total motion time
    temax = times_end.max()
    
    velocity_limit = (temax * acceleration_limit / 2) - np.sqrt((math.pow(temax, 2) * np.power(acceleration_limit, 2)) / 4 - acceleration_limit * distance)
    
    # points in time
    times_acc = velocity_limit / acceleration_limit
    times_end = temax
    times_dec = times_end - times_acc
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end
   
   

def limits_and_times_full_synchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(robot, distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
        
    # latest deceleration start time
    tdmax = times_dec.max()
    
    # latest acceleration stop time
    tamax = times_acc.max()
    
    # recalculate velocity and acceleration limit
    velocity_limit = distance / tdmax
    acceleration_limit = velocity_limit / tamax

    # points in time
    times_acc = velocity_limit / acceleration_limit
    times_end = (distance / velocity_limit) + times_acc
    times_dec = times_end - times_acc
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end


