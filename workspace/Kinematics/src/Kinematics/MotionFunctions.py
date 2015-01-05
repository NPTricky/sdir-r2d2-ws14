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

def distance_abs(start_cfg, target_cfg):
    distance = np.fabs(target_cfg - start_cfg)
    distance = distance.clip(min = _EPS)
    return distance
    
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
    distance = distance_abs(start_cfg, target_cfg)
    
    print 'start_cfg',start_cfg
    print 'target_cfg',target_cfg
    print 'distance_abs',distance
    
    #TODO: Implement PTP (Replace pseudo implementation with your own code)! Consider the max. velocity and acceleration of each axis
    
    # calculate velocity, acceleration and points in time for the trajectory interpolation
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times(robot, distance, motiontype)
    print 'velocity_limit',velocity_limit
    print 'acceleration_limit',acceleration_limit
    print 't_acc',times_acc
    print 't_dec',times_dec
    print 't_end',times_end
    
    # make values discrete for trajectory interpolation
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = discretize(distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    print '(discrete) velocity_limit',velocity_limit
    print '(discrete) acceleration_limit',acceleration_limit
    print '(discrete) t_acc',times_acc
    print '(discrete) t_dec',times_dec
    print '(discrete) t_end',times_end
    
    # trajectory interpolation
    trajectory = generate_trajectory(robot, start_cfg, target_cfg, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    
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



def generate_trajectory(robot, start_cfg, target_cfg, velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    # calculate the number of discrete time steps
    time_steps_acc = times_acc / _SAMPLE_RATE
    time_steps_end = times_end / _SAMPLE_RATE
    time_steps_dec = times_dec / _SAMPLE_RATE

    # maximum time step count
    time_steps_max = int(time_steps_end.max())

    trajectory = np.empty([time_steps_max, robot.GetDOF()])
    trajectory[0] = start_cfg
    trajectory[time_steps_max-1] = target_cfg
    
    diff = target_cfg - start_cfg
    sign = np.sign(diff)
    delta = np.zeros(robot.GetDOF())
    
    # trajectory generation
    for i in xrange(1,time_steps_max-1):
        # calculation of time t
        t = i * _SAMPLE_RATE
         
        # calculation of the angle difference at time t for the (trapezoid) velocity profile
        #  acc:          t < t_acc
        #  dec: t_acc <= t < t_dec
        #  end: t_dec <= t < t_end
        # idle: t_end <= t
        for j in xrange(robot.GetDOF()):
            if (i <= time_steps_acc[j]):
                delta[j] = 0.5 * acceleration_limit[j] * math.pow(t, 2)
            elif (i <= time_steps_dec[j]):
                delta[j] = (velocity_limit[j] * t) - (np.power(velocity_limit[j], 2) / (2 * acceleration_limit[j]))
            elif (i <= time_steps_end[j]):
                delta[j] = (velocity_limit[j] * times_dec[j]) - ((0.5 * acceleration_limit[j]) * np.power(times_end[j] - t, 2))
            else:
                delta[j] = math.fabs(target_cfg[j])
                
        # calculate the angular change at time t
        # delta * sign, because the angular change is an absolute value
        trajectory[i,:] = start_cfg + delta * sign
        print trajectory[i,:]
    
    #===========================================================================
    # # trajectory generation (without inner loop)
    # for i in xrange(1,time_steps_max-1):
    #     # calculation of time t
    #     t = i * _SAMPLE_RATE
    #     
    #     # calculation of the angle difference at time t for the (trapezoid) velocity profile
    #     #  acc:          t < t_acc
    #     #  dec: t_acc <= t < t_dec
    #     #  end: t_dec <= t < t_end
    #     # idle: t_end <= t
    #     acc_condition = np.less_equal(i,time_steps_acc)
    #     a = np.where(acc_condition,0.5 * acceleration_limit * math.pow(t, 2),0)
    #     dec_condition = np.logical_and(np.greater(i,time_steps_acc),np.less_equal(i,time_steps_dec)) 
    #     d = np.where(dec_condition,(velocity_limit * t) - (np.power(velocity_limit, 2) / (2 * acceleration_limit)),0)
    #     end_condition = np.logical_and(np.greater(i,time_steps_dec),np.less_equal(i,time_steps_end))
    #     e = np.where(end_condition,(velocity_limit * times_dec) - ((0.5 * acceleration_limit) * np.power(times_end - t, 2)),0)
    #     idle_condition = np.greater(i,time_steps_end)
    #     i = np.where(idle_condition,np.abs(target_cfg),0)
    #     delta = a + d + e + i
    #     
    #     # calculate the angular change at time t 
    #     # delta * sign, because the angular change is an absolute value
    #     trajectory[i,:] = start_cfg + delta * sign
    #     print trajectory[i,:]
    #===========================================================================

    return trajectory



def discretize(distance, velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    
    # adjust times_acc, times_dec, times_end to satisfy
    #  - n = number of discrete time steps
    #  - h = _SAMPLE_RATE
    #
    # times_acc = n_acc * h, n_acc element of N+
    # times_end = n_end * h, n_end element of N+
    # times_dec = n_dec * h, n_dec element of N+

    # acceleration stop time
    times_acc_discrete = np.round(times_acc + _SAMPLE_RATE_CEIL_OFFSET, _SAMPLE_RATE_DECIMAL)
    # total motion time
    times_end_discrete = np.round(times_end + (times_acc_discrete - times_acc) + _SAMPLE_RATE_CEIL_OFFSET, _SAMPLE_RATE_DECIMAL)
    # deceleration start time
    times_dec_discrete = times_end_discrete - times_acc_discrete
    
    #===========================================================================
    # # does not work due to numerical imprecision
    # 
    # # cross-multiplication to scale the acceleration limit to the new times_acc
    # scalar = times_acc / times_acc_discrete
    # acceleration_limit_discrete = acceleration_limit * scalar
    # 
    # # cross-multiplication to scale the velocity limit to the new times_end
    # scalar = times_end / times_end_discrete
    # velocity_limit_discrete = velocity_limit * scalar
    #===========================================================================
    
    # recalculate velocity limit
    velocity_limit_discrete = distance / times_dec_discrete
    
    # recalculate acceleration limit
    acceleration_limit_discrete = velocity_limit_discrete / times_acc_discrete
    
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
    
    # recalculate velocity limit
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
    times_end = (distance / velocity_limit) + times_acc # required due to recalculation of velocity limit
    times_dec = times_end - times_acc
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end


