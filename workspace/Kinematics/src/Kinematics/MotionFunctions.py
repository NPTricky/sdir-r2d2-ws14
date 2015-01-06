from openravepy import *
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

_DEBUG_DRAW = []

def difference_rel(start_cfg, target_cfg):
    # calculate the difference between positions in space
    diff_pos = target_cfg[:3] - start_cfg[:3]
    # calculate the difference between angles
    diff_angle = target_cfg[3:] - start_cfg[3:]
    diff_angle = (diff_angle + np.pi) % (2 * np.pi) - np.pi
    # concatenate the results
    diff = np.concatenate((diff_pos, diff_angle))
    return diff

def distance_rel(start_cfg, target_cfg):
    diff = difference_rel(start_cfg, target_cfg)
    dist_pos = np.sqrt(np.sum(np.power(diff[:3],2)))
    dist_angle = np.sqrt(np.sum(np.power(diff[3:],2))) 
    return dist_pos, dist_angle

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
    diff_r = difference_rel(start_cfg, target_cfg)
    diff_a = np.fabs(difference_rel(start_cfg, target_cfg))
    
    print 'start_cfg',start_cfg
    print 'target_cfg',target_cfg
    print 'difference_rel',diff_r
    print 'difference_abs',diff_a
    
    #TODO: Implement PTP (Replace pseudo implementation with your own code)! Consider the max. velocity and acceleration of each axis
    
    if motiontype == "L":
        trajectory = linear_trajectory_interpolation(robot, start_cfg, target_cfg, 0.5, 1.5)
        return trajectory 
    
    # calculate velocity, acceleration and points in time for the trajectory interpolation
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times(robot, diff_a, motiontype)
    print 'velocity_limit',velocity_limit
    print 'acceleration_limit',acceleration_limit
    print 't_acc',times_acc
    print 't_dec',times_dec
    print 't_end',times_end
    
    # make values discrete for trajectory interpolation
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = discretize(diff_a, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
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
        _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin.forward(trajectory[i]), 0.2, 0.5))
        time.sleep(_SAMPLE_RATE)



"""
@type robot: model of the robot
@param robot: robot instance
@rtype: configurations of angles, angle in radiant
@return: configuration for each joint of the robot
"""
def get_fastest_inverse_solution(robot, configurations):
    current_angles = robot.GetDOFValues()
    velocity_limits = robot.GetDOFVelocityLimits()
       
    times_acc = velocity_limits / robot.GetDOFAccelerationLimits()
    times_min = sys.maxsize
    
    # the function checked, if the configuration for this robot possible
    possible_confs = get_possible_inverse_solution(robot, configurations)
    
    # calculate the movement time for each configuration and give back
    # the fastest configuration
    ret_value = []
    for conf in possible_confs:
        times_end = np.fabs(conf - current_angles) / velocity_limits + times_acc
        
        if times_end.max() < times_min:
            times_min = times_end.max()
            ret_value = conf
        
    return ret_value

"""
@type robot: model of the robot
@param robot: robot instance
@rtype: configurations of angles, angle in radiant
@return: configuration for each joint of the robot
"""
def get_lowest_difference_inverse_solution(robot, configurations):

    current_angles = robot.GetDOFValues()
    
    # the function checked, if the configuration for this robot possible
    possible_confs = get_possible_inverse_solution(robot, configurations)
    
    # calculate the movement time for each configuration and give back
    # the fastest configuration
    ret_value = []
    for conf in possible_confs:
        print np.sum( np.fabs(conf - current_angles))
                
    return ret_value

"""
@type robot: model of the robot
@param robot: robot instance
@rtype: configurations of angles, angle in radiant
@return: configuration for each joint of the robot
"""
def get_possible_inverse_solution(robot, configurations):

    if configurations is None:
        return None
        
    angle_limits = robot.GetDOFLimits()
    
    ret_value = []
    for conf in configurations:
        if np.less_equal(angle_limits[0], conf).all() and np.less_equal(conf, angle_limits[1]).all():
            ret_value.append(conf)
            
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
        #  acc:          i <= ts_acc
        #  dec: ts_acc < i <= ts_dec
        #  end: ts_dec < i <= ts_end
        # idle: ts_end < i
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
        print trajectory[i]
    #===========================================================================
    # # trajectory generation (without inner loop)
    # for i in xrange(1,time_steps_max-1):
    #     # calculation of time t
    #     t = i * _SAMPLE_RATE
    #     
    #     # calculation of the angle difference at time t for the (trapezoid) velocity profile
    #     #  acc:          i <= ts_acc
    #     #  dec: ts_acc < i <= ts_dec
    #     #  end: ts_dec < i <= ts_end
    #     # idle: ts_end < i
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
    # knowledge base
    velocity_limit = robot.GetDOFVelocityLimits()
    acceleration_limit = robot.GetDOFAccelerationLimits()
    
    if motiontype == "A": # asynchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(distance, velocity_limit, acceleration_limit)
    elif motiontype == "S": # synchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_synchronous(distance, velocity_limit, acceleration_limit)    
    elif motiontype == "F": # full synchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_full_synchronous(distance, velocity_limit, acceleration_limit)
    else: # default to asynchronous
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(distance, velocity_limit, acceleration_limit)
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end



def limits_and_times_asynchronous(distance, velocity_limit, acceleration_limit):
    # maximum amplitude of velocity ramp (triangle situation)
    v_limit = np.sqrt(distance * acceleration_limit)
    velocity_limit = np.minimum( v_limit, velocity_limit )

    # points in time
    times_acc = velocity_limit / acceleration_limit
    times_end = (distance / velocity_limit) + times_acc
    times_dec = times_end - times_acc
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end



def limits_and_times_synchronous(distance, velocity_limit, acceleration_limit):
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(distance, velocity_limit, acceleration_limit)
    
    # latest total motion time
    temax = times_end.max()
    
    # recalculate velocity limit
    velocity_limit = (temax * acceleration_limit / 2) - np.sqrt((math.pow(temax, 2) * np.power(acceleration_limit, 2)) / 4 - acceleration_limit * distance)
    
    # points in time
    times_acc = velocity_limit / acceleration_limit
    times_end = np.ones(len(distance)) * temax
    times_dec = times_end - times_acc
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end
   
   

def limits_and_times_full_synchronous(distance, velocity_limit, acceleration_limit):
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(distance, velocity_limit, acceleration_limit)
        
    # latest acceleration stop time
    tamax = times_acc.max()
    
    # latest deceleration start time
    tdmax = times_dec.max()
    
    # recalculate velocity and acceleration limit
    velocity_limit = distance / tdmax
    acceleration_limit = velocity_limit / tamax

    # points in time
    times_acc = np.ones(len(distance)) * tamax
    times_dec = np.ones(len(distance)) * tdmax
    times_end = times_acc + times_dec
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end



def linear_trajectory_interpolation(robot, start_cfg, target_cfg, velocity, acceleration):
       
    # determine the start and the end position in space
    start_matrix = kin.forward(start_cfg)
    start_pose = kin.get_pose_from( start_matrix )
    
    target_matrix = kin.forward(target_cfg)
    target_pose = kin.get_pose_from( target_matrix )

    _DEBUG_DRAW.append(robot.GetEnv().drawlinestrip(points=np.array(((start_pose[0],start_pose[1],start_pose[2]),(target_pose[0],target_pose[1],target_pose[2]))),
                                                    linewidth=1.0,
                                                    colors=np.array(((0,0,0),(0,0,0))))) 
    
    # calculate the difference form start to end position and the distance
    diff = difference_rel(start_pose, target_pose)
    sign_diff = np.sign(diff)
    dist = np.sqrt( diff[0]**2 + diff[1]**2 + diff[2]**2 )
        
    # determine of the times and recalculate of the velocity
    time_end = velocity / acceleration + dist / velocity
    
    velocity = (time_end * acceleration / 2) - np.sqrt((math.pow(time_end, 2) * np.power(acceleration, 2)) / 4 - acceleration * dist)
    
    time_acc = velocity / acceleration
    time_dec = dist / velocity
    
    # interpolation of the trajectory path
    sample_rate = _SAMPLE_RATE * 10
    time_steps_max = np.int(time_end / sample_rate)
    
    lin_cfg = []
    lin_cfg.append(start_cfg)
    
    for i in range(1, time_steps_max - 1):
        # calculate the time t 
        t = i * sample_rate
        
        # generate a trapezoid trajectory path
        if t < time_acc:   #       t < ta
            delta = diff * (0.5 * t**2 * acceleration) / dist 
        elif t < time_dec: # ta <= t < td
            delta = diff * (velocity * ( t - 0.5 * time_acc)) / dist
        elif t < time_end: # td <= t < te
            delta = diff * (velocity * time_dec - 0.5 * acceleration * ( time_end - t)**2) / dist 
        else:               # te <= t
            delta = diff
        
        # determine the current position and orientation at time t
        current_pose = start_pose + delta
        
        # determine the inverse kinematics for the pose
        inverse_cfgs = get_possible_inverse_solution(robot, kin.inverse(current_pose))
        
        if len(inverse_cfgs) == 0:
            return None
        
        lin_cfg.append(inverse_cfgs[0])
        
    lin_cfg.append(target_cfg)
    
    
    # generation of the trajectory path
    velocity = robot.GetDOFVelocityLimits()
    acceleration = robot.GetDOFAccelerationLimits()
    
    trajectory = np.ones([1,6]) * start_cfg
    
    last_cfg = start_cfg;
    for cfg in lin_cfg[1:]:
        
        distance_a = distance_abs(last_cfg, cfg)
                
        # calculate velocity, acceleration and points in time for the trajectory interpolation
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_full_synchronous(distance_a, velocity, acceleration)
        
        # make values discrete for trajectory interpolation
        velocity_limit, acceleration_limit, times_acc, times_dec, times_end = discretize(distance_a, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
        
        current_trajectory = generate_trajectory(robot, last_cfg, cfg, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)       
        
        # trajectory interpolation
        trajectory = np.concatenate((trajectory, current_trajectory[1:]))
                
        last_cfg = cfg
    
    return trajectory
