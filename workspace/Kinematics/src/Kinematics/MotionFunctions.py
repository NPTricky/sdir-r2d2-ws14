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

def difference_rel(start_pose, target_pose):
    # calculate the difference between positions in space
    diff_pos = target_pose[:3] - start_pose[:3]
    # calculate the difference between angles
    diff_angle = target_pose[3:] - start_pose[3:]
    diff_angle = (diff_angle + np.pi) % (2 * np.pi) - np.pi
    # concatenate the results
    diff = np.concatenate((diff_pos, diff_angle))
    return diff

def distance_rel(start_pose, target_pose):
    diff = difference_rel(start_pose, target_pose)
    # euclidean distance
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
    start_cfg = robot.GetDOFValues()
    diff_r = difference_rel(start_cfg, target_cfg)
    diff_a = np.fabs(diff_r)
    dist_pos, dist_angle = distance_rel(start_cfg, target_cfg)
    
    print 'start_cfg',start_cfg
    print 'target_cfg',target_cfg
    print 'difference_rel',diff_r
    print 'difference_abs',diff_a
    print 'distance_pos',dist_pos,'distance_angle',dist_angle
    
    #TODO: Implement PTP (Replace pseudo implementation with your own code)! Consider the max. velocity and acceleration of each axis
    motion_options = motiontype.split(';')
    
    configurations = ((start_cfg, target_cfg))
    
    if motion_options[2] == "R":
        print "rrt"
        configurations = ((start_cfg, np.array((-0.6696,-0.466, 0.5493,1.4661,-0.6739,-1.4371)), target_cfg))
        #configurations = rrt.rrt
        
    if motion_options[1] == "L":
    
        config = np.ones([1,len(configurations[0])]) * configurations[0]
        for cfg in configurations[1:]:
            # determine the start and the end position in space
            start_pose  = kin.get_pose_from(kin.forward(config[-1]))    
            target_pose = kin.get_pose_from(kin.forward(cfg))

            _DEBUG_DRAW.append(robot.GetEnv().drawlinestrip(points=np.array(((start_pose[0],start_pose[1],start_pose[2]),(target_pose[0],target_pose[1],target_pose[2]))),
                                                            linewidth=1.0,
                                                            colors=np.array(((0,0,0),(0,0,0))))) 
            
            sample_rate = _SAMPLE_RATE * 10
        
            interpolated_path = linear_path_interpolation( robot, start_pose, target_pose, sample_rate)
        
            config_next = convert_path_interpolation_to_configurations( robot, config[-1], cfg, interpolated_path)
            config = np.concatenate((config, config_next[1:]))
        
        configurations = config
        

    trajectory = generate_trajectory_from_configurations(robot, configurations, motion_options[0])
    
    """
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
    trajectory = generate_trajectory(start_cfg, target_cfg, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    """
    #return None
    return trajectory

def Move(robot, trajectory):
    #_DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin._BASE, 4.0, 4.5))
    if not trajectory is None:
        for i in range(trajectory.shape[0]):
            robot.SetDOFValues(trajectory[i])
            _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin.forward(trajectory[i]), 0.1, 0.3))
            time.sleep(_SAMPLE_RATE)


def get_fastest_inverse_solution(robot, configurations):
    """
    @type robot: model of the robot
    @param robot: robot instance
    @rtype: configurations of angles, angle in radiant
    @return: configuration for each joint of the robot
    """
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

def get_nearest_angle_solution(current_cfg, possible_confs):
    """
    @type current_cfg: angle in radiant
    @param current_cfg: current angle configuration for each joint of the robot
    @type: possible_confs: angle in radiant
    @param: possible_confs: possible configurations for each joint of the robot
    @rtype: one configuration from the possible configurations, angle in radiant
    @return: the best configuration with the lowest angle difference
    """

    indices = np.array(xrange(len(possible_confs)))
    configurations = possible_confs
    # compare mask for possible solutions
    mask_angle = np.array(xrange(len(current_cfg))) 
        
    for j in xrange(len(possible_confs)):
        
        # determine the angle difference from the configurations and the current configuration
        diff_confs_abs = np.fabs(configurations - current_cfg)
    
        # determine the minimum angles difference
        min = np.amin(diff_confs_abs,0)
        
        # compare the angle differences with the minimum angle difference
        res = np.where(diff_confs_abs == min)
            
        # compare the result with a mask for determine of the lowest
        # angle difference solution for the first three angles 
        mask = mask_angle
        indices = []
        for j in xrange(len(current_cfg)):
            for i in xrange(len(res[1]) - len(mask)):
                if np.all(res[1][i:i+len(mask)] == mask):
                    indices.append(res[0][i])
            
            if 0 < len(indices): 
                break
            
            # if the angle mask not found, resize the mask
            mask = np.delete(mask, len(mask) - 1)
                
        if len(indices) < 2:
            break
        
        for i in xrange(0,len(indices)):
            configurations[i] = configurations[indices[i]]
        
        configurations = configurations[:len(indices)]
        
    return configurations[indices[0]]

def get_possible_inverse_solution(robot, configurations):
    """
    @type robot: model of the robot
    @param robot: robot instance
    @type configurations: angles in radiant
    @param configurations: configuration for each joint of the robot
    @rtype: configurations of angles, angle in radiant
    @return: possible configuration for each joint of the robot 
    """

    if configurations is None:
        return None
        
    angle_limits = robot.GetDOFLimits()
        
    ret_value = []
    for conf in configurations:
        if np.less_equal(angle_limits[0], conf).all() and np.less_equal(conf, angle_limits[1]).all():
            ret_value.append(conf)
            
    return ret_value
    
    
def generate_trajectory(start_cfg, target_cfg, velocity_limit, acceleration_limit, times_acc, times_dec, times_end):
    # calculate the number of discrete time steps
    time_steps_acc = times_acc / _SAMPLE_RATE
    time_steps_end = times_end / _SAMPLE_RATE
    time_steps_dec = times_dec / _SAMPLE_RATE
    
    # maximum time step count
    time_steps_max = int(time_steps_end.max() + 0.5)
    
    trajectory = np.empty([time_steps_max + 1, len(start_cfg)])
    trajectory[0] = start_cfg
    trajectory[-1] = target_cfg
    
    diff = target_cfg - start_cfg
    sign = np.sign(diff)
    
    acceleration = acceleration_limit * sign
    velocity = velocity_limit * sign
    
    delta = np.zeros(len(start_cfg))
    
    # trajectory generation
    for i in xrange(1,time_steps_max):
        # calculation of time t
        t = i * _SAMPLE_RATE
         
        # calculation of the angle difference at time t for the (trapezoid) velocity profile
        #  acc:          i <= ts_acc
        #  dec: ts_acc < i <= ts_dec
        #  end: ts_dec < i <= ts_end
        # idle: ts_end < i
        for j in xrange(len(start_cfg)):
            if (i <= time_steps_acc[j]):
                delta[j] = 0.5 * acceleration[j] * np.power(t, 2)
            elif (i <= time_steps_dec[j]):
                delta[j] = velocity[j] * ( t - 0.5 * times_acc[j])
            elif (i <= time_steps_end[j]):
                delta[j] = (velocity[j] * times_dec[j]) - ((0.5 * acceleration[j]) * np.power(times_end[j] - t, 2))
            else:
                delta[j] = diff[j]
        
        # calculate the angular change at time t
        # delta * sign, because the angular change is an absolute value
        trajectory[i,:] = start_cfg + np.nan_to_num(delta)

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
   
def concatenate_trajectory(robot, trajectory_0, trajectory_1, gind_over, velocity_profile_0, velocity_profile_1):
    
    if not gind_over:
        return np.concatenate((trajectory_0, trajectory_1[1:]))
        
    time_dec0_to_end0 = get_ta(velocity_profile_0).max()
    time_end0_to_acc1 = get_ta(velocity_profile_1).max()
    time_end =  time_dec0_to_end0 + time_end0_to_acc1
            
    velocity = np.minimum( np.fabs(get_v(velocity_profile_1) - get_v(velocity_profile_0)), robot.GetDOFVelocityLimits())
    acceleration = np.minimum(np.nan_to_num( 2 * velocity / time_end), robot.GetDOFAccelerationLimits())
    
    time_end = np.nan_to_num(velocity / acceleration).max()
    
    time_steps_max = int(time_end / _SAMPLE_RATE + 0.5)
    
    if time_steps_max < 0:
        return np.concatenate((trajectory_0, trajectory_1[1:]))
    
    index_0 = np.minimum(np.abs(len(trajectory_0) - int(time_dec0_to_end0 / _SAMPLE_RATE)-1), len(trajectory_0) - 1)
    index_1 = np.minimum(int(time_end0_to_acc1 / _SAMPLE_RATE)+1, len(trajectory_1) - 1)
    
    start_cfg = trajectory_0[index_0]
    target_cfg = trajectory_1[index_1]
    
    diff = target_cfg - start_cfg
    velocity = diff / time_end
    
    #_DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin.forward(start_cfg), 0.25, 0.5))
    #_DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin.forward(target_cfg), 0.25, 0.5))
    
    trajectory_grind = np.empty([time_steps_max, len(start_cfg)])
    for i in xrange(time_steps_max):
        t = i * _SAMPLE_RATE   
        trajectory_grind[i,:] = start_cfg + velocity * t    
       # _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin.forward(trajectory_grind[i,:]), 0.25, 0.5))
        
    return np.concatenate((trajectory_0[:index_0], trajectory_grind, trajectory_1[index_1:]))

def generate_trajectory_from_configurations(robot, configurations, motiontype = "A"):
    # generation of the trajectory path

    i = 0
    last_velocity_profile = [ np.zeros(robot.GetDOF()) for i in range(5)]
    trajectory = np.ones([1,len(configurations[0])]) * configurations[0]
    
    for cfg in configurations[1:]:
        
        distance = np.fabs(cfg - trajectory[-1])

        # calculate velocity, acceleration and points in time for the trajectory interpolation
        velocity_profile = limits_and_times(robot, distance, motiontype)
        
        # make values discrete for trajectory interpolation
        velocity_profile = discretize(distance, get_v(velocity_profile), get_a(velocity_profile), get_ta(velocity_profile), get_td(velocity_profile), get_te(velocity_profile))
        
        current_trajectory = generate_trajectory(trajectory[-1], cfg, get_v(velocity_profile), get_a(velocity_profile), get_ta(velocity_profile), get_td(velocity_profile), get_te(velocity_profile))       
    
        trajectory = concatenate_trajectory( robot, trajectory, current_trajectory, 1 < i , last_velocity_profile, velocity_profile)
        
        i = i + 1
        last_velocity_profile = velocity_profile
        
    return trajectory
 
    
def get_v(velocity_profile):
    return velocity_profile[0]
    
def get_a(velocity_profile):
    return velocity_profile[1]
    
def get_ta(velocity_profile):
    return velocity_profile[2]
    
def get_td(velocity_profile):
    return velocity_profile[3]
    
def get_te(velocity_profile):
    return velocity_profile[4]
      
 
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
    velocity_limit_discrete = np.nan_to_num( distance / times_dec_discrete )
    
    # recalculate acceleration limit
    acceleration_limit_discrete = np.nan_to_num( velocity_limit_discrete / times_acc_discrete )
    
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
    times_acc = np.nan_to_num(velocity_limit / acceleration_limit)
    times_end = np.nan_to_num((distance / velocity_limit) + times_acc)
    times_dec = times_end - times_acc
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end

def limits_and_times_synchronous(distance, velocity_limit, acceleration_limit):
    velocity_limit, acceleration_limit, times_acc, times_dec, times_end = limits_and_times_asynchronous(distance, velocity_limit, acceleration_limit)
    
    # latest total motion time
    temax = times_end.max()
    
    # recalculate velocity limit
    velocity_limit = (temax * acceleration_limit / 2) - np.sqrt((math.pow(temax, 2) * np.power(acceleration_limit, 2)) / 4 - acceleration_limit * distance)
    
    # points in time
    times_acc = np.nan_to_num(velocity_limit / acceleration_limit)
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
    velocity_limit = np.nan_to_num(distance / tdmax)
    acceleration_limit = np.nan_to_num(velocity_limit / tamax)

    # points in time
    times_acc = np.ones(len(distance)) * tamax
    times_dec = np.ones(len(distance)) * tdmax
    times_end = times_acc + times_dec
    
    return velocity_limit, acceleration_limit, times_acc, times_dec, times_end

    
def linear_path_interpolation(robot, start_pose, target_pose, sample_rate, velocity = 1.0, acceleration=4.0):    
    # create the path interpolation from the target position to the start position
    # calculate the difference form start to end position and the distance
    diff = difference_rel(start_pose, target_pose)
    dist = np.amax(distance_rel(start_pose, target_pose))
    diff_dist_ratio = diff / dist
    
    # determine of the times and recalculate of the velocity
    time_end = velocity / acceleration + dist / velocity
    
    print "dist", dist
    print "dist 0", distance_rel(start_pose, target_pose)[0]
    print "dist 1", distance_rel(start_pose, target_pose)[1]
    print "velocity", velocity
    print "time_end", time_end
    velocity = (time_end * acceleration / 2) - np.sqrt((math.pow(time_end, 2) * np.power(acceleration, 2)) / 4 - acceleration * dist)
    
    time_acc = velocity / acceleration
    time_dec = dist / velocity
    time_end = time_dec + time_acc
    
    print "velocity", velocity
    print "time_end_new", time_end
    
    
    time_steps_acc = time_acc / sample_rate
    time_steps_end = time_end / sample_rate
    time_steps_dec = time_dec / sample_rate
    
    # interpolation of the trajectory path
    time_steps_max = np.int(time_end / sample_rate + 0.5)
    
    interpolated_path = np.empty([time_steps_max + 1, len(start_pose)])
    
    interpolated_path[0] = start_pose
    interpolated_path[-1] = target_pose

    #_DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin.get_matrix_from(interpolated_path[0]), 0.3, 0.5))
    #_DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin.get_matrix_from(interpolated_path[-1]), 0.3, 0.5))
    for i in range(1, time_steps_max):
        # calculate the time t 
        t = i * sample_rate
        
        # generate a trapezoid trajectory path
        if i < time_steps_acc:   #       t < ta
            delta = diff_dist_ratio * (0.5 * t**2 * acceleration)
        elif i < time_steps_dec: # ta <= t < td
            delta = diff_dist_ratio * (velocity * ( t - 0.5 * time_acc))
        elif i < time_steps_end: # td <= t < te
            delta = diff_dist_ratio * (velocity * time_dec - 0.5 * acceleration * ( time_end - t)**2)
        else:               # te <= t
            delta = diff
        
        #delta = diff * i / time_steps_max
        
        # determine the current position and orientation at time t
        interpolated_path[i] = (start_pose + delta )
        
        _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin.get_matrix_from(interpolated_path[i]), 0.2, 0.5))
        
        
        #_DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin.get_matrix_from(interpolated_path[i]), 0.2, 0.5))
        
    return interpolated_path

def convert_path_interpolation_to_configurations(robot, start_cfg, target_cfg, interpolated_path):
    
    configurations = []
    configurations.append(target_cfg) 
    
    for pose in (interpolated_path[::-1])[1:-2]:
    
        # determine the inverse kinematics for the pose
        inverse_configurations = get_possible_inverse_solution(robot, kin.inverse(pose, configurations[-1]))
        
        if len(inverse_configurations) == 0:
            return None
        
        inverse_configuration = inverse_configurations[0]
        
        if np.pi/2 < np.fabs( inverse_configuration - configurations[-1] ).sum():
            inverse_configuration = get_nearest_angle_solution( configurations[-1], inverse_configurations)
            
        if np.pi/2 < np.fabs( inverse_configuration - configurations[-1] ).sum():
            inverse_configurations = get_possible_inverse_solution(robot, kin.inverse(kin.get_pose_from(kin.forward(configurations[-1])), configurations[-1]))
                
            inverse_singularity_solution = inverse_configurations[ int((len(inverse_configurations) + 1) / 2) ] 
            configurations.append(inverse_singularity_solution)
            
        #_DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), kin.forward(configurations[-1]), 0.6, 0.8))
             
        configurations.append(inverse_configuration)
                
    configurations.append(start_cfg)
    
    return configurations[::-1]
    
