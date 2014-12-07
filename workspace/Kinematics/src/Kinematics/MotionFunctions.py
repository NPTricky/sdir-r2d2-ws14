import numpy as np
import time

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
        time.sleep(0.01)
