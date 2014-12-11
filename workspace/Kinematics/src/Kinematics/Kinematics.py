from openravepy import *
import numpy as np

_DH_KUKA_KR30L16 = np.matrix([[-np.pi/2,   815, 350, np.pi/2],
                              [       0,     0,1200,-np.pi/2],
                              [ np.pi/2,     0, 145,       0],
                              [-np.pi/2, -1545,   0,       0],
                              [ np.pi/2,     0,   0,       0],
                              [   np.pi,  -158,   0,       0]])

"""
@type robot: model of the robot
@param robot: robot instance
@return:
"""
def forward(robot):

    if len(robot.GetManipulators()) > 1:
        for m in robot.GetManipulators():
            print "ERROR"
            print " - Manipulator: \""+m.GetName()+"\" - Base: \""+m.GetBase().GetName()+"\" - End Effector: \""+m.GetEndEffector().GetName()+"\""
        return
    
    m = robot.GetManipulators()[0]
    
    # transformation matrix of the base as a starting point       
    A = m.GetBase().GetTransform()
    
    DH = _DH_KUKA_KR30L16
    rows, cols = DH.shape
    # apply the denavit-hartenberg parameters for homogeneous transformation
    for i in range(0, rows-1):
        A = A * homogeneous_transformation(DH[i,0], DH[i,1], DH[i,2], DH[i,3])
    
    # transform the tool into the coordinate system of the base
    A = A * m.GetEndEffector().GetTransform()
    
    return A

""" Homogeneous transformation by given parameters.

@type alpha: angle in radiant
@param alpha: link twist
@type d: millimeter
@param d: link offset
@type a: millimeter
@param a: link length
@type theta: angle in radiant
@param theta: joint angle
@return: homogeneous coordinates
"""
def homogeneous_transformation(alpha, d, a, theta):
    ca = np.cos(alpha)
    sa = np.cos(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)
    return np.matrix([[ct,-st*ca, st*sa, a*ct],
                      [st, ct*ca,-ct*sa, a*st],
                      [ 0,    sa,    ca,    d],
                      [ 0,     0,     0,    1]])

"""
@type robot: model of the robot
@param robot: robot instance
@type pose: homogeneous coordinates
@param pose: pose of the end effector
@return: pose of the end effector in joint coordinates
"""
def inverse(robot,pose):
    return
