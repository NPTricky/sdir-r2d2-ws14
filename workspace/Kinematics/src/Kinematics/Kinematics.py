from openravepy import *
import numpy as np
from sympy.mpmath import *
import math

_DEBUG_DRAW = []

# openrave (alpha, d, a, theta)
_DH_KUKA_KR30L16_OPENRAVE = np.matrix([[  -np.pi,     0,     0,       0],
                                       [-np.pi/2,-0.815,  0.35,       0],
                                       [       0,     0,  -1.2,-np.pi/2],
                                       [ np.pi/2,     0,-0.145,       0],
                                       [-np.pi/2,-1.545,     0,       0],
                                       [ np.pi/2,     0,     0,       0]])

# team modbots (alpha, d, a, theta)
_DH_KUKA_KR30L16_MODBOTS = np.matrix([[-np.pi/2, 0.815,  0.35, np.pi/2],
                                      [       0,     0,   1.2,-np.pi/2],
                                      [ np.pi/2,     0, 0.145,       0],
                                      [-np.pi/2,-1.545,     0,       0],
                                      [ np.pi/2,     0,     0,       0],
                                      [   np.pi,-0.158,     0,       0]])

# team modbots (alpha, d, a, theta)
_DH_KUKA_KR30L16_R2D2 = np.matrix([[-np.pi/2, 0.815,  0.35,       0],
                                      [       0,     0,   1.2,-np.pi/2],
                                      [ np.pi/2,     0, 0.145,       0],
                                      [-np.pi/2,-1.545,     0,       0],
                                      [ np.pi/2,     0,     0,       0],
                                      [   np.pi,-0.158,     0,       0]])

_DH_KUKA_KR30L16 = _DH_KUKA_KR30L16_R2D2

def get_alpha(i):
    return _DH_KUKA_KR30L16[i,0]
def get_d(i):
    return _DH_KUKA_KR30L16[i,1]
def get_a(i):
    return _DH_KUKA_KR30L16[i,2]
def get_theta(i):
    return _DH_KUKA_KR30L16[i,3]

_BASE = np.matrix([[ 1, 0, 0, 0],
                   [ 0, 1, 0, 0],
                   [ 0, 0, 1, 0],
                   [ 0, 0, 0, 1]])

_TOOL = np.matrix([[ 1, 0, 0, 0.25],
                   [ 0, 1, 0,    0],
                   [ 0, 0, 1,    0],
                   [ 0, 0, 0,    1]])

"""
@type robot: model of the robot
@param robot: robot instance
@return: pose of end effector in homogeneous coordinates
"""
def forward(robot):

    if len(robot.GetManipulators()) > 1:
        for m in robot.GetManipulators():
            print "ERROR"
            print " - Manipulator: \""+m.GetName()+"\" - Base: \""+m.GetBase().GetName()+"\" - End Effector: \""+m.GetEndEffector().GetName()+"\""
        return

    # transformation matrix of the base as a starting point       
    A = _BASE
    
    # apply the denavit-hartenberg parameters for homogeneous transformation
    for i in range(0, robot.GetDOF()):
        A = A * homogeneous_transformation_from(i)
        _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), A, 0.5, 2))
    
    # for debug purposes without tool
    # transform the tool into the coordinate system of the base
    #A = A * robot.GetManipulators()[0].GetEndEffector().GetTransform()
    #A = A * _TOOL
    
    extract_euler_angles_from(A)
    
    return A

""" Homogeneous transformation from joint i to i+1

@param i: joint index
"""
def homogeneous_transformation_from(i):
    return homogeneous_transformation(get_alpha(i), get_d(i), get_a(i), get_theta(i))

def extract_euler_angles_from(matrix):
    beta = math.atan2(-matrix[2,0], math.sqrt(math.pow(matrix[0,0], 2) + math.pow(matrix[1,0], 2))) 
    beta = beta / pi * 180         
    alpha = math.atan2(matrix[1,0]/math.cos(beta), matrix[0,0]/math.cos(beta))
    alpha = alpha / pi * 180   
    gamma = math.atan2(matrix[2,1]/math.cos(beta), matrix[2,2]/math.cos(beta))
    gamma = gamma / pi * 180   
    print "Alpha: "+str(alpha)+", Beta: "+str(beta)+", Gamma: "+str(gamma)

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
    ca = math.cos(alpha)
    ct = math.cos(theta)
    sa = math.sin(alpha)
    st = math.sin(theta)
    return np.matrix([[ct,-st*ca, st*sa, a*ct],
                      [st, ct*ca,-ct*sa, a*st],
                      [ 0,    sa,    ca,    d],
                      [ 0,     0,     0,    1]])

"""
@type robot: model of the robot
@param robot: robot instance
@type T: homogeneous coordinates
@param T: pose of end effector
@rtype: joint coordinates
@return: pose of end effector
"""
def inverse(robot,T):
    
    # calculate position of wrist point in coordinate system 0
    pos_wp_0 = T * _TOOL
    print "Pos WP 0:\n"+str(pos_wp_0)

    # calculate position of wrist point in coordinate system 1
    pos_wp_1 = np.linalg.inv(homogeneous_transformation_from(0)) * pos_wp_0
    print "Pos WP 1:\n"+str(pos_wp_1)
    theta_11 = math.atan2(pos_wp_1[0,0], pos_wp_1[1,0])
    
    theta_12 = theta_11
    if theta_11 < 0:
        theta_12 += np.pi
    else:
        theta_12 -= np.pi
    print "Theta 11:\n"+str(theta_11)
    print "Theta 12:\n"+str(theta_12)
    
    # calculate position of wrist point in coordinate system 2 for theta_11 and theta_12
    pos_wp_21 = np.linalg.inv(homogeneous_transformation(get_alpha(1), get_d(1), get_a(1), theta_11)) * pos_wp_1
    print "Pos WP 21:\n"+str(pos_wp_21)
    pos_wp_22 = np.linalg.inv(homogeneous_transformation(get_alpha(1), get_d(1), get_a(1), theta_12)) * pos_wp_1
    print "Pos WP 22:\n"+str(pos_wp_22)
    # theta_21
    d_h = math.sqrt(math.pow(get_a(3), 2) + math.pow(get_d(4), 2))
    d_wp_2 = math.sqrt(math.pow(pos_wp_21[0,0], 2) + math.pow(pos_wp_21[1,0], 2))
    alpha_h_1 = mp.sec(math.pow(d_h, 2) - math.pow(d_wp_2, 2) - math.pow(get_a(2), 2))
    alpha_h_2 = math.atan2(pos_wp_21[0,0], pos_wp_21[1,0])
    theta_21 = alpha_h_1 + alpha_h_2
    # theta_22
    d_wp_2 = math.sqrt(math.pow(pos_wp_22[0,0], 2) + math.pow(pos_wp_22[1,0], 2))
    alpha_h_1 = mp.sec(math.pow(d_h, 2) - math.pow(d_wp_2, 2) - math.pow(get_a(2), 2))
    alpha_h_2 = math.atan2(pos_wp_22[0,0], pos_wp_22[1,0])
    theta_22 = alpha_h_1 - alpha_h_2
    
    # calculate position of wrist point in coordinate system 3 for theta_21 and theta_22
    return


"""
@type theta_4: angle in radiant
@param theta_4: joint angle of joint 4
@type theta_5: angle in radiant
@param theta_5: joint angle of joint 5
@type theta_6: angle in radiant
@param theta_6: joint angle of joint 6
@return: transformation matrix from 3 to 6
"""
def wrist_orientation(theta_4, theta_5, theta_6):
    c4 = math.cos(theta_4)
    c5 = math.cos(theta_5)
    c6 = math.cos(theta_6)
    s4 = math.sin(theta_4)
    s5 = math.sin(theta_5)
    s6 = math.sin(theta_5)
    return np.matrix([[ c4*c5*c6-s4*s6,-c4*c5*s6-s4*c6, c4*s5],
                      [ s4*c5*c6+c4*s6,-s4*c5*s6+c4*c6, s4*s5],
                      [         -s5*c6,          s5*s6,    c5]])
