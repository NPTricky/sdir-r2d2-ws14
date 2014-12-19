from openravepy import *
import numpy as np
from sympy.mpmath import *
import math

_DEBUG_DRAW = []

# team r2d2 (alpha, a, theta, d)
# alpha = rotation about the x-axis in radiant
#     a = translation along the x-axis in meter
# theta = rotation about the z-axis in radiant
#     d = translation along the z-axis in meter
_DH_KUKA_KR30L16_R2D2 = np.matrix([[ np.pi/2, 0.350,        0, 0.815],
                                   [       0, 1.200,  np.pi/2, 0.000],
                                   [-np.pi/2, 0.145,        0, 0.000],
                                   [ np.pi/2, 0.000,        0,-1.545],
                                   [-np.pi/2, 0.000,        0, 0.000],
                                   [   np.pi, 0.000,  np.pi/2,-0.158]])

_DH_KUKA_KR30L16_TEST = np.matrix([[-np.pi/2, 0.350,        0, 0.815],
                                   [       0, 1.200, -np.pi/2, 0.000],
                                   [-np.pi/2, 0.145,        0, 0.000],
                                   [ np.pi/2, 0.000,        0, 1.545],
                                   [-np.pi/2, 0.000,        0, 0.000],
                                   [       0, 0.000, -np.pi/2, 0.158]])

_DH_KUKA_KR30L16 = _DH_KUKA_KR30L16_TEST

def get_alpha(i):
    return _DH_KUKA_KR30L16[i,0]
def get_a(i):
    return _DH_KUKA_KR30L16[i,1]
def get_theta(i):
    return _DH_KUKA_KR30L16[i,2]
def get_d(i):
    return _DH_KUKA_KR30L16[i,3]

_BASE = np.matrix([[ 1, 0, 0, 0],
                   [ 0, 1, 0, 0],
                   [ 0, 0, 1, 0],
                   [ 0, 0, 0, 1]])

_TOOL = np.matrix([[ 1, 0, 0,  0],
                   [ 0, 1, 0,  0],
                   [ 0, 0, 1,  -_DH_KUKA_KR30L16[5,3]],
                   [ 0, 0, 0,  1]])


def get_x(matrix):
    return matrix[0,3]
def get_y(matrix):
    return matrix[1,3]
def get_z(matrix):
    return matrix[2,3]

""" 
@type matrix: homogeneous transformation matrix
@param matrix: matrix from a point or coordinate-system
@rtype: position and rotation of a point
@return: position and rotation of a point
"""
def get_pose_from(matrix):
    euler_angle = extract_euler_angles_from(matrix)
    return np.array(((get_x(matrix),get_y(matrix),get_z(matrix),euler_angle[0], euler_angle[1], euler_angle[2])))


""" 
@type pose: position and rotation of a point
@param pose: position and rotation of a point
@rtype: homogeneous transformation matrix
@return: matrix from a point or coordinate-system
"""    
def get_matrix_from(pose):
    ca = math.cos(pose[3])
    cb = math.cos(pose[4])
    cg = math.cos(pose[5])
    sa = math.sin(pose[3])
    sb = math.sin(pose[4])
    sg = math.sin(pose[5])
    return np.matrix([[ ca*cb, ca*sb*sg - sa*cg, ca*sb*cg + sa*sg, pose[0]],
                      [ sa*cb, sa*sb*sg + ca*cg, sa*sb*cg - ca*sg, pose[1]],
                      [   -sb,    cb*sg        ,    cb*cg        , pose[2]],
                      [     0,                0,                0,       1]])
    
    
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
    #_DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), A, 0.5, 2))
    
    # apply the denavit-hartenberg parameters for homogeneous transformation
    for i in range(0, robot.GetDOF()):
        A = A * homogeneous_transformation_from(robot.GetDOFValues()[i], i)
        #_DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), A, 0.5, 2))
    
    # for debug purposes without tool
    # transform the tool into the coordinate system of the base
    #A = A * robot.GetManipulators()[0].GetEndEffector().GetTransform()
    #A = A * _TOOL
    
    return A
    
""" Homogeneous transformation from joint i to i+1

@type angels: angel of the z-axis of an joint
@param angles: angles in radiant
@type i: i as integer
@param i: joint index
return: homogeneous coordinates
"""
def homogeneous_transformation_from( angle, i):
    return homogeneous_transformation(get_alpha(i), get_d(i), get_a(i), get_theta(i) - angle)

""" Homogeneous transformation by given parameters.
    A = T(d) * R(theta) * T(a) * R(alpha)
@type alpha: angle in radiant
@param alpha: rotation about the x-axis of the link 
@type d: meter
@param d: translation along the z-axis of the link
@type a: meter
@param a: translation along the x-axis of the link
@type theta: angle in radiant
@param theta: rotation about the z-axis of the link
@return: homogeneous coordinates
"""
def homogeneous_transformation(alpha, d, a, theta):
    ca = mp.cos(alpha)
    ct = mp.cos(theta)
    sa = mp.sin(alpha)
    st = mp.sin(theta)
    return np.matrix([[ct,-st*ca, st*sa, a*ct],
                      [st, ct*ca,-ct*sa, a*st],
                      [ 0,    sa,    ca,    d],
                      [ 0,     0,     0,    1]])
    
    
    
"""
@type matrix: transformation matrix
@param matrix: instance of the transformation matrix
@return: prints the values to console
"""
def extract_euler_angles_from(matrix):
    beta = math.atan2(-matrix[2,0], math.sqrt(math.pow(matrix[0,0], 2) + math.pow(matrix[1,0], 2)))    
    alpha = math.atan2(matrix[1,0]/math.cos(beta), matrix[0,0]/math.cos(beta))
    gamma = math.atan2(matrix[2,1]/math.cos(beta), matrix[2,2]/math.cos(beta))
    return alpha, beta, gamma

"""
@type robot: model of the robot
@param robot: robot instance
@type pose: position and rotation of a point
@param pose: point in robot coordinate system were move the robot
@rtype: configuration of angles
@return: configuration for each joint of the robot
"""
def inverse(robot, pose):
    """
    # calculate position of wrist point in coordinate system 0 for theta0
    """
    # determin of the wrist point matrix for ks0
    P_wp0 = get_matrix_from( pose) * _TOOL
    
    # theta 0 / 1
    theta0_0 = math.atan2( get_y( P_wp0), get_x( P_wp0))
   
    # theta 0 / 2
    theta0_1 = theta0_0 + np.pi if theta0_0 < 0 else theta0_0 - np.pi

    
    
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), P_wp0, 0.5, 2))

    #print "Pos WP 0:\n" + str(P_wp0)
    print "Theta 0 / 0: " + str(mp.degrees(theta0_0))
    print "Theta 0 / 1: " + str(mp.degrees(theta0_1))
    
    
    
    """    
    # calculate position of wrist point in coordinate system 1 for theta1
    """
    # d_h is the lenght from ks2 to ks4
    d_h = math.sqrt( math.pow( get_a(2), 2) + math.pow( get_d(3), 2))
  
    # wrist point matrix form ks1 for theta0_0
    P_wp1_0 = np.linalg.inv( homogeneous_transformation_from( theta0_0, 0)) * P_wp0
    
    # determin of theta0_00 and theta0_01 for theta0_0
    d_wp2 = math.sqrt( math.pow( get_x(P_wp1_0), 2) + math.pow( get_y(P_wp1_0), 2))

    alpha_h1 = mp.acos( ( -math.pow( d_h, 2) + math.pow( d_wp2, 2) + math.pow( get_a(1), 2)) / ( 2 * d_wp2 * get_a(1)))
    alpha_h2 = math.atan2( get_y(P_wp1_0), get_x(P_wp1_0))
    
    theta1_00 = alpha_h1 + alpha_h2 - np.pi/2
    theta1_01 = alpha_h1 - alpha_h2 - np.pi/2
    
    
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from( theta0_0, 0) * P_wp1_0, 0.5, 2))
 
    #print "math.atan2( get_y(P_wp1_0), get_x(P_wp1_0)):\n" + str(mp.degrees( math.atan2( get_y(P_wp1_0), get_x(P_wp1_0))) )
    #print "math.atan2( get_x(P_wp1_0), get_y(P_wp1_0)):\n" + str(mp.degrees( math.atan2( get_x(P_wp1_0), get_y(P_wp1_0))) )
 
    #print "Pos WP 1 / 0:\n" + str(P_wp1_0)
    #print "d_wp2:\n" + str(d_wp2)
    print "Theta 1 / 0  for Theta 0 / 0: " + str(mp.degrees(theta1_00)) 
    print "Theta 1 / 1  for Theta 0 / 0: " + str(mp.degrees(theta1_01))

    
    
    # wrist point matrix form ks1 for theta0_1
    P_wp1_1 = np.linalg.inv( homogeneous_transformation_from( theta0_1, 0)) * P_wp0
        
    # determin of theta1_10 and theta1_11 for theta0_1
    d_wp2 = math.sqrt( math.pow( get_x(P_wp1_1), 2) + math.pow( get_y(P_wp1_1), 2))
    
    alpha_h1 = mp.acos( ( -math.pow( d_h, 2) + math.pow( d_wp2, 2) + math.pow( get_a(1), 2)) / ( 2 * d_wp2 * get_a(1)))
    alpha_h2 = math.atan2( get_x(P_wp1_1), get_y(P_wp1_1))
    
    theta1_10 = alpha_h1 + alpha_h2 - np.pi
    theta1_11 = alpha_h1 - alpha_h2 - np.pi
    
    

    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from( theta0_1, 0) * P_wp1_1, 0.5, 2))
    
    #print "Pos WP 1 / 1:\n" + str(P_wp1_1)    
    #print "d_wp2:\n" + str(d_wp2)
    print "Theta 1 / 0  for Theta 0 / 1: " + str(mp.degrees(theta1_10)) 
    print "Theta 1 / 1  for Theta 0 / 1: " + str(mp.degrees(theta1_11))

    
    
    """
    # calculate position of wrist point in coordinate system 2 for theta2
    """
    beta_2 = math.atan2( get_a(2), get_d(3))
    
    # wrist point matrix form ks2 for theta1_00
    P_wp2_00 = np.linalg.inv( homogeneous_transformation_from( theta1_00, 1)) * P_wp1_0
    
    # theta2_00
    beta_1 = math.atan2( get_y(P_wp2_00), get_x(P_wp2_00))
    theta2_00 = beta_1 + beta_2 - np.pi/2
    
    
    
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from( theta0_0, 0) * homogeneous_transformation_from( theta1_00, 1) * P_wp2_00, 0.5, 2))
    #print "Pos WP 2 / 00:\n" + str(P_wp2_00)
    print "Theta 2 / 0  for Theta 1 / 0: " + str(mp.degrees(theta2_00)) 
    
    
        
    # wrist point matrix form ks2 for theta1_01
    P_wp2_01 = np.linalg.inv( homogeneous_transformation_from( theta1_01, 1)) * P_wp1_0
    
    # theta2_01
    beta_1 = math.atan2( get_y(P_wp2_01), get_x(P_wp2_01))
    theta2_01 = beta_1 + beta_2 - np.pi/2
    
    
    
    
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from( theta0_0, 0) * homogeneous_transformation_from( theta1_01, 1) * P_wp2_01, 0.5, 2))
    #print "Pos WP 2 / 00:\n" + str(P_wp2_00)
    print "Theta 2 / 1  for Theta 1 / 1: " + str(mp.degrees(theta2_01)) 
    
    
    
    # wrist point matrix form ks2 for theta1_10
    P_wp2_10 = np.linalg.inv( homogeneous_transformation_from( theta1_10, 1)) * P_wp1_1
    
    # theta2_10
    beta_1 = math.atan2( get_x(P_wp2_10), get_y(P_wp2_10))
    theta2_10 = beta_1 - beta_2 
    
    
    
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from( theta0_1, 0) * homogeneous_transformation_from( theta1_10, 1) * P_wp2_10, 0.5, 2))
    #print "Pos WP 2 / 00:\n" + str(P_wp2_10)
    print "Theta 2 / 0  for Theta 1 / 0: " + str(mp.degrees(theta2_10)) 
    
    
        
    # wrist point matrix form ks2 for theta1_11
    P_wp2_11 = np.linalg.inv( homogeneous_transformation_from( theta1_11, 1)) * P_wp1_1
    
    # theta2_11
    beta_1 = math.atan2( get_x(P_wp2_11), get_y(P_wp2_11))
    theta2_11 = beta_1 + beta_2
    
    
    
    
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from( theta0_1, 0) * homogeneous_transformation_from( theta1_11, 1) * P_wp2_11, 0.5, 2))
    #print "Pos WP 2 / 00:\n" + str(P_wp2_11)
    print "Theta 2 / 1  for Theta 1 / 1: " + str(mp.degrees(theta2_11)) 
    
    
       
          
    new_forward(robot, np.array((theta0_0,theta1_00, theta2_00,0,0,0)))
    new_forward(robot, np.array((theta0_0,theta1_01, theta2_01,0,0,0)))
    new_forward(robot, np.array((theta0_1,theta1_10, theta2_10,0,0,0)))
    #new_forward(robot, np.array((theta0_1,theta1_11, theta2_11,0,0,0)))
    
    
    # calculate position of wrist orientation in coordinate system 3 to 6 for theta_4, theta_5 and theta_6
    T_0_3 = _BASE
    for i in range(0,2):
        T_0_3 = T_0_3 * homogeneous_transformation_from(0,i)
                
    T_3_6 = np.linalg.inv(T_0_3) *  get_matrix_from( pose) * np.linalg.inv(_TOOL)
    
    theta_5 = math.atan2(math.sqrt(math.pow(T_3_6[0,2], 2) + math.pow(T_3_6[1,2], 2)), T_3_6[2,2])
    
    if theta_5 < 0:    
        theta_4 = math.atan2(-T_3_6[1,2], -T_3_6[0,2]) 
        theta_5 = math.atan2(-math.sqrt(math.pow(T_3_6[0,2], 2) + math.pow(T_3_6[1,2], 2)), T_3_6[2,2])
        theta_6 = math.atan2(-T_3_6[2,1],  T_3_6[2,0])
    else:
        theta_4 = math.atan2( T_3_6[1,2],  T_3_6[0,2]) 
        theta_6 = math.atan2( T_3_6[2,1], -T_3_6[2,0])                    
    
    
    print "Wrist Orientation:\n"+str(T_3_6)
    print "Theta 4:\n"+str(mp.degrees(theta_4))
    print "Theta 5:\n"+str(mp.degrees(theta_5))
    print "Theta 6:\n"+str(mp.degrees(theta_6))
    
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


def new_forward(robot, values):
    A = _BASE
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), A, 0.5, 2))
    
    # apply the denavit-hartenberg parameters for homogeneous transformation
    for i in range(0, robot.GetDOF()):
        A = A * homogeneous_transformation_from(values[i], i)
        _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), A, 0.5, 2))