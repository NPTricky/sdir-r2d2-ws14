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

def extract_euler_angles_from(matrix):
    beta = math.atan2(-matrix[2,0], math.sqrt(math.pow(matrix[0,0], 2) + math.pow(matrix[1,0], 2))) 
    alpha = math.atan2(matrix[1,0]/math.cos(beta), matrix[0,0]/math.cos(beta))
    gamma = math.atan2(matrix[2,1]/math.cos(beta), matrix[2,2]/math.cos(beta))
    print "Alpha: "+str(mp.degrees(alpha))+", Beta: "+str(mp.degrees(beta))+", Gamma: "+str(mp.degrees(gamma))
    return np.array((alpha, beta, gamma))

def get_pose(matrix):
    euler_angle = extract_euler_angles_from(matrix)
    return np.array(((get_x(matrix),get_y(matrix),get_z(matrix),euler_angle[0], euler_angle[1], euler_angle[2])))

"""
@type robot: model of the robot
@param robot: robot instance
@return: pose of end effector in homogeneous coordinates
"""
def forward(robot, ):

    if len(robot.GetManipulators()) > 1:
        for m in robot.GetManipulators():
            print "ERROR"
            print " - Manipulator: \""+m.GetName()+"\" - Base: \""+m.GetBase().GetName()+"\" - End Effector: \""+m.GetEndEffector().GetName()+"\""
        return

    # transformation matrix of the base as a starting point       
    A = _BASE
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), A, 0.5, 2))
    
    # apply the denavit-hartenberg parameters for homogeneous transformation
    for i in range(0, robot.GetDOF()):
        A = A * homogeneous_transformation_from(robot.GetDOFValues(), i)
        _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), A, 0.5, 2))
    
    # for debug purposes without tool
    # transform the tool into the coordinate system of the base
    #A = A * robot.GetManipulators()[0].GetEndEffector().GetTransform()
    #A = A * _TOOL
    
    return A
    
""" Homogeneous transformation from joint i to i+1

@type angels: angels of the joints
@param angles: angles in radiant
@param     i: joint index
"""
def homogeneous_transformation_from( angels, i):
    return homogeneous_transformation(get_alpha(i), get_d(i), get_a(i), get_theta(i) - angels[i])

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
    ca = math.cos(alpha)
    ct = math.cos(theta)
    sa = math.sin(alpha)
    st = math.sin(theta)
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
    return np.rad2deg(alpha), np.rad2deg(beta), np.rad2deg(gamma)

"""
@type robot: model of the robot
@param robot: robot instance
@type T: homogeneous coordinates
@param T: pose of end effector
@rtype: joint coordinates
@return: pose of end effector
"""
def inverse(robot,T):
    
    # calculate position of wrist point in coordinate system 1 for theta_11 and theta_12
    pos_wp_0 = T * _TOOL
    print "Pos WP 0:\n" + str(pos_wp_0)
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), pos_wp_0, 0.5, 2))

    # theta 11
    theta_11 = math.atan2( get_y(pos_wp_0), get_x(pos_wp_0))
    
    # theta 21
    theta_12 = theta_11 
    theta_12 += np.pi if theta_11 < 0 else -np.pi

    print "Theta 11:\n"+str(mp.degrees(theta_11))
    print "Theta 12:\n"+str(mp.degrees(theta_12))
        
    # calculate position of wrist point in coordinate system 2 for theta_21 and theta_22
    pos_wp_21 = np.linalg.inv(homogeneous_transformation_from(np.array((theta_11,0,0,0,0,0)),0)) * pos_wp_0
    print "Pos WP 21:\n" + str(pos_wp_21)
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from(np.array((theta_11,0,0,0,0,0)),0) * pos_wp_21, 0.5, 2))
 
    pos_wp_22 = np.linalg.inv(homogeneous_transformation_from(np.array((theta_12,0,0,0,0,0)),0)) * pos_wp_0
    print "Pos WP 22:\n" + str(pos_wp_22)
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from(np.array((theta_12,0,0,0,0,0)),0) * pos_wp_22, 0.5, 2))
    
    # theta_21 and theta_22
    d_h = math.sqrt( math.pow( get_a(2), 2) + math.pow( get_d(3), 2))
    
    d_wp2 = math.sqrt( math.pow( get_x(pos_wp_21), 2) + math.pow( get_y(pos_wp_21), 2))
    print "d_wp2:\n" + str(d_wp2)
    alpha_h1 = mp.acos( ( -math.pow( d_h, 2) + math.pow( d_wp2, 2) + math.pow( get_a(1), 2)) / ( 2 * d_wp2 * get_a(1)))
    alpha_h2 = math.atan2( get_y(pos_wp_21), get_x(pos_wp_21))
    
    theta_21_1 = alpha_h2 + alpha_h1
    theta_22_1 = alpha_h2 - alpha_h1
    print "Theta 21 by Theta 11:\n" + str(mp.degrees(theta_21_1)) 
    print "Theta 22 by Theta 11:\n" + str(mp.degrees(theta_22_1))
	
    d_wp2 = math.sqrt( math.pow( get_x(pos_wp_22), 2) + math.pow( get_y(pos_wp_22), 2))
    print "d_wp2:\n" + str(d_wp2)
    alpha_h1 = mp.acos( ( -math.pow( d_h, 2) + math.pow( d_wp2, 2) + math.pow( get_a(1), 2)) / ( 2 * d_wp2 * get_a(1)))
    alpha_h2 = math.atan2( get_y(pos_wp_22), get_x(pos_wp_22))
    
    theta_21_2 = alpha_h2 + alpha_h1
    theta_22_2 = alpha_h2 - alpha_h1
    
    print "Theta 21 by Theta 12:\n" + str(mp.degrees(theta_21_2)) 
    print "Theta 22 by Theta 12:\n" + str(mp.degrees(theta_22_2))
    
    
    # calculate position of wrist point in coordinate system 3 for theta_31 and theta_32
    pos_wp_31 = np.linalg.inv(homogeneous_transformation_from(np.array((theta_11,theta_21_1,0,0,0,0)),1)) * pos_wp_21
    print "Pos WP 31:\n" + str(pos_wp_31)
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from(np.array((theta_11,0,0,0,0,0)),0) * homogeneous_transformation_from(np.array((theta_11,theta_21_1- np.pi/2,0,0,0,0)),1) * pos_wp_31, 0.5, 2))
    
    pos_wp_32 = np.linalg.inv( homogeneous_transformation_from(np.array((theta_11,theta_22_1,0,0,0,0)),1)) * pos_wp_21
    print "Pos WP 32:\n" + str(pos_wp_32)
    _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from(np.array((theta_11,0,0,0,0,0)),0) * homogeneous_transformation_from(np.array((theta_11,theta_22_1- np.pi/2,0,0,0,0)),1) * pos_wp_32, 0.5, 2))
    
    #pos_wp_33 = np.linalg.inv( homogeneous_transformation_from(np.array((theta_12,theta_21_2,0,0,0,0)),1)) * pos_wp_22
    #print "Pos WP 33:\n" + str(pos_wp_33)
    #   _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from(np.array((theta_12,0,0,0,0,0)),0) * homogeneous_transformation_from(np.array((theta_12,theta_21_2- np.pi/2,0,0,0,0)),1) * pos_wp_33, 0.5, 2))
    
    #pos_wp_34 = np.linalg.inv(homogeneous_transformation_from(np.array((theta_12,theta_22_2- np.pi/2,0,0,0,0)),1)) * pos_wp_22
    #print "Pos WP 34:\n" + str(pos_wp_34)
    #   _DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), homogeneous_transformation_from(np.array((theta_12,0,0,0,0,0)),0) * homogeneous_transformation_from(np.array((theta_12,theta_22_2- np.pi/2,0,0,0,0)),1) * pos_wp_34, 0.5, 2))
    
    # theta_31 and theta_32
    beta_2 = math.atan2( get_a(2), get_d(3))
    
    # theta_31
    beta_1 = math.atan2( get_y(pos_wp_31), get_x(pos_wp_31))
    theta_31 = beta_1 + beta_2
    
    # theta_32
    beta_1 = math.atan2( get_y(pos_wp_32), get_x(pos_wp_32))
    theta_32 = beta_1 + beta_2
    
    # theta_33
    #beta_1 = math.atan2( get_y(pos_wp_33), get_x(pos_wp_33))
    theta_33 = beta_1 + beta_2
    
    # theta_34
    #beta_1 = math.atan2( get_y(pos_wp_34), get_x(pos_wp_34))
    theta_34 = beta_1 + beta_2
    
    print "Theta 31:\n" + str(mp.degrees(theta_31))
    print "Theta 32:\n" + str(mp.degrees(theta_32))
    print "Theta 33:\n" + str(mp.degrees(theta_33))
    print "Theta 34:\n" + str(mp.degrees(theta_34))
      
    # calculate position of wrist orientation in coordinate system 3 to 6 for theta_4, theta_5 and theta_6
    
    T_0_3 = _BASE
    for i in range(0,2):
        T_0_3 = T_0_3 * homogeneous_transformation_from(np.array((0,0,0,0,0,0)),i)
                
    T_3_6 = np.linalg.inv(T_0_3) * T * np.linalg.inv(_TOOL)
    
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
