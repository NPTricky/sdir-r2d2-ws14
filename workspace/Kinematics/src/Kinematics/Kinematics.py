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
@type pose: position and rotation of a point
@param pose: point in robot coordinate system were move the robot
@rtype: configuration of angles, angles in radiant
@return: configuration for each joint of the robot
"""
def inverse(pose):
    """
    # calculate position of wrist point in coordinate system 0 and theta0
    """
    theta0_0, theta0_1, P_wp0 = determin_theta0a_theta0b_and_wp0(pose)
        
    """    
    # calculate position of wrist point in coordinate system 1 and  theta1
    """
    theta1_00, theta1_01, P_wp1_0 = determin_theta1a_theta1b_and_wp1(theta0_0, P_wp0)
    theta1_10, theta1_11, P_wp1_1 = determin_theta1a_theta1b_and_wp1(theta0_1, P_wp0)  
    
    """
    # calculate the position of wrist point in coordinate system 2 and theta2
    """
    theta2_00, P_wp2_00 = determin_theta2_and_wp2( theta1_00, P_wp1_0 )
    theta2_01, P_wp2_01 = determin_theta2_and_wp2( theta1_01, P_wp1_0 )
    theta2_10, P_wp2_10 = determin_theta2_and_wp2( theta1_10, P_wp1_1 )
    theta2_11, P_wp2_11 = determin_theta2_and_wp2( theta1_11, P_wp1_1 )
    
    """
    # calculate of theta_3, theta_4 and theta_5
    """
    theta3_000, theta3_001, theta4_000, theta4_001, theta5_000, theta5_001 = determin_theta3a_theta3b_theta4a_theta5b_theta5a_theta5b(theta0_0, theta1_00, theta2_00, pose)
    theta3_010, theta3_011, theta4_010, theta4_011, theta5_010, theta5_011 = determin_theta3a_theta3b_theta4a_theta5b_theta5a_theta5b(theta0_0, theta1_01, theta2_01, pose)
    theta3_100, theta3_101, theta4_100, theta4_101, theta5_100, theta5_101 = determin_theta3a_theta3b_theta4a_theta5b_theta5a_theta5b(theta0_1, theta1_10, theta2_10, pose)
    theta3_110, theta3_111, theta4_110, theta4_111, theta5_110, theta5_111 = determin_theta3a_theta3b_theta4a_theta5b_theta5a_theta5b(theta0_1, theta1_11, theta2_11, pose)
    
    
    """
    # create the configurations
    """
    configurations = []
    if not theta1_00 is None:
        configurations.append(np.array(((theta0_0,theta1_00,theta2_00, theta3_000, theta4_000, theta5_000))))
        configurations.append(np.array(((theta0_0,theta1_00,theta2_00, theta3_001, theta4_001, theta5_001))))
        configurations.append(np.array(((theta0_0,theta1_01,theta2_01, theta3_010, theta4_010, theta5_010))))
        configurations.append(np.array(((theta0_0,theta1_01,theta2_01, theta3_011, theta4_011, theta5_011))))
    
    if not theta1_10 is None:
        configurations.append(np.array(((theta0_1,theta1_10,theta2_10, theta3_100, theta4_100, theta5_100))))
        configurations.append(np.array(((theta0_1,theta1_10,theta2_10, theta3_101, theta4_101, theta5_101))))
        configurations.append(np.array(((theta0_1,theta1_11,theta2_11, theta3_110, theta4_110, theta5_110))))
        configurations.append(np.array(((theta0_1,theta1_11,theta2_11, theta3_111, theta4_111, theta5_111))))
            
    return configurations

"""
@type pose: position and rotation of a point
@param pose: point in robot coordinate system were move the robot
@rtype: three values; theta0a and theta0b in radiant and the wrist point in position and rotation of a point
@return: theta0a, theta0b of joint 0 and position of wrist point in coordinate system 0
"""
def determin_theta0a_theta0b_and_wp0(pose):

    # determin of the wrist point matrix for ks0
    P_wp0 = get_matrix_from( pose) * _TOOL
    
    # theta 0 / 1
    theta0a = math.atan2( -get_y( P_wp0), get_x( P_wp0))
   
    # theta 0 / 2
    theta0b = theta0a + np.pi if theta0a < 0 else theta0a - np.pi

    return theta0a, theta0b, P_wp0

"""
@type theta0: angle in radiant
@param theta0: angle of joint 0
@type wp0: position and rotation of a point
@param wp0: position of wrist point in coordinate system 0
@rtype: three values; theta1a and theta1b in radiant and the wrist point in position and rotation of a point
@return: theta1a, theta1b of joint 1 and position of wrist point in coordinate system 1
"""
def determin_theta1a_theta1b_and_wp1(theta0, wp0):
    
    if theta0 is None or wp0 is None:
        return None, None
    
    d_h = math.sqrt( math.pow( get_a(2), 2) + math.pow( get_d(3), 2))
  
    # wrist point matrix form ks1 for theta0_0
    P_wp1 = np.linalg.inv( homogeneous_transformation_from( theta0, 0)) * wp0
    
    # determin of theta0_00 and theta0_01 for theta0_0
    d_wp2 = math.sqrt( math.pow( get_x(P_wp1), 2) + math.pow( get_y(P_wp1), 2))

    # check, it is not possible to reach the wrist point
    if d_h + get_a(1) < d_wp2:
        return None, None, None

    alpha_h1 = mp.acos( ( -math.pow( d_h, 2) + math.pow( d_wp2, 2) + math.pow( get_a(1), 2)) / ( 2 * d_wp2 * get_a(1)))
    alpha_h2 = math.atan2( -get_x(P_wp1), -get_y(P_wp1))
    
    theta1a = alpha_h2 + alpha_h1
    theta1b = alpha_h2 - alpha_h1
   
    return theta1a, theta1b, P_wp1

"""
@type theta1: angle in radiant
@param theta1: angle of joint 1
@type wp0: position and rotation of a point
@param wp0: position of wrist point in coordinate system 1
@rtype: two values; theta2 in radiant and the wrist point in position and rotation of a point
@return: theta2 of joint 2 and position of wrist point in coordinate system 2
"""
def determin_theta2_and_wp2(theta1, wp1):
    
    if theta1 is None or wp1 is None:
        return None, None
    
    # wrist point matrix form ks2 for theta1
    P_wp2 = np.linalg.inv( homogeneous_transformation_from( theta1, 1)) * wp1
    
    # theta2
    beta_1 = math.atan2( get_x(P_wp2), get_y(P_wp2))
    beta_2 = math.atan2( get_a(2), get_d(3))
    
    theta2 = beta_1 - beta_2
    
    return theta2, P_wp2 


"""
@type theta0: angle in radiant
@param theta0: angle of joint 0
@type theta1: angle in radiant
@param theta1: angle of joint 1
@type theta2: angle in radiant
@param theta2: angle of joint 2
@rtype: six values; theta3a, theta3b, theta4a, theta4b, theta5a and theta5b in radiant 
@return: angles for joint 3 to 6
"""
def determin_theta3a_theta3b_theta4a_theta5b_theta5a_theta5b(theta0, theta1, theta2, pose):
    
    if theta0 is None or theta1 is None or theta2 is None:
        return None, None, None, None, None, None
    
    # calculate position of wrist orientation in coordinate system 3 to 6 for theta_4, theta_5 and theta_6
    T_0_1 = homogeneous_transformation_from(theta0,0)
    T_1_2 = homogeneous_transformation_from(theta1,1)
    T_2_3 = homogeneous_transformation_from(theta2,2)
                
    T_4_6 = np.linalg.inv( T_0_1 * T_1_2 * T_2_3 ) *  get_matrix_from( pose ) * np.linalg.inv( _TOOL )

    theta3a = math.atan2( T_4_6[1,2],  T_4_6[0,2])    
    theta4a = math.atan2(math.sqrt(math.pow(T_4_6[0,2], 2) + math.pow(T_4_6[1,2], 2)), T_4_6[2,2])
    theta5a = math.atan2( T_4_6[2,0], -T_4_6[2,1]) 

    theta3b = theta3a + np.pi
    theta4b = -theta4a
    theta5b = theta5a + np.pi    
    
    return theta3a, theta3b, theta4a, theta4b, theta5a, theta5b
    
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
       
        
#eine Liste mit 8 Konfiguration mit je 6 Theta
#loop ueber Liste: checken ob Singularity vorhanden ist
#waehle den aus der Liste der keine Singularitaet + kleinsten theta 1 (pos 0)
def selectConfiguration(configList):
    configListWithoutSing = []
    #WP = getWristPoint
    
    WP = _TOOL
    
    for config in configList:
        isSingularity = checkSingularity(config, WP)
        if not isSingularity:
            configListWithoutSing.append(config, WP)
       
    selectedConfig = [] 
    for config in configListWithoutSing:
        if not 'selectedConfig' in locals():
            selectedConfig = config
        elif selectedConfig[0] > config[0]: 
            selectedConfig = config           
    
    return selectedConfig

def checkSingularity(angles, wp):
    T4 = homogeneous_transformation_from(angles[3], 3)
    A = T4
    for i in range(4, 6):
        A = A * homogeneous_transformation_from(angles[i], i)
    T4to6 = A
        
    #evtl werden noch in der if else noch solution eingebaut (abhaengig von anderen test-daten)
    if isOverheadSingularity(wp) and isWristSingularity(T4to6):
        print str('Overhead and Wrist')
        return True
    elif isOverheadSingularity(wp):
        print str('Overhead')
        return True
    elif isWristSingularity(T4, T4to6):
        print str('Wrist')
        return True
    else:
        return False

def isOverheadSingularity(WP):
    print WP[0,3], WP[1,3]
    if WP[0,3] == 0 and WP[1,3] == 0:
       return True
       
    return False

def isWristSingularity(T4to6):
    print T4to6[0,3], T4to6[1,3]
    if T4to6[0,3] == 0 and T4to6[1,3] == 0:
       return True
       
    return False
"""
    v4 = np.array(T4[:,2], dtype=np.float)
    print v4
    v6 = np.array(T6[:,2], dtype=np.float)
    print v6
    
    v = v4/v6
    print v
    
    if v[0] == v[1] == v[2]:
        return True
     
    return False
""" 
