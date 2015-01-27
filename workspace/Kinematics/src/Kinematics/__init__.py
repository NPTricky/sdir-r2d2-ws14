from openravepy import *
import Kinematics as kin
import MotionFunctions as mf
import RRT as rrt
import numpy as np
import sys
import socket

SERVER_IP = '127.0.0.1'
SERVER_PORT = 54321

_DEBUG_DRAW = []

# handles the data transfer between openrave (server) and the GUI (client)
def dataTransfer():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((SERVER_IP,SERVER_PORT))

    try:
        while True:
            recv_data, addr = s.recvfrom(2048)
            if not recv_data:
                break
            
            # parse received data and get the string that should be sent back to the GUI
            send = handleData(recv_data)
            
            # send new informations to the GUI for updating purposes
            s.sendto(send,addr)
    finally:
        s.close()



# handles the data received from the GUI and sets up data for sending
def handleData(data):
    # split data string
    data_arr = data.split("#")
    
    # check if GUI requests the current robot axis values as well as current orientation and position 
    if data_arr[0] == 'GET':
        # prefix for parsing
        prefix = "VAL#"
        # get Axis values
        axis_arr = robot.GetDOFValues()
        # convert to string
        axis_values = str(axis_arr[0])+";"+str(axis_arr[1])+";"+str(axis_arr[2])+";"+str(axis_arr[3])+";"+str(axis_arr[4])+";"+str(axis_arr[5])+'#'
        
        # adding dummy values for orientation and position (you need to compute the values)
        T = kin.forward(robot.GetDOFValues())
        
        pose = kin.get_pose_from( T )
        cart_values = str( str(pose[0]) + ";" + str(pose[1]) + ";" + str(pose[2]) + ";" + 
                           str(pose[3]) + ";" + str(pose[4]) + ";" + str(pose[5]))  
        
        return prefix+axis_values+cart_values
    
    # check if the robot should be moved 
    elif data_arr[0] == 'MOV':
        # get values
        values = data_arr[1].split(';')
        # convert from string to float and save in numpy array
        target = np.array([float(values[0]), float(values[1]), float(values[2]), float(values[3]), float(values[4]), float(values[5])])
                
        # get the motion type
        motion_type = data_arr[2]
                
        # get trajectory
        trajectory = mf.PTPtoConfiguration(robot, target, motion_type)
        # move robot
        mf.Move(robot, trajectory)
        
        # send new information about the robot's axis values, position and orientation to the GUI for updating purpose
        # prefix for parsing
        prefix = "VAL#"
        # get Axis values
        axis_arr = robot.GetDOFValues()
        # convert to string
        axis_values = str(axis_arr[0])+";"+str(axis_arr[1])+";"+str(axis_arr[2])+";"+str(axis_arr[3])+";"+str(axis_arr[4])+";"+str(axis_arr[5])+'#'
        
        # adding dummy values for orientation and position (you need to compute the values)
        T = kin.forward(robot.GetDOFValues())
        
        #_DEBUG_DRAW.append(misc.DrawAxes(robot.GetEnv(), T, 0.5, 2))
        
        pose = kin.get_pose_from( T )
        cart_values = str( str(pose[0]) + ";" + str(pose[1]) + ";" + str(pose[2]) + ";" + 
                           str(pose[3]) + ";" + str(pose[4]) + ";" + str(pose[5]))  
                
        return prefix+axis_values+cart_values
    
    # check if inverse kinematics should be calculated
    if data_arr[0] == "CAL":
        # get string with values
        values = data_arr[1].split(';')
        
        # calculate inverse kinematic solution
        pose = [ float(values[0]), float(values[1]), float(values[2]), 
                 float(values[3]), float(values[4]), float(values[5]) ]
        
        I = kin.inverse( pose, robot.GetDOFValues() )        
        confs = mf.get_possible_inverse_solution(robot, I)
        
        print "get_best_inverse_angles_solution" ,mf.get_nearest_angle_solution(robot.GetDOFValues(), confs)
        
        # send the (multiple) solutions to the GUI
        # prefix for parsing
        prefix = "INK#"
        ik_values = ""
        
        if 0 < len(confs):
            for i in xrange(0, len(confs)):
                for j in xrange(0, len(confs[i])):
                    ik_values += str( confs[i][j] )  + str(" " if j < len(confs[i]) - 1 else "")
                ik_values += str(";" if i < len(confs) - 1 else "")
        else:
            return prefix
        return prefix+ik_values
    
if __name__ == "__main__":
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)
    #np.seterr(divide='ignore') # ignore division by zero
    
    RaveSetDebugLevel(DebugLevel.Verbose)
    misc.InitOpenRAVELogging()
    
    # setting up the operave environment
    env = Environment() # create openrave environment
    env.SetViewer('qtcoin') # attach viewer (optional)
    env.Load('../../MyData/MyEnvironment/MyEnv.xml') # load a simple scene
    robot = env.GetRobots()[0] # get the first robot

    print "DH (OpenRave):\n"+str(planningutils.GetDHParameters(robot))
    
    m = robot.GetManipulators()[0]
    print "End Effector (OpenRave):\n"+str(m.GetEndEffectorTransform())
    
    #H = []
    #H.append(misc.DrawAxes(robot.GetEnv(), m.GetEndEffectorTransform(), 0.5, 2))
    #H.append(misc.DrawAxes(robot.GetEnv(), m.GetBase().GetTransform(), 0.5, 2))

    #T = kin.forward(robot)
    #angles = kin.extract_euler_angles_from(T)
    #print angles[0]
    #print "End Effector:\n"+str(T)

    #print  kin.get_pose_from(T)
    #I = kin.inverse(kin.get_pose_from(T))
    #print I
    
    #mf.PTPtoConfiguration(robot, [1.2,-1.8,0.55,0.2,1.2,-3.2], 'async')

    #configList = [[0,0,0,0,90,0], [90,0,0,0,0,0]] 
    #config = kin.selectConfiguration(configList)
    
    #rrt.rrt(robot, [1.2,-1.8,0.55,0.2,1.2,-3.2])
    
    dataTransfer()
