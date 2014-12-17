from openravepy import *
import Kinematics as kin
import MotionFunctions as mf
import numpy as np
import sys
import socket

SERVER_IP = '127.0.0.1'
SERVER_PORT = 54321

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
        angles = kin.extract_euler_angles_from(T)
        cart_values = str(T[0,3]) + ";" + str(T[1,3]) + ";" + str(T[2,3]) + ";" + str(angles[0]) + ";" + str(angles[1]) + ";" + str(angles[2])
        
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
        trajectory = mf.PTPtoConfiguration(robot.GetDOFValues(), target, motion_type)
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
        #welche Matrix nehme ich? wie berechnen wir diese?
        angles = kin.extract_euler_angles_from(T)
        cart_values = str(T[0,3]) + ";" + str(T[1,3]) + ";" + str(T[2,3]) + ";" + str(angles[0]) + ";" + str(angles[1]) + ";" + str(angles[2])  
        
        return prefix+axis_values+cart_values
    
    # check if inverse kinematics should be calculated
    if data_arr[0] == "CAL":
        # get string with values
        values = data_arr[1].split(';')
        
        # calculate inverse kinematic solution
        
        # send the (multiple) solutions to the GUI
        # prefix for parsing
        prefix = "INK#"
        
        # adding dummy values (you need to replace them with the solutions)
        ik_values = "0;0;0;0;0;0"
        
        return prefix+ik_values
    
    
if __name__ == "__main__":
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)
    
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

    T = kin.forward(robot)
    angles = kin.extract_euler_angles_from(T)
    print angles[0]
    print "End Effector:\n"+str(T)
    I = kin.inverse(robot, T)
    #print I
    
    dataTransfer()
    
    
