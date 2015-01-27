from openravepy import *
import numpy as np
import Kinematics as kin
import RRT as rrt

def createEnvironment():
     # setting up the operave environment
    env = Environment() # create openrave environment
    env.SetViewer('qtcoin') # attach viewer (optional)
    env.Load('../../MyData/MyEnvironment/MyEnv.xml') # load a simple scene
    robot = env.GetRobots()[0]    
    valid = False
    while not valid:
        with env:
            noOfGeom = np.random.randint(1,11,1)        
            geomArray = np.empty([0,noOfGeom])
        
            print 'Number of GeomObjects: ', noOfGeom
        
            for i in range(0, noOfGeom):
                geomType = np.random.randint(0,2,1)
                print 'GeomType from %d: %d' % (i, geomType)
                geom = setSettingsOfGeom(geomType[0], robot)
                geomArray = np.append(geomArray, geom)      
        
        
            body = RaveCreateKinBody(env, '')
            body.SetName('Body')
    
            body.InitFromGeometries(geomArray)
        
            env.Add(body, True)
            valid = rrt.is_valid(robot, robot.GetDOFValues())
        
    return env

def createFixEnvironment():
    # setting up the operave environment
    env = Environment() # create openrave environment
    env.SetViewer('qtcoin') # attach viewer (optional)
    env.Load('../../MyData/MyEnvironment/MyEnv.xml') # load a simple scene
    
    with env:
        geom = KinBody.Link.GeometryInfo()
        # position in room
        geom._t[0,3] = 1.5
        geom._t[1,3] = 0.5
        geom._t[2,3] = 1.5
    
        geom._type = KinBody.Link.GeomType.Box
        geom._vGeomData = [0.1, 0.1, 1]
    
        # set last information 
        geom._bVisible = True
        geom._fTransparency = 0
        geom._vDiffuseColor = [1,0,1]
        
    body = RaveCreateKinBody(env, '')
    body.SetName('Body')
    
    body.InitFromGeometries([geom])
        
    env.Add(body, True)
    
    return env

def getRandFloat(min, max, size):
    
    return min + np.random.ranf(size) * (max - min)

def setSettingsOfGeom(type, robot):
    geom = KinBody.Link.GeometryInfo()
    
    # position in room
    geom._t[0,3] = getRandFloat(-2,2,1)[0]
    geom._t[1,3] = getRandFloat(-2,2,1)[0]
    geom._t[2,3] = getRandFloat(0,3,1)[0]
    
    # size and color from geom object
    geomSize = getRandFloat(0,0.4,3)
    geomColor = getRandFloat(0,1,3)
    
    # set geom type (0 = box, 1 = sphere)
    if type == 0:
        geom._type = KinBody.Link.GeomType.Box
        geom._vGeomData = [geomSize[0], geomSize[1], geomSize[2]]
    else:
        geom._type = KinBody.Link.GeomType.Sphere
        geom._vGeomData = [geomSize[0]]
    
    # set last information 
    geom._bVisible = True
    geom._fTransparency = 0
    geom._vDiffuseColor = [geomColor[0],geomColor[1],geomColor[2]]
    
    return geom