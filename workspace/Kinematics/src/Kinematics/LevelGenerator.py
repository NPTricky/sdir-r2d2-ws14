from openravepy import *
import numpy as np
import Kinematics as kin
import RRT as rrt

def setup():
    # setting up the operave environment
    env = Environment() # create openrave environment
    env.SetViewer('qtcoin') # attach viewer (optional)
    
    # scene load after physics engine configuration
    env.Load('../../MyData/MyEnvironment/MyEnv.xml') # load a simple scene
    
    env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Contacts)
    
    return env
    
def insertBodies(env, bodies):
    body = RaveCreateKinBody(env, '')
    body.SetName('Body')
    body.InitFromGeometries(bodies)
    env.Add(body, True)

def createEnvironment():
    env = setup()
    
    robot = env.GetRobots()[0]    
    valid = False
    while not valid:
        with env:
            noOfGeom = np.random.randint(1,11)
            geomArray = np.empty([0,noOfGeom])
        
            print 'Number of GeomObjects: ', noOfGeom
        
            for i in range(0, noOfGeom):
                geomType = np.random.randint(0,2)
                print 'GeomType from %d: %d' % (i, geomType)
                geom = setSettingsOfGeom(geomType)
                geomArray = np.append(geomArray, geom)
            
            insertBodies(env, geomArray)
            
            valid = rrt.is_valid(robot, robot.GetDOFValues())
        
    return env

def createFixEnvironment():
    env = setup()
    
    # load a simple scene
    with env:
        geom = KinBody.Link.GeometryInfo()
        geom._type = KinBody.Link.GeomType.Box
        geom._t[0:3,3] = [1.5, 0.5, 1.8] # position in room
        geom._vGeomData = [0.1, 0.1, 1]
    
        # set last information 
        geom._bVisible = True
        geom._fTransparency = 0
        geom._vDiffuseColor = [1,0,1]
    
    insertBodies(env, [geom])
    
    with env:
        geom = KinBody.Link.GeometryInfo()
        geom._type = KinBody.Link.GeomType.Box
        geom._t[0:3,3] = [1.5, -0.5, 1.8] # position in room
        geom._vGeomData = [0.1, 0.1, 1]
    
        # set last information 
        geom._bVisible = True
        geom._fTransparency = 0
        geom._vDiffuseColor = [1,0,1]
    
    insertBodies(env, [geom])
    
    return env

def setSettingsOfGeom(type):
    geom = KinBody.Link.GeometryInfo()
    
    # position in room
    geom._t[0:3,3] = [np.random.uniform(-2,2), np.random.uniform(-2,2), np.random.uniform(0,3)] # position in room
    
    # size and color from geom object
    geomSize = np.random.uniform(0,0.4,3)
    geomColor = np.random.uniform(0,1,3)
    
    # set geom type (0 = box, 1 = cylinder, 2 = sphere)
    if type == 0:
        geom._type = KinBody.Link.GeomType.Box
        geom._vGeomData = [geomSize[0], geomSize[1], geomSize[2]]
    elif type == 1:
        geom._type = KinBody.Link.GeomType.Cylinder
        geom._vGeomData = [geomSize[0], geomSize[1]]
    elif type == 2:
        geom._type = KinBody.Link.GeomType.Sphere
        geom._vGeomData = [geomSize[0]]
    else:
        print "Unknown Geometry Type!"
    
    # set last information 
    geom._bVisible = True
    geom._fTransparency = 0.5
    geom._vDiffuseColor = [geomColor[0],geomColor[1],geomColor[2]]
    
    return geom
