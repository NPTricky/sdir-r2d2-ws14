"""
@type robot: model of the robot
@param robot: robot instance
@return: xxx in yyy
"""
def forward(robot):
    vecAcceleration = robot.GetDOFAccelerationLimits()
    vecVelocity = robot.GetDOFVelocityLimits()
    vecTorque = robot.GetDOFTorqueLimits()
    vecvecLimits = robot.GetDOFLimits()
    print vecAcceleration
    print vecVelocity
    print vecTorque
    for vecLimits in vecvecLimits:
        print vecLimits
        
    return

"""
@type robot: model of the robot
@param robot: robot instance
@type pose: homogeneous coordinates
@param pose: end-effector pose
@return: end-effector pose in joint coordinates
"""
def inverse(robot,pose):
    return
