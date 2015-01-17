from openravepy import *
import igraph as ig
import MotionFunctions as mf
import Kinematics as kin
import numpy as np

# why rrt?
# saves a lot of inverse kinematic calculation time

# tipps
# use inverse kinematics for the obstacles to transform them into c space

# input:
# - robot: respective robot
# - vertex_count: number of vertices k
# - delta_time: incremental distance
# output:
# - rrt graph g
def generate_rrt(robot, vertex_count, delta_time):
    start_cfg = robot.GetDOFValues()
    angle_limits = robot.GetDOFLimits()
    
    print 'start_cfg: ',start_cfg
    
    velocity = 0
    state = np.append(start_cfg,velocity)
    
    print 'state:' ,state
    
    g = ig.Graph([(0,1), (0,2)])
    g.vs["config"] = [start_cfg,start_cfg,start_cfg]
    g.vs["name"] = ["Alice", "Bob", "Claire"]
    g.vs["label"] = g.vs["config"]
    
    #g.add_vertices(1)
    #g.vs[0]
    #print 'g.vs: ',g.vs[0].attributes()
    #print 'g.es: ',g.es[0].attributes()
    
    #g.add_edges()
    lay = g.layout_kamada_kawai()
    ig.plot(g, layout = lay)
    return g
