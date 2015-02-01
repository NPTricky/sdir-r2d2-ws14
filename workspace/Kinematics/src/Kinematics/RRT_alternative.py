from openravepy import *
import MotionFunctions as mf
import Kinematics as kin
import numpy as np
import sys
import math
import igraph as ig

import time 

def RapidlyExploringRandomTree(robot, start_state, target_state, motiontype):
    # clone of the environment
    env = robot.GetEnv().CloneSelf(CloningOptions.Bodies) #  | CloningOptions.RealControllers
    new_robot = env.GetRobots()[0]
    
    # check, if the target_state in a collision
    
    
    g = G(start_state)
    cfg = start_state
    
    i = 0
    while not (cfg == target_state).all():
        if  2 < i:
            return g, None
        
        g = RapidlyExploringDenseTree(g, new_robot, cfg, target_state, motiontype)
        neighbor = NearestNeighbor(g, target_state)
        cfg = StoppingConfiguration(new_robot, get_cfg(neighbor), target_state, motiontype)
                
        if (get_cfg(neighbor) != cfg).all():                
            g.add_vertex(cfg)
            g.add_edge(get_cfg(neighbor), cfg)
        
        
        i = i + 1
    neighbor = NearestNeighbor(g, target_state)
    shortest_paths = g.get_shortest_path(neighbor)
    
    env.Destroy()
    
    if shortest_paths is None:
        return None, None
    
    vertex = g.vertex()
    cfg_path = []
    for i in shortest_paths:
        cfg = vertex[i]
        cfg_path.append(cfg)
        
    return g, cfg_path
    
    
def RapidlyExploringDenseTree( g, robot, start_state, target_state, motiontype, iterations = 250):    
    for i in xrange(iterations):
        print i, "=================================================================", i
        
        # create a random configuration for the robot
        alpha = RandomState(robot)
        
        # lead the rrt to the target configuration
        if np.random.randint(1,10) == 1:
            alpha = target_state
        
        # find the nearest neighbor from the random configuration
        # to the configuration in our graph
        vn = NearestNeighbor(g, alpha)
                
        # check of collision from the nearest neighbor to the random configuration
        # and gives back by a collision the the point before the collision
        qs = StoppingConfiguration(robot, get_cfg(vn), alpha, motiontype, 2)
        
        # check of the nearest neighbor and the new configuration that they not equal 
        if (get_cfg(vn) != qs).all():                
            g.add_vertex(qs)
            g.add_edge(get_cfg(vn), qs)
            
        #if (qs == target_state).all():
        #    break
            
    return g

def RandomState(robot):
    lower,upper = robot.GetDOFLimits()
    angular_limits_difference = upper - lower
    
    # calculate a random configuration between the minimum angle and the maximum angle of the robot
    configuration_random = lower + np.random.ranf(len(lower)) * angular_limits_difference
    
    return configuration_random
    
def NearestNeighbor(g, state_cfg):    
    start_pose = kin.get_pose_from(kin.forward(state_cfg))
    
    min_dist = sys.maxsize
    min_state = state_cfg
    for cfg in g.vertex():
        target_pose = kin.get_pose_from(kin.forward(cfg))
        dist = mf.distance_rel(start_pose, target_pose)[0]
        
        if dist < min_dist:
            min_dist = dist
            min_state = cfg
    
    
    #step 1: suchen aller knoten in dem der naechste nachbar vorhanden ist
    #step 2: pruefen der distanze vom partner knoten der kante zur uebergbenen konfiguration, kuerzste wird wird weiter verwendent
    #step 3: berechnung der kuerzesten distanz zwischen nachbar und partnerknoten zur uebergbenen konfiguration
    #step 4: bestimmung der konfiguration zwischen nachbar und partnerknoten an der kuerzesten distanz zur uebergbenen konfiguration
    #step 5: splitten der kante, von nachbar zum schnitpunkt und vom schnitpunkt zum partnerknoten
    #step 6: rueckgabe des schnittpunktes
    vertexknot_1 = g.get_vertex(min_state)
    
    edge_split = ((get_name(vertexknot_1),get_name(vertexknot_1)))
    min_dist = sys.maxsize
    for edge in g.edges():
        if edge[0] == get_name(vertexknot_1):
            cfg = g.vertex()[edge[1]]
            dist = mf.distance_rel(start_pose, kin.get_pose_from(kin.forward(cfg)))[0]
        elif edge[1] == get_name(vertexknot_1):
            cfg = g.vertex()[edge[0]]
            dist = mf.distance_rel(start_pose, kin.get_pose_from(kin.forward(cfg)))[0]
        else:
            continue
        
        if dist < min_dist: 
            min_dist = dist
            min_state = cfg
            edge_split = edge
            
    vertexknot_2 = g.get_vertex(min_state)
        
    if (get_cfg(vertexknot_1) == get_cfg(vertexknot_2)).all():
        return vertexknot_1
    
    pose_knot1 = kin.get_pose_from(kin.forward(get_cfg(vertexknot_1)))
    pose_knot2 = kin.get_pose_from(kin.forward(get_cfg(vertexknot_2)))
    dist_a = mf.distance_rel(start_pose, pose_knot1)[0]
    dist_b = mf.distance_rel(start_pose, pose_knot2)[0]
    dist_c = np.amax(mf.distance_rel(pose_knot1,pose_knot2))
    
    #if dist_c == 0:
    #    return vertexknot_1
    
    dist_d = (np.power(dist_c,2) + np.power(dist_b,2) - np.power(dist_a,2)) / (2 * dist_c)
        
    if 1 < dist_d / dist_c:
        return vertexknot_1
    
    diff_cfg = get_cfg(vertexknot_2) - get_cfg(vertexknot_1)
    new_cfg = np.float64(get_cfg(vertexknot_1) + diff_cfg * (dist_d / dist_c))
    
    ret_vertix = g.add_vertex(new_cfg)
    g.graph.delete_edges(edge_split)
    
    if edge_split[0] == get_name(vertexknot_1):
        g.add_edge(new_cfg, get_cfg(vertexknot_2))
        g.add_edge(get_cfg(vertexknot_1), new_cfg)
    else:
        g.add_edge(new_cfg, get_cfg(vertexknot_1))
        g.add_edge(get_cfg(vertexknot_2),new_cfg)
         
    return ret_vertix

def StoppingConfiguration(robot, start_state, target_state, motiontype, delta_time = sys.maxsize ):
        
    # calculate the trajectory from the start configuration to the target configuration
    configuration = np.array((start_state, target_state))
    trajectory = mf.generate_trajectory_from_configurations(robot, configuration, motiontype)
    
    # collision check
    env = robot.GetEnv()
    env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Contacts)
    report = CollisionReport()
     
    # take the minimum end time when delta_time is gives
    time_step_end = min(len(trajectory) - 1, int(math.ceil( delta_time / mf._SAMPLE_RATE + 0.5 )))
    
    # check the trajectory of collision
    collision = False
    last_cfg = start_state
    for i in xrange(1, time_step_end, 4):
        robot.SetDOFValues(trajectory[i])
        
        for link in robot.GetLinks()[1:]:
            collision=env.CheckCollision(link,report=report)
            if len(report.contacts) > 0:
                collision = True
                break
        
        # if collision, return the last possible configuration
        if collision or robot.CheckSelfCollision():
            return last_cfg
        
        last_cfg = trajectory[i]        
        
    return trajectory[time_step_end]

    
def get_cfg(vertex):
    return vertex[1]

def get_name(vertex):
    return vertex[0]


class G:
    def __init__(self, state):
        self.graph = ig.Graph()
        self.add_vertex(state)
    
    def add_vertex(self, state):
        if not self.get_vertex(state) is None:
            return self.get_vertex(state)
        
        self.graph.add_vertex()
        vertex = self.graph.vs[self.graph.vcount()-1]
        vertex["name"] = vertex.index
        vertex["configuration"] = state        
        return vertex.index, state
    
    def vertex(self):
        return self.graph.vs["configuration"]
    
    def get_vertex(self, state):
        res = self.graph.vs.select(lambda vertex: (vertex["configuration"] == state).all())
        if 0 < len(res):
            return res["name"][0], res["configuration"][0]
        return None
    
    def add_edge(self, start_state, target_state):
        start = self.get_vertex(start_state)
        if start is None: 
            return None
        
        target = self.get_vertex(target_state)
        if target is None: 
            return None
        
        self.graph.add_edge(start[0], target[0])
        idx = self.graph.ecount() - 1
        return idx
            
    def edges(self):
        return self.graph.get_edgelist()
    
    def get_shortest_path(self, vertexknot):
        res = self.graph.get_all_shortest_paths(0, get_name(vertexknot))
        
        if 0 < len(res[0]):
            return res[0]
        return None
    
    def graph(self):
        return self.graph
    
    def printGraph(self, env):
        
        plot_igraph(self.graph, self.graph.layout_kamada_kawai())
        
        mf._DEBUG_DRAW = []
        for edge in self.edges():
            print "edge", edge
            
            start_cfg = self.graph.vs["configuration"][edge[0]]
            start_pose = kin.get_pose_from(kin.forward(start_cfg))
        
            target_cfg = self.graph.vs["configuration"][edge[1]]
            target_pose = kin.get_pose_from(kin.forward(target_cfg))
            
            mf._DEBUG_DRAW.append(misc.DrawAxes(env, kin.forward(start_cfg), 0.1, 0.1))
            mf._DEBUG_DRAW.append(misc.DrawAxes(env, kin.forward(target_cfg), 0.1, 0.1))
            mf._DEBUG_DRAW.append(env.drawlinestrip(points=np.array(((start_pose[0],start_pose[1],start_pose[2]),(target_pose[0],target_pose[1],target_pose[2]))),
                                linewidth=1.0,
                                colors=np.array(((0,0,0),(0,0,0)))))
                                
            #time.sleep(3)
 
# input:
# - g: graph to plot
# - layout: plot layout algorithm
def plot_igraph(graph, layout):
    visual_style = {}
    visual_style["vertex_size"] = 20
    #visual_style["vertex_color"] = [color_dict[gender] for gender in g.vs["gender"]]
    visual_style["vertex_label"] = graph.vs["name"]
    #visual_style["edge_width"] = [1 + 2 * int(is_formal) for is_formal in g.es["is_formal"]]
    visual_style["layout"] = layout
    visual_style["bbox"] = (600,600)
    visual_style["margin"] = 20
    ig.plot(graph, **visual_style)
    return