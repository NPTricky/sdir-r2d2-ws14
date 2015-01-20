from openravepy import *
from scipy import spatial
import igraph as ig
import MotionFunctions as mf
import Kinematics as kin
import numpy as np
import random

_STATE_LEN = 7 # six angles and one velocity parameter (kinodynamic problem)
_GENERATE_GOAL_DIVISOR = 100 # 1/100's chance to generate goal state
_RRT_PRECISION = 1 # sampling interval of in between vertices (lower == higher precision)
_EPS = 1e-4

# return the configuration of the given state
def get_cfg(state):
    assert(len(state) == _STATE_LEN)
    return state[:6]
# return the velocity of the given state
def get_v(state):
    assert(len(state) == _STATE_LEN)
    return state[6]
# create a state from a given configuration and velocity
def create_state(cfg, v = 0.0):
    assert(len(cfg) == _STATE_LEN - 1)
    return np.append(cfg, v)

# general hints
# - why? saves a lot of inverse kinematic calculation time
# - use inverse kinematics for the obstacles to transform them into c space

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



# interpolate between two configurations with parameter t
def lerp(cfg_init, cfg_goal, t):
    assert(t <= 1) # try not to extrapolate
    return (1 - t) * cfg_init + t * cfg_goal



# apply the precision parameters
def lerp_range(cfg_init, cfg_goal):
    max_i = (1 / _RRT_PRECISION) * max(np.linalg.norm(cfg_goal - cfg_init),1)
    factor = 1 / max_i
    return int(max_i), factor



# input:
# - robot:
# - configuration: configuration to be checked
# output:
# - boolean whether configuration is valid (in free space)
def is_valid(robot, configuration):
    configuration_backup = robot.GetDOFValues()
    robot.SetDOFValues(configuration)
    report = CollisionReport()
    inlier = robot.GetEnv().CheckCollision(robot,report)
    contact_count = len(report.contacts)
    contact_count = contact_count + 1 if robot.CheckSelfCollision() else contact_count
    robot.SetDOFValues(configuration_backup)
    return contact_count < 1



# input:
# - robot:
# - cfg_init:
# - cfg_goal:
# output:
# - possible configuration
def make_valid(robot, cfg_init, cfg_goal):
    if (cfg_goal - cfg_init).all() < mf._EPS:
        return cfg_init
    max_i, factor = lerp_range(cfg_init, cfg_goal)
    for i in xrange(1,max_i-1):
        valid = is_valid(robot, lerp(cfg_init, cfg_goal, i * factor))
        if not valid:
            return lerp(cfg_init, cfg_goal, (i - 1) * factor)
    return cfg_goal



# input:
# - robot:
# - state_goal:
# output:
# - random state in the configuration space (velocity of 0.0)
# - generates only configurations without a collision (self intersection & contact)
def generate_random_state(robot, state_goal):
    if random.randint(1, _GENERATE_GOAL_DIVISOR) == 1:
        return state_goal
    lower,upper = robot.GetDOFLimits()
    angular_limits_difference = upper - lower
    valid = False
    while valid == False:
        configuration_random = lower + np.random.sample(len(lower)) * angular_limits_difference
        valid = is_valid(robot, configuration_random)
    return create_state(configuration_random)



# input:
# - robot:
# - state_goal:
# - graph:
# output:
# - the nearest configuration
# - the index of the vertex of the nearest configuration
# find the nearest neighbor of a (random) state about to be inserted into the graph
def find_nearest_neighbor(robot, state_goal, graph):
    # search structure creation (including interpolation of edges)
    edge_interpolation = []
    # keep track of edge index
    edge_idx = 0
    edge_idx_list = []
    for a,b in graph.get_edgelist():
        cfg_init = graph.vs["configuration"][a]
        cfg_goal = graph.vs["configuration"][b]
        if (cfg_goal - cfg_init).all() < _EPS:
            # distance not interpolation worthy
            continue
        else:
            max_i, factor = lerp_range(cfg_init, cfg_goal)
            for i in xrange(1,max_i-1):
                edge_interpolation.append(lerp(cfg_init, cfg_goal, i * factor))
                edge_idx_list.append(a)
    
    interpolation_idx_offset = len(graph.vs["configuration"])
    search_container = graph.vs["configuration"] + edge_interpolation
    search_structure = spatial.KDTree(search_container)
    # search for smallest (euclidean) distance between configurations (KNN with k = 1)
    distance, search_idx = search_structure.query(get_cfg(state_goal), 1);
    if search_idx < interpolation_idx_offset:
        # state is already in the graph
        state_near = create_state(graph.vs["configuration"][search_idx], graph.vs["velocity"][search_idx])
        state_near_idx = search_idx
    else:
        # state is not already in the graph
        state_near_cfg = make_valid(robot, search_container[search_idx], get_cfg(state_goal))
        state_near = create_state(state_near_cfg)
        state_near_idx = insert_state(state_near, graph)
        insert_edge(edge_idx_list[search_idx-interpolation_idx_offset], state_near_idx, graph)
    return state_near,state_near_idx



def generate_state_naive(state_init, state_goal, delta_time):
    return state_goal

# input:
# - state_init:
# - state_goal:
# - delta_time:
# output:
# - new state generated by the given state_init and input
def generate_state(state_init, state_goal, delta_time):
    #helpful?
    #difference = np.fabs(mf.difference_rel(start_cfg, target_cfg))
    #velocity_limit, acceleration_limit, times_acc, times_dec, times_end = mf.limits_and_times(robot, difference, 'F')
    #velocity_limit, acceleration_limit, times_acc, times_dec, times_end = mf.discretize(difference, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    state_new = np.zeros(_STATE_LEN)
    return state_new



# input:
# - state: state to insert into the graph
# - graph: the corresponding graph
# output:
# - index of the new vertex
def insert_state(state, graph):
    graph.add_vertex()
    idx = graph.vcount()-1
    vertex = graph.vs[idx]
    vertex["name"] = idx
    vertex["configuration"] = get_cfg(state)
    vertex["velocity"] = get_v(state)
    return idx



# input:
# - idx_from: index of vertex from
# - idx_to: index of vertex to
# - graph: the corresponding graph
# output:
# - index of the new edge
def insert_edge(idx_from, idx_to, graph):
    graph.add_edge(idx_from, idx_to)
    idx = graph.ecount() - 1
    return idx



# input:
# - robot: instance of the robot
# - vertex_count: number of vertices k
# - delta_time: incremental distance
# output:
# - rt graph g
def generate_rt(robot, target_cfg, vertex_count, delta_time):
    # create initial & goal state for the rrt algorithm
    state_init = create_state(robot.GetDOFValues())
    print 'state_init:',state_init,'- len:',len(state_init)
    state_goal = create_state(target_cfg)
    print 'state_goal:',state_goal,'- len:',len(state_goal)
    
    # create graph structure with initial state
    g = ig.Graph()
    state_init_idx = insert_state(state_init,g)
    
    # entire rt generation algorithm as in [Lav98c]
    for i in range(1, vertex_count):
        state_random = generate_random_state(robot, state_goal)
        state_near,state_near_idx = find_nearest_neighbor(robot, state_random, g)
        state_new = generate_state_naive(state_near, state_random, delta_time)
        state_new_idx = insert_state(state_new, g)
        # add edge and assign input_u as an attribute
        edge_idx = insert_edge(state_near_idx, state_new_idx, g)
        # g.es[edge_idx]["input_u"] = input_u
    
    plot_igraph(g, g.layout_kamada_kawai())

    return g



def rrt(robot, target_cfg):
    vertex_count = 100
    delta_time = None
    rt = generate_rt(robot, target_cfg, vertex_count, delta_time)


