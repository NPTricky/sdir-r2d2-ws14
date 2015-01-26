from openravepy import *
from scipy import spatial
import igraph as ig
import MotionFunctions as mf
import Kinematics as kin
import numpy as np
import random

# general hints
# - why? saves a lot of inverse kinematic calculation time
# - use inverse kinematics for the obstacles to transform them into c space

# influenced by the robot (usually 6 for 1 configuration or 6 angles)
_CFG_LEN = 6
_STATE_LEN = 6
# 1/100's chance to generate goal state
_GENERATE_GOAL_DIVISOR = 100
# sampling interval multiplier of in between configurations on collision check
# (higher == higher precision)
_RRT_PRECISION_COLLISION = 2
# sampling interval multiplier of interpolation configurations on nearest
# neighbor search (higher == higher precision)
_RRT_PRECISION_NEIGHBOR = 1
# distance not interpolation or collision check worthy
_EPS = 1e-6



# input:
# - state:
# return the configuration of the given state
def get_cfg(state):
    assert(len(state) == _STATE_LEN)
    return state[:_CFG_LEN]



# input:
# - cfg:
# create a state from a given configuration and velocity
def create_state(cfg):
    assert(len(cfg) == _CFG_LEN)
    return np.array(cfg)



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



# input:
# - cfg_init:
# - cfg_goal:
# - t:
# interpolate between two configurations with parameter t
def lerp(cfg_init, cfg_goal, t):
    assert(0 <= t and t <= 1) # try not to extrapolate
    return (1 - t) * cfg_init + t * cfg_goal



# input:
# - cfg_init:
# - cfg_goal:
# - precision:
# apply the precision parameters
def lerp_range(cfg_init, cfg_goal, precision):
    max_i = precision * max(np.linalg.norm(cfg_goal - cfg_init),1)
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
    # check for inlier's and set the corresponding report.contacts number
    robot.GetEnv().CheckCollision(robot,report)
    contact_count = len(report.contacts)
    contact_count = contact_count + 1 if robot.CheckSelfCollision() else contact_count
    robot.SetDOFValues(configuration_backup)
    return contact_count < 1



# input:
# - robot:
# - cfg_init:
# - cfg_goal:
# output:
# - if the path is collision free
def is_valid_path(robot, cfg_init, cfg_goal):
    if (cfg_goal - cfg_init).all() < mf._EPS:
        return True
    max_i, factor = lerp_range(cfg_init, cfg_goal, _RRT_PRECISION_COLLISION)
    for i in xrange(1,max_i-1):
        valid = is_valid(robot, lerp(cfg_init, cfg_goal, i * factor))
        if not valid:
            i = 0
            break
    return not i == 0



# input:
# - robot:
# - cfg_init:
# - cfg_goal:
# output:
# - possible configuration
def make_valid_path(robot, cfg_init, cfg_goal):
    if (cfg_goal - cfg_init).all() < mf._EPS:
        return cfg_init
    max_i, factor = lerp_range(cfg_init, cfg_goal, _RRT_PRECISION_COLLISION)
    for i in xrange(1,max_i-1):
        valid = is_valid(robot, lerp(cfg_init, cfg_goal, i * factor))
        if not valid:
            return lerp(cfg_init, cfg_goal, (i - 1) * factor)
    return cfg_goal



# input:
# - robot:
# output:
# - random state in the configuration space (velocity of 0.0)
# - generates only configurations without a collision (self intersection & contact)
def generate_random_state(robot):
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
    # container for the resulting configurations by interpolation of existing edges
    edge_interpolation = []
    # keep track of edge index
    edge_idx_list = []
    for a,b in graph.get_edgelist():
        cfg_init = graph.vs["configuration"][a]
        cfg_goal = graph.vs["configuration"][b]
        if (cfg_goal - cfg_init).all() < _EPS:
            # distance not interpolation worthy
            continue
        else:
            max_i, factor = lerp_range(cfg_init, cfg_goal, _RRT_PRECISION_NEIGHBOR)
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
        state_near = create_state(graph.vs["configuration"][search_idx])
        state_near_idx = search_idx
    else:
        # state is not already in the graph
        state_near_cfg = make_valid_path(robot, search_container[search_idx], get_cfg(state_goal))
        state_near = create_state(state_near_cfg)
        state_near_idx = insert_state(state_near, graph)
        insert_edge(edge_idx_list[search_idx-interpolation_idx_offset], state_near_idx, graph)
    return state_near,state_near_idx



# no heuristics or rules for a generated state
def generate_state_naive(robot, state_init, state_goal, delta_time):
    return state_goal



# input:
# - robot:
# - state_init:
# - state_goal:
# - delta_time:
# output:
# - new state generated by the given state_init and input
def generate_state(robot, state_init, state_goal, delta_time):
    # check for relevant distance
    cfg_init = get_cfg(state_init)
    cfg_goal = get_cfg(state_goal)
    if (cfg_goal - cfg_init).all() < mf._EPS:
        return state_goal
    # calculate difference
    difference = mf.difference_rel(cfg_init, cfg_goal)
    sign = np.sign(difference)
    # calculate limits (without signs)
    v_limit = np.sqrt(np.fabs(difference) * robot.GetDOFAccelerationLimits())
    velocity_limit = np.minimum(v_limit, robot.GetDOFVelocityLimits())
    times_acc = velocity_limit / robot.GetDOFAccelerationLimits()
    # calculate and apply delta
    delta = velocity_limit * (delta_time * 0.5 * times_acc)
    state_new = create_state(cfg_init + (delta * sign))
    return state_new



# input:
# - state: state to insert into the graph
# - graph: the corresponding graph
# output:
# - index of the new vertex
def insert_state(state, graph):
    graph.add_vertex()
    vertex = graph.vs[graph.vcount()-1]
    vertex["name"] = vertex.index
    vertex["configuration"] = get_cfg(state)
    return vertex.index



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
def generate_rt(robot, vertex_count, delta_time):
    # degrees of freedom
    _CFG_LEN = robot.GetDOF()
    
    # create initial & goal state for the rrt algorithm
    state_init = create_state(robot.GetDOFValues())
    _STATE_LEN = len(state_init)
    
    # create graph structure with initial state
    g = ig.Graph()
    insert_state(state_init,g)
    
    # entire rt generation algorithm as in [Lav98c]
    for i in range(1, vertex_count):
        state_random = generate_random_state(robot)
        state_near,state_near_idx = find_nearest_neighbor(robot, state_random, g)
        state_new = generate_state(robot, state_near, state_random, delta_time)
        state_new_idx = insert_state(state_new, g)
        # add edge and assign distance as an attribute
        edge_idx = insert_edge(state_near_idx, state_new_idx, g)
        g.es[edge_idx]["weight"] = np.linalg.norm(get_cfg(state_new) - get_cfg(state_near))
    return g



# input;
# - robot:
# - goal_cfg:
# public interface of the rapidly-exploring random tree algorithm
def rrt(robot, goal_cfg):
    vertex_count = 250
    delta_time = 1
    g = generate_rt(robot, vertex_count, delta_time)
    
    # add goal to the graph
    
    goal_idx = vertex_count - 1
    assert(goal_idx != 0)
    shortest_paths = g.get_all_shortest_paths(0, goal_idx)
    print shortest_paths
    plot_igraph(g, g.layout_kamada_kawai())

    #vertex_goal = g.vs.select(lambda vertex: (vertex["configuration"] == goal_cfg).all()) if np.array_equal(state_new, state_goal) else None
    #goal_idx = vertex_goal[0].index
    