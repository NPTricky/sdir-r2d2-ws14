from openravepy import *
from scipy import spatial
import igraph as ig
import MotionFunctions as mf
import Kinematics as kin
import numpy as np
import random
import math

# general hints
# - why rrt? saves a lot of inverse kinematic calculation time
# - overview: obstacles in configuration space
# - use inverse kinematics for the obstacles to transform them into c space
# - task space - euclidean distance?!? distance function?
# - guided search?

# influenced by the robot (usually 6 for 1 configuration or 6 angles)
_CFG_LEN = 6
_STATE_LEN = 6
# sampling interval maximum of in between configurations on collision check
# (higher == higher precision / 1 = no interpolation)
_RRT_PRECISION_COLLISION = 10
# sampling interval maximum of interpolation configurations on nearest
# neighbor search (higher == higher precision / 1 = no interpolation)
_RRT_PRECISION_NEIGHBOR = 5
# whether to plot the history of graphs
_PLOT_HISTORY = False


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



# green = high z coordinate
# blue = low z coordinate
# input:
# - minimum:
# - maximum:
# - value:
def heatmap_rgb(value):
    ratio = map_interval(0.567, 3.567, 0.0, 1.0, value)
    b = max(0, (1 - ratio))
    g = max(0, ratio)
    return (0, g, b)



# input:
# - graph;
def attributes_and_spatial_layout(graph):
    layout = []
    for vertex in graph.vs:
        vertex_pose = kin.get_pose_from(kin.forward(vertex["configuration"]))
        vertex["vertex_size"] = map_interval(0.567, 3.567, 10.0, 40.0, vertex_pose[2])
        vertex["vertex_color"] = heatmap_rgb(vertex_pose[2])
        layout.append((vertex_pose[0],vertex_pose[1]))
    return layout



# input:
# - graph: graph to plot
# - idx: index of the plot (in a series of plots)
def plot_igraph(graph, idx = 0):
    # calculate the layout
    layout = attributes_and_spatial_layout(graph)
    # apply the layout to the plot
    visual_style = {}
    visual_style["vertex_size"] = graph.vs["vertex_size"]
    visual_style["vertex_color"] = graph.vs["vertex_color"]
    #visual_style["vertex_label"] = graph.vs["name"]
    #visual_style["edge_width"] = [1 + 2 * int(is_formal) for is_formal in g.es["is_formal"]]
    visual_style["layout"] = layout
    # need to be specified explicitly due to custom layout
    bbox = ig.BoundingBox(0,0,666,666)
    bbox.contract(16)
    visual_style["bbox"] = bbox
    #visual_style["margin"] = 16 # same as bbox contract
    #visual_style["rescale"] = False # deactivate rescaling
    #visual_style["asp"] = False # deactivate 1:1 aspect ratio
    #visual_style["xlim"] = (0,5)
    #visual_style["ylim"] = (0,5)
    ig.plot(graph, "plot_igraph_"+str(idx)+".png", **visual_style)
    return



# input:
# - graph:
# - env:
def draw_graph(graph, env):
    mf._DEBUG_DRAW = []

    for a,b in graph.get_edgelist():
        cfg_init = graph.vs["configuration"][a]
        kin_init = kin.forward(cfg_init)
        pose_init = kin.get_pose_from(kin_init)
        
        cfg_goal = graph.vs["configuration"][b]
        kin_goal = kin.forward(cfg_goal)
        pose_goal = kin.get_pose_from(kin_goal)

        col = np.array((heatmap_rgb(pose_init[2]),heatmap_rgb(pose_goal[2])))

        mf._DEBUG_DRAW.append(misc.DrawAxes(env, kin_init, 0.1, 1))
        mf._DEBUG_DRAW.append(misc.DrawAxes(env, kin_goal, 0.1, 1))
        mf._DEBUG_DRAW.append(env.drawlinestrip(points=np.array(((pose_init[:3]),(pose_goal[:3]))),
                                                linewidth=1.0,
                                                colors=col))
    
    
    
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
    # 0.0 and 1.6 are experimentally proven to describe the average norm of an
    # average motion
    max_i = int(map_interval(0.0, 1.6, 1.0, precision, np.linalg.norm(cfg_goal - cfg_init)))
    factor = 1 / max_i
    return max_i, factor



# input:
# - from_min:
# - from_max:
# - to_min:
# - to_max:
# - value:
def map_interval(from_min, from_max, to_min, to_max, value):
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min



# input:
# - robot:
# - configuration: configuration to be checked
# output:
# - boolean whether configuration is valid (in free space)
def is_valid(robot, configuration):
    configuration_backup = robot.GetDOFValues()
    env = robot.GetEnv()
    with env: # lock environment
        robot.SetDOFValues(configuration)
        # check for inlier's and set the corresponding report.contacts number
        # do not check for robot base
        # (as it is colliding all the time due to a bad environment)
        contact_count = 0
        for link in robot.GetLinks()[1:]:
            report = CollisionReport()
            collision = env.CheckCollision(link,report=report)
            contact_count += report.numCols
        
        # check additionaly for self collision
        contact_count = contact_count + report.numCols if robot.CheckSelfCollision() else contact_count
        robot.SetDOFValues(configuration_backup)
    
    return (contact_count < 1)



# input:
# - robot:
# - cfg_init:
# - cfg_goal:
# output:
# - if the path is collision free
def is_valid_path(robot, cfg_init, cfg_goal):
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
def generate_random_state(robot):
    lower,upper = robot.GetDOFLimits()
    angular_limits_difference = upper - lower
    configuration_random = lower + np.random.sample(len(lower)) * angular_limits_difference
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
# - new state generated by the given state_init and delta_time
def generate_state(robot, state_init, state_goal, delta_time):
    cfg_init = get_cfg(state_init)
    cfg_goal = get_cfg(state_goal)
    # calculate difference & sign of the difference
    difference = mf.difference_rel(cfg_init, cfg_goal)
    sign = np.sign(difference)
    # calculate limits (without signs)
    v_limit = np.sqrt(np.fabs(difference) * robot.GetDOFAccelerationLimits())
    velocity_limit = np.minimum(v_limit, robot.GetDOFVelocityLimits())
    times_acc = velocity_limit / robot.GetDOFAccelerationLimits()
    # calculate and apply delta
    delta = velocity_limit * (delta_time * 0.5 * times_acc)
    cfg_new = limit_cfg(robot, cfg_init + (delta * sign))
    return create_state(cfg_new)



# input:
# - robot;
# - cfg:
# limit the configuration to the given lower and upper limits and return the result
def limit_cfg(robot, cfg):
    lower,upper = robot.GetDOFLimits()
    return np.clip(cfg, lower, upper)



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
    
    if _PLOT_HISTORY: plot_igraph(g)
    
    # entire rt generation algorithm as in [Lav98c]
    for i in range(1, vertex_count):
        state_random = generate_random_state(robot)
        state_near,state_near_idx = find_nearest_neighbor(robot, state_random, g)
        state_new = generate_state(robot, state_near, state_random, delta_time)
        state_new_idx = insert_state(state_new, g)
        # add edge and assign distance as an attribute
        edge_idx = insert_edge(state_near_idx, state_new_idx, g)
        g.es[edge_idx]["weight"] = np.linalg.norm(get_cfg(state_new) - get_cfg(state_near))
        if _PLOT_HISTORY: plot_igraph(g, i)
        
    if not _PLOT_HISTORY: plot_igraph(g)
    
    return g



def node_to_cfg(graph, index_path):
    cfg_path = []
    for i in xrange(len(index_path)):
        cfg = graph.vs["configuration"][index_path[i]]
        cfg_path.append(cfg)
    return cfg_path
    
    
    
# input;
# - robot:
# - goal_cfg:
# public interface of the rapidly-exploring random tree algorithm
def rrt(robot, goal_cfg):
    vertex_count = 300
    delta_time = 0.5
    
    # clone of the environment
    # env = robot.GetEnv().CloneSelf(CloningOptions.Bodies | CloningOptions.RealControllers )
    # new_robot = env.GetRobots()[0]
    # env.Destroy()
    
    # dangerous due to https://github.com/rdiankov/openrave/issues/335
    # do not want to introduce another cause for errors...
    g = generate_rt(robot, vertex_count, delta_time)
    
    # add goal to the graph
    goal_idx = vertex_count - 1
    assert(goal_idx != 0)
    shortest_paths = g.get_all_shortest_paths(0, goal_idx)
    shortest_path = node_to_cfg(g, shortest_paths[0])

    # draw rrt related information
    draw_graph(g, robot.GetEnv())
    return shortest_path
    
    #plot_igraph(g, g.layout_kamada_kawai())
    #vertex_goal = g.vs.select(lambda vertex: (vertex["configuration"] == goal_cfg).all()) if np.array_equal(state_new, state_goal) else None
    #goal_idx = vertex_goal[0].index
