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
# - g: graph to plot
# - layout: plot layout algorithm
def plot_igraph(graph, layout):
    visual_style = {}
    visual_style["vertex_size"] = 20
    #visual_style["vertex_color"] = [color_dict[gender] for gender in g.vs["gender"]]
    visual_style["vertex_label"] = graph.vs["state"]
    #visual_style["edge_width"] = [1 + 2 * int(is_formal) for is_formal in g.es["is_formal"]]
    visual_style["layout"] = layout
    visual_style["bbox"] = (600,600)
    visual_style["margin"] = 20
    ig.plot(graph, **visual_style)
    return

# generate a random state in the configuration space
def generate_random_state():
    #angular_limit = robot.GetDOFLimits()
    state_random = 0.0
    return state_random

# input:
# - state:
# - graph:
# find the nearest neighbor of a (random) state about to be inserted into the graph
def find_nearest_neighbor(state, graph):
    state_near = 0.0
    state_near_idx = 0
    return state_near,state_near_idx

# input:
# - state_init:
# - state_goal:
# selects the minimal input_u to reach the state_goal from the state_init
def select_input(state_init, state_goal):
    #difference = np.fabs(mf.difference_rel(start_cfg, target_cfg))
    #velocity_limit, acceleration_limit, times_acc, times_dec, times_end = mf.limits_and_times(robot, difference, 'F')
    #velocity_limit, acceleration_limit, times_acc, times_dec, times_end = mf.discretize(difference, velocity_limit, acceleration_limit, times_acc, times_dec, times_end)
    input_u = 0.0
    return input_u

# input:
# - state_init:
# - input_u:
# - delta_time:
# generates a new state from the given state_init and input
def generate_state(state_init, input_u, delta_time):
    state_new = 0.0
    return state_new

# input:
# - robot: respective robot
# - vertex_count: number of vertices k
# - delta_time: incremental distance
# output:
# - rrt graph g
def generate_rrt(robot, target_cfg, vertex_count, delta_time):
    # create initial & goal state for the rrt algorithm
    velocity = 0.0
    state_init = np.append(robot.GetDOFValues(),velocity)
    print 'state_init: ',state_init
    state_goal = np.append(target_cfg,velocity)
    print 'state_goal: ',state_goal
    
    # create graph structure with initial state
    g = ig.Graph()
    g.add_vertices(vertex_count)
    g.vs[0]["state"] = state_init
    
    # entire rrt algorithm as in [Lav98c]
    for i in range(1, vertex_count):
        state_random = generate_random_state()
        state_near,state_near_idx = find_nearest_neighbor(state_random, g)
        input_u = select_input(state_near, state_random)
        state_new = generate_state(state_near, input_u, delta_time)
        g.vs[i]["state"] = state_new
        g.add_edges((state_near_idx, i))
        g.es[i-1]["input_u"] = input_u
    
    plot_igraph(g, g.layout_kamada_kawai())

    return g
