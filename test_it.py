import networkx as nx 
import numpy as np
import argparse
import random
import helper
import PlanarArm
import PlanarArmEnvironment
import os

NO_ENV = 1
PP_PER_ENV = 1
Directory  = None
EDGE_DISCRETIZATION = 10

def write_to_file(s_node, g_node, path_nodes, occ_grid, obstacles):
    global Directory

    occ_grid = list(occ_grid.reshape(100))
    obs = []
    for ob in obstacles:
        for b in ob:
            obs.extend(b)

    with open(Directory + "/start_nodes.txt", 'a') as file:
        file.writelines(str(s_node)+"\n")
    with open(Directory + "/goal_nodes.txt", 'a') as file:
        file.writelines(str(g_node)+"\n")
    with open(Directory + "/path_nodes.txt", 'a') as file:
        file.writelines(" ".join(str(node) for node in path_nodes)+"\n")
    with open(Directory + "/occ_grid.txt", 'a') as file:
        file.writelines(" ".join(str(status) for status in occ_grid)+"\n")
    with open(Directory + "/obstacle_posns.txt", 'a') as file:
        file.writelines(" ".join(str(b) for b in obs)+"\n")

def get_obstacles():
    obstacles = []
    
    ox = random.randint(2,6)/10.0
    oy = random.randint(0,9)/10.0

    obs1 = [[ox,0], [ox+0.2,0] ,[ox+0.2, oy], [ox, oy]]
    obs2 = [[ox, oy+0.1], [ox+0.2, oy+0.1], [ox+0.2, 1], [ox, 1]]

    return obs1, obs2, ox

def remove_invalid_edges(G, arm):

    to_remove = []
    for i, node in enumerate(G.nodes()):
        config = helper.state_to_numpy(G.node[node]['state'])
        # print("config = ", config)
        arm.set_base_location(config[:2])
        arm.set_configuration(config[2:])
        if(arm.in_collision()):
            to_remove.append(node)
    for node in to_remove:
        G.remove_node(node)

    print("Removed "+`len(to_remove)`+" invalid nodes")
    # raw_input("press enter")

    to_remove = []
    for i,edge in enumerate(G.edges()):
        if(i%1000==0):
            print("no of edges checked = ", i)
        u,v = edge
        # print("i, u, v = ", i, u, v)
        node1_pos = helper.state_to_numpy(G.node[u]['state'])
        node2_pos = helper.state_to_numpy(G.node[v]['state'])

        diff = node2_pos - node1_pos
        step = diff/EDGE_DISCRETIZATION

        for i in range(EDGE_DISCRETIZATION+1):
            nodepos = node1_pos + step*i
            arm.set_base_location(nodepos[:2])
            arm.set_configuration(nodepos[2:])
            if(arm.in_collision()):
                to_remove.append((u,v))
                break
    
    for edge in to_remove:
        u, v = edge
        G.remove_edge(u, v) 

    print("Removed "+`len(to_remove)`+" invalid edges")
    # raw_input("press enter")

    return G

def is_free(p, obstacles):
    for obs in obstacles:
        flag = 0
        if(p[0]<obs[0][0] or p[0]>obs[2][0]):
            flag = 1
        if(p[1]<obs[0][1] or p[1]>obs[2][1]):
            flag = 1
        if(not flag):
            return 0
    return 1

def create_occ_grid(obstacles):
    occ_grid = np.ones((10,10), dtype=int)
    eps = 0.01
    for i in range(0,10):
        for j in range(0, 10):
            if(not (is_free((i/10.0+eps,j/10.0+eps), obstacles))):
                occ_grid[i,j] = 0
            else:
                occ_grid[i,j] = 1
    return occ_grid.ravel()

def main():

    obs1, obs2, ox = get_obstacles()
    env = PlanarArm.PlanarArmEnvironment()
    env.set_obstacles([obs1])
    env.add_obstacle([obs2])
    arm = PlanarArm.PlanarArm(env)
    arm.visualize()

if __name__ == '__main__':
    main()