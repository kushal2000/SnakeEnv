import numpy as np
from itertools import islice, chain
import math

def remove_invalid_edges(G, arm, EDGE_DISCRETIZATION = 10):

    to_remove = []
    for i,edge in enumerate(G.edges()):
        if(i%1000==0):
            print("no of edges checked = ", i)
        u,v = edge
        # print("i, u, v = ", i, u, v)
        node1_pos = state_to_numpy(G.node[u]['state'])
        node2_pos = state_to_numpy(G.node[v]['state'])

        diff = node2_pos - node1_pos
        step = diff/EDGE_DISCRETIZATION

        for i in range(EDGE_DISCRETIZATION+1):
            nodepos = node1_pos + step*i
            arm.set_base_location(nodepos[:2])
            arm.set_configuration(nodepos[2:])
            if(arm.in_collision()):
                to_remove.append((u,v))
                if('o' in u and 'o' in v):
                    print(u, v, " HAAAAAAAAAAWWWWWWWWWWW")
                break
    
    for edge in to_remove:
        u, v = edge
        G.remove_edge(u, v) 

    print("Removed "+str(len(to_remove))+" invalid edges")
    # raw_input("press enter")

    return G

def calc_weight(config1, config2):
    return math.sqrt(float(np.sum((config2-config1)**2)))

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

def remove_nodes(G, nodes):
    for node in nodes:
        if(node==-1):
            continue
        G.remove_node(node)
    return G

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

def state_to_numpy(state):
    strlist = state.split()
    val_list = [float(s) for s in strlist]
    return np.array(val_list) 

def numpy_to_state(val_list):
    val_list = [str(val) for val in val_list]
    state = " ".join(val_list)
    return state

def edge_to_configs(state1, state2, EDGE_DISCRETIZATION = 20):
    
    config1 = state_to_numpy(state1)
    config2 = state_to_numpy(state2)

    diff = config2 - config1
    step = diff/EDGE_DISCRETIZATION

    to_check = list()
    to_check.append(config1)

    for i in range(EDGE_DISCRETIZATION - 1):
        conf = config1 + step*(i+1)
        to_check.append(conf)

    return to_check

def connect_knn_for_one_node(G, K, node, THRESHOLD = 1):
    state = G.node[node]['state']
    conf = state_to_numpy(state)
    G1 = G.copy()

    for k in range(K):
        w = 1000000
        sn = None
        for node1 in G1.nodes():
            if(node == node1):
                continue
            state1 = G1.node[node1]['state']
            conf1  = state_to_numpy(state1)
            if(calc_weight(conf, conf1) < w):
                w = calc_weight(conf, conf1)
                sn = node1
        if(w<THRESHOLD):
            G.add_edge(node, sn)
            G[node][sn]['weight'] = w
            G1.remove_node(sn)
        else:
            break    
    return G

def connect_within_thresh(G, lmbda, threshold,nodes_l):
    for node in nodes_l:
        conf1 = state_to_numpy(G.node[node]['state'])
        for node1 in G.nodes():
            conf2 = state_to_numpy(G.node[node1]['state'])

            w = calc_weight(conf1, conf2)
            if(w<threshold*lmbda):
                G.add_edge(node, node1)
                G[node][node1]['weight'] = w
    return G