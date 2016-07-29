from gurobipy import *
import numpy as np
import itertools
from priodict import priorityDictionary
from copy import deepcopy
import sys
import time
import math
from munkres import Munkres
import networkx as nx

def compute_new_G(G_ORIG, nodes_to_remove, bs):
    #print nodes_to_remove
    G = deepcopy(G_ORIG)
    new_connections = set()
    for n in nodes_to_remove:
        for j in range(G.shape[0]):
            if(G[n,j] == 1):
                G[n,j] = 0
                new_connections.add(j)

    for n in nodes_to_remove:
        for i in range(G.shape[0]):
            G[i,n] = 0

    #print new_connections
    for c in new_connections:
        if c not in nodes_to_remove:
            G[bs,c] = 1
            G[c,bs] = 1

    #print G
    return G

def lazy_constraint(model, where):
    MAXCUTS = 20    
    if((where == GRB.callback.MIPNODE and GRB.status.OPTIMAL == model.cbGet(GRB.callback.MIPNODE_STATUS) ) or where == GRB.callback.MIPSOL):
        X_W = {}
        V_VAL = {}
        G = model._G
        bs = model._bs
        X = model._X
        Y = model._Y
        edges = model._edges

        for edge in edges:
            if(where == GRB.callback.MIPNODE):
                x = model.cbGetNodeRel(X[(edge[0],edge[1])])
            else:
                x = model.cbGetSolution(X[(edge[0],edge[1])])
                #print "VAL " + str(x)
            X_W[(edge[0],edge[1])] = x

        #retrive the Y val
        y_val = {}
        for i in range(G.shape[0]):
            if i == bs: continue
            if(where == GRB.callback.MIPNODE):
                y_val[i] = model.cbGetNodeRel(Y[i])
            else:
                y_val[i] = model.cbGetSolution(Y[i])
                        
        for i in range(G.shape[0]):
            if i == bs or y_val[i] == 0: continue
            cut_number = 0
            added_constraint = True
            
            while cut_number < MAXCUTS and added_constraint:
                added_constraint = False

                #print "ESAMINO NODO " + str(i)
                #print Y[i]
            

                G_W = nx.DiGraph()
                for v1 in range(G.shape[0]):
                    G_W.add_node(v1)

                for edge in edges:
                    G_W.add_edge(edge[0],edge[1], capacity = X_W[(edge[0],edge[1])])

                cut_value, partition = nx.minimum_cut(G_W, bs, i)

                #print "CUT VAL " + str(cut_value)
                #print "Y VAL " + str(y_val[i])
                #print partition

                partition_root = partition[0]
                if cut_value < y_val[i]:
                    added_constraint = True
                    cut_number += 1
                    cut_set = set()

                    for node in partition_root:
                        for j in range(G.shape[0]):
                            if(j not in partition_root and G[node,j] == 1):
                                cut_set.add((node,j))

                    expr = 0.0
                    #print "CUT SET"
                    #print cut_set

                    for edge in cut_set:
                        X_W[(edge[0], edge[1])] == 1.0
                        expr += X[edge[0], edge[1]]

                    model.cbLazy(expr >= Y[i])                   


def addVariablesDepl(m, G, TERMINALS, X, Y, edges, out_arcs):

    for v in range(G.shape[0]):
        if v == TERMINALS[0]: continue
        Y[v] = m.addVar(vtype=GRB.BINARY, name="y[%d]" %(v))
            
    for v1 in range(G.shape[0]):
        for v2 in range(G.shape[0]):
            if v1 == v2 or v1 == TERMINALS[0] or v2 == TERMINALS[0]: continue

            if (G[v1,v2] == 1): #edge
                X[v1, v2] = m.addVar(vtype=GRB.BINARY, name="x[%d,%d]" %(v1,v2))
                edges.append((v1,v2))

    for v in range(G.shape[0]):
        if(G[TERMINALS[0], v] == 1):
            X[TERMINALS[0], v] = m.addVar(vtype=GRB.BINARY, name="x[%d,%d]" %(v1,v2))
            edges.append((TERMINALS[0], v))

    for v1 in range(G.shape[0]):
        out_arcs[v1] = []
        for v2 in range(G.shape[0]):
            if (G[v1,v2] == 1 and v1 != v2):
                out_arcs[v1].append((v1,v2))        


def addConstraintsDepl(m, G, TERMINALS, X, Y, edges, max_dim):
    #The BS is at a fixed position
    #m.addConstr(R[0, CUR_LOC[0]] == 1)
    #m.addConstr(quicksum(X[edge[0], edge[1]] for edge in filter(lambda x: x[0] == TERMINALS[0], edges)) >= 1)    

    #Mapping X-Y variables
    for v in range(G.shape[0]):
        if v == TERMINALS[0]: continue 
        #m.addConstr(Y[v] <= 1)      
        m.addConstr(quicksum(X[edge[0], edge[1]] for edge in filter(lambda x: x[1] == v, edges)) == Y[v])

    #The number of edges is at most NROBOTS - 1

    m.addConstr(quicksum(X[edge[0], edge[1]] for edge in edges) <= max_dim)

    #Not sure this can strenghten relaxation
    #for v in range(G.shape[0]):
        #in degree <= out degree for non terminal nodes.
        #if v in TERMINALS: continue
        #m.addConstr(quicksum(X[edge[0], edge[1]] for edge in filter(lambda x: x[1] == v, edges)) <= quicksum(X[edge[0], edge[1]] for edge in filter(lambda x: x[0] == v, edges)))

def setObjectiveDepl(m, Y, FRONTIERS, utilities):
    #print NORM
    m.setObjective(quicksum(utilities[FRONTIERS.index(fr)]*Y[fr] for fr in FRONTIERS), sense = GRB.MAXIMIZE)

def solve_bpcst(true_frontiers, bs, utilities, max_dim, G, out_arcs):
    X = {}
    Y = {}
    edges = []
    
    m = Model('Bpcst_opt')

    addVariablesDepl(m, G, [bs] + true_frontiers, X, Y, edges, out_arcs)
    m.update()
    addConstraintsDepl(m, G, [bs] + true_frontiers, X, Y, edges, max_dim)

    setObjectiveDepl(m, Y, true_frontiers, utilities)
    
    m.setParam( 'OutputFlag', True )
    m.setParam("TimeLimit", 3600)
    #m.setParam("Threads", 3)
    #m.setParam("Presolve", 2) #aggressive
    #m.setParam("MIPFocus", 1)
    #m._maxdim = max_dim
    m.params.LazyConstraints = 1
    m._G = G
    m._X = X
    m._Y = Y
    m._edges = edges
    m._bs = bs
    m.update()

    m.optimize(lazy_constraint)
    if m.status == GRB.status.INF_OR_UNBD or m.status == GRB.status.INFEASIBLE:
        return []
    
    elif m.status == GRB.status.INTERRUPTED:

        return []

    elif m.status == GRB.status.OPTIMAL or (m.status == GRB.status.TIME_LIMIT):
        solution = []
        x = m.getAttr('x', X) 
        y = m.getAttr('x', Y)

        selected_frontiers = []

        for edge in edges:
            if (x[edge[0], edge[1]] > 0.5): #f[fr,v1,v2] > 0 and (v1,v2) not in solution and (v2,v1) not in solution): #edge
                solution.append((edge[0], edge[1]))


        for fr in true_frontiers:
            if (y[fr] > 0.5):
                selected_frontiers.append(fr)

        #print "THE SOLUTION IS:"
        #print solution
        return ([solution], selected_frontiers)



def allocate_remaining_robots(allocation, nodes, out_arcs, DIST, CUR_LOC, NOT_READY_COMM, dist_not_ready_comm):
    #print "Allocating remaining robots"
    new_allocation = deepcopy(allocation)
    fake_frontiers = []
    candidate_positions = []
    for n in nodes:
        for edge_neigh in out_arcs[n]:
            if(not(edge_neigh[1] in nodes) and  not(edge_neigh[1] in candidate_positions)):
                candidate_positions.append(edge_neigh[1])

    #Construct the matrix for the hungarian algorithm
    #These are to retrieve the correct ids from the matrix    
    robot_ids_map = {}
    matrix = []
    robot_matrix_index = 0

    for robot in CUR_LOC:
        if(not(robot in map(lambda x: x[0], allocation))):
            matrix.append(map(lambda x: DIST[x, CUR_LOC[robot]], candidate_positions))
            robot_ids_map[robot_matrix_index] = robot
            robot_matrix_index += 1

    for robot in NOT_READY_COMM:
        if(not(robot in map(lambda x: x[0], allocation))):
            matrix.append(map(lambda x: dist_not_ready_comm[robot][x], candidate_positions))
            
            robot_ids_map[robot_matrix_index] = robot
            robot_matrix_index += 1

    #print robot_ids_map
    #print candidate_positions
    #print matrix

    
    m = Munkres()
    indexes = m.compute(matrix)
    #print_matrix(matrix, msg='Lowest cost through this matrix:')
    for row, column in indexes:
        #print column
        new_allocation.append((robot_ids_map[row], candidate_positions[column]))
        #new_allocation.append((robot_ids_map[column], candidate_positions[row]))
        fake_frontiers.append(candidate_positions[column])

    
    #print new_allocation
    
    return (new_allocation, fake_frontiers)

def allocate_robots_min_cost(allocation, nodes, DIST, CUR_LOC, NOT_READY_COMM, dist_not_ready_comm):
    #print "Allocating robots to current nodes"

    new_allocation = deepcopy(allocation)
    candidate_positions = deepcopy(nodes)

    #Construct the matrix for the hungarian algorithm
    #These are to retrieve the correct ids from the matrix    
    robot_ids_map = {}
    matrix = []
    robot_matrix_index = 0

    for robot in CUR_LOC:
        if(not(robot in map(lambda x: x[0], allocation))):
            matrix.append(map(lambda x: DIST[x, CUR_LOC[robot]], candidate_positions))
            robot_ids_map[robot_matrix_index] = robot
            robot_matrix_index += 1

    for robot in NOT_READY_COMM:
        if(not(robot in map(lambda x: x[0], allocation))):
            matrix.append(map(lambda x: dist_not_ready_comm[robot][x], candidate_positions))
            
            robot_ids_map[robot_matrix_index] = robot
            robot_matrix_index += 1

    #print robot_ids_map
    #print candidate_positions
    #print matrix
    
    m = Munkres()
    indexes = m.compute(matrix)
    #print_matrix(matrix, msg='Lowest cost through this matrix:')
    for row, column in indexes:
        #print column
        new_allocation.append((robot_ids_map[row], candidate_positions[column]))
        #new_allocation.append((robot_ids_map[column], candidate_positions[row]))
        #fake_frontiers.append(candidate_positions[column])
    
    #print new_allocation
    return new_allocation

def compute_min_path(G_ORIG, frontier, DIST, allocation_n_r_dict, allocation_r_n_dict, CUR_LOC, CUR_DISTANCES_NR, CUR_LOC_NCOMM_DEST, NOT_READY_COMM):
    #print "Examining frontier " + str(frontier)
    #make a single dictionary
    r_n_dict = {}
    n_r_dict = {}
    distance_dict = {}  

    for robot in allocation_r_n_dict:
        if(robot in NOT_READY_COMM):
            r_n_dict[robot] = allocation_r_n_dict[robot]
            n_r_dict[allocation_r_n_dict[robot]] = robot
            distance_dict[robot] = CUR_DISTANCES_NR[robot][r_n_dict[robot]]
            
        else:
            r_n_dict[robot] = allocation_r_n_dict[robot]
            n_r_dict[allocation_r_n_dict[robot]] = robot
            distance_dict[robot] = DIST[CUR_LOC[robot], r_n_dict[robot]]
        
    for robot in filter(lambda x: x not in NOT_READY_COMM, CUR_LOC_NCOMM_DEST):
        r_n_dict[robot] = CUR_LOC_NCOMM_DEST[robot]
        n_r_dict[CUR_LOC_NCOMM_DEST[robot]] = robot
        distance_dict[robot] = CUR_DISTANCES_NR[robot][r_n_dict[robot]]

    robots_sorted = distance_dict.keys()
    robots_sorted = sorted(robots_sorted, key=lambda x:distance_dict[x])
    
    for robot in robots_sorted:
        #print "Examining robot " + str(robot) + " in node " + str(r_n_dict[robot]) + " at distance " + str(distance_dict[robot])
        #create a copy of the dictionaries containing only the robots with distance <= dist[robot]
        r_n_dict_temp = {}
        n_r_dict_temp = {}
        distance_dict_temp = {}
        for robot2 in distance_dict:
            if distance_dict[robot2] <= distance_dict[robot]:
                distance_dict_temp[robot2] = distance_dict[robot2]
                r_n_dict_temp[robot2] = r_n_dict[robot2]
                n_r_dict_temp[r_n_dict[robot2]] = robot2

        #print "Dictionary that will be used:"
        #print n_r_dict_temp
        #create a graph restricted to the robots (nodes) in temp
        G = nx.Graph()
        for node in n_r_dict_temp:
            G.add_node(node)
        G.add_node(CUR_LOC[0]) #bs

        for n1 in n_r_dict_temp:
            for n2 in n_r_dict_temp:
                if(n1 != n2 and G_ORIG[n1,n2]==1):
                    G.add_edge(n1,n2)
                    #print "adding edge " + str(n1) + "," + str(n2)

        for n in n_r_dict_temp:
            if(G_ORIG[CUR_LOC[0], n] == 1):
                G.add_edge(n,CUR_LOC[0])
                G.add_edge(CUR_LOC[0],n)
                #print "adding edge " + str(CUR_LOC[0]) + "," + str(n)

        if(not(frontier in nx.descendants(G, source=CUR_LOC[0]))): continue #skip this config

        #since the robots are sorted by increasing distance order, as soon as I find a feasible path I can stop the search
        return nx.shortest_path(G, source=frontier, target=CUR_LOC[0])

    print "FATAL ERROR!"
    return None



def run():
    my_dir = ""#"/home/banfi/NetBeansProjects/MRESim-master_4.0/"
    MIXED = False
    if(len(sys.argv) > 1): #mixed version
        MIXED = True

    if(MIXED):
        fm = open(my_dir + "mixIlp.txt", "w")
        fm.write("COMPUTING")
        fm.close()
   
    f_g = open( my_dir + "G.txt", "r")
    lines = f_g.readlines()
    s = ""
    for line in lines:
        linet = line.replace("\t"," ").replace("\n","").replace("\r","")
	linet = linet[:len(linet)-1]
	#print linet
	s += linet + ";"
    s = s[:len(s)-1]
    
    #print s
    G_ORIG = np.matrix(s)
    #print G_ORIG

    f_m1 = open(my_dir + "m_1.txt", "r")
    lines = f_m1.readlines()
    replanning_robots = []
    CUR_LOC = {}
    bs = -1
    for i in range(len(lines)):
        line = lines[i]
        s = line.split()
        CUR_LOC[ int(s[0]) ] = int(s[1]) - 1
        replanning_robots.append( int(s[0]))
        if(i == 0):
            bs = int(s[1]) - 1
            bs_key = int(s[0])
    #print CUR_LOC
    f_m1.close()

    CUR_DISTANCES_NR = {}
    CUR_LOC_NCOMM_DEST = {}

    NOT_READY_COMM = []
    if(os.path.exists(my_dir + "notReadyDist.txt")):
       f_nrc = open(my_dir + "notReadyDist.txt", "r")
       lines = f_nrc.readlines()
       for line in lines:
           s = line.split()
           if(s[0] != "COMM"):
               CUR_DISTANCES_NR[int(s[0])] = map(lambda x: int(x), s[1:])
           else:
               NOT_READY_COMM= map(lambda x: int(x), s[1:])

    #physical distances
    f_wp = open(my_dir + "w_p.txt","r")
    lines = f_wp.readlines()
    s = ""
    for line in lines:
        linet = line.replace("\t"," ").replace("\n","").replace("\r","")
	linet = linet[:len(linet)-1]
	#print linet
	s += linet + ";"
    s = s[:len(s)-1]
    f_wp.close()
    
    DIST = np.matrix(s)
    #print DIST

    nodes_to_remove = []
    
    currentTreeOnlyNotComm = []
    
    #a cosa serve?
    comm_nodes_to_reallocate = []


    if(os.path.exists(my_dir + "currentTree.txt")):
        #fill currentTree, the list of the current Tree initially without the replanning robots
        fct = open(my_dir + "currentTree.txt" , "r")
        currentTree = []
        for line in fct.readlines():
            s = line.split()
            if(int(s[0]) not in replanning_robots):
                currentTree.append((int(s[0]), int(s[1])))
                nodes_to_remove.append(int(s[1]) - 1)
                
                
                if(int(s[0]) not in NOT_READY_COMM):
                    currentTreeOnlyNotComm.append((int(s[0]), int(s[1])))
                    CUR_LOC_NCOMM_DEST[int(s[0])] = int(s[1]) - 1
                else:
                    comm_nodes_to_reallocate.append(int(s[1]) - 1)

        fct.close()
        #TODO
        #modify G (remove each edge from non replanning robots' locations, and add new edges from the BS to those locations that would become reachable)
        if(len(currentTree) > 0):
            G = compute_new_G(G_ORIG, nodes_to_remove, bs) #this is the bs
        else:
            G = G_ORIG

        #np.savetxt("nuovoG.txt", G)
    else:
        currentTree = [] #is an empty list
        G = G_ORIG

    print "CURRENT TREE:"
    print currentTree  

    print "COMM NODES TO REALLOCATE:"
    print comm_nodes_to_reallocate

    print "CURRENT DESTINATIONS OF NON COMM/NON READY ROBOTS:"
    print CUR_LOC_NCOMM_DEST

    f_f = open(my_dir + "F.txt", "r")
    lines = f_f.readlines()
    FRONTIERS = np.matrix(lines[0])
    FRONTIERS = FRONTIERS.tolist()[0]
    FRONTIERS = map(lambda x: x - 1,FRONTIERS)
    f_f.close()

    f_fa = open(my_dir + "FA.txt", "r")
    lines = f_fa.readlines()
    F_AREAS = np.matrix(lines[0])
    F_AREAS = F_AREAS.tolist()[0]
    f_fa.close()

    #finally, load also the current forming frontiers (will be used to recompute the first forming path)
    f_old_frontiers = open(my_dir + "currWaitingList.txt", "r")
    lines = f_old_frontiers.readlines()
    old_frontiers = []
    for line in lines:
        s = line.split()
        old_frontiers.append(int(s[0]))
    f_old_frontiers.close()        

    #exclude from frontiers those that will be occupied by some non-replanning robots
    temp_frontiers = []
    temp_f_areas = []
    for f in range(len(FRONTIERS)):
        if(FRONTIERS[f] not in nodes_to_remove):
            temp_frontiers.append(FRONTIERS[f])
            temp_f_areas.append(F_AREAS[f])

    FRONTIERS = temp_frontiers
    F_AREAS= temp_f_areas    

    print FRONTIERS
    print F_AREAS

    true_frontiers = filter(lambda x: x != bs, FRONTIERS)
    if bs in FRONTIERS:
        info_gain = []
        for i in range(len(F_AREAS)):
            if i != FRONTIERS.index(bs):
                info_gain.append(F_AREAS[i])
    else:
        info_gain = F_AREAS
    
    #normalize info_gain
    max_info_gain = max(info_gain)
    info_gain = map(lambda x: float(x)/max_info_gain, info_gain)

    #print info_gain
    #Compute a utility also based on expected movement
    min_dists = []    
    for fr in true_frontiers:
        min_dist = -1
        #print fr
        for key in CUR_LOC:
            if (CUR_LOC[key] == bs): continue
            cur_robot_pos = CUR_LOC[key]
            #print cur_robot_pos
            if min_dist == -1:
                min_dist = DIST[cur_robot_pos, fr]
            elif DIST[cur_robot_pos, fr] < min_dist:
                min_dist = DIST[cur_robot_pos, fr]

        if(min_dist == 0): #may happen, as I am passing them as integers
            min_dist = 1
        min_dists.append(min_dist)
        
        #print fr
        #print min_dist

    #normalize min_dists
    max_min_dists = max(min_dists)
    min_dists = map(lambda x: float(x)/max_min_dists, min_dists)

    utilities = []                                                                                                           
    for g in range(len(info_gain)):                                                                                          
        utilities.append((info_gain[g])/(min_dists[g]**2))                                                       
    
    #print true_frontiers
    true_frontiers = sorted(true_frontiers, key = lambda x: utilities[true_frontiers.index(x)])
    true_frontiers.reverse()
    utilities.sort()
    utilities.reverse()


    print "Utilities:"
    print utilities
    print "Frontiers:"
    print true_frontiers

    #print true_frontiers
    #print utilities

    z_best = [0]
    sol = [[]]
    selected_frontiers = []
    out_arcs = {} #filled, to be used also later
    (sol, selected_frontiers) = solve_bpcst(true_frontiers, CUR_LOC[0], utilities, len(CUR_LOC) - 1, G, out_arcs)

    if(len(selected_frontiers) == 0 and len(CUR_DISTANCES_NR) == 0):
        #no frontiers taken with all robots ready
        if(MIXED):
            fcmin = open(my_dir + "costMinIlp.txt", "w")
            fcmin.write("Inf")
            fcmin.close()
            fm = open(my_dir + "mixIlp.txt", "w")
            fm.write("DONE")
            fm.close()
        else:
            fcmin = open(my_dir + "costMin.txt", "w")
            fcmin.write("Inf")
            fcmin.close()      
        
        return 
    
    nodes = []
    for edge in sol[0]:
        if not(edge[0]) in nodes and edge[0] != bs:
            nodes.append(edge[0])
        if not(edge[1]) in nodes and edge[1] != bs:
            nodes.append(edge[1]) 

    CUR_LOC_COPY = deepcopy(CUR_LOC)
    del(CUR_LOC_COPY[bs_key])

    allocation = []
    all_nodes = nodes + comm_nodes_to_reallocate

    #metto anche i comm not ready qua. devo allocare sia i nuovi nodi che i vecchi che erano coperti da dei robot in comm.
    if(len(all_nodes) > 0):
        allocation = allocate_robots_min_cost(allocation, all_nodes, DIST, CUR_LOC_COPY, NOT_READY_COMM, CUR_DISTANCES_NR)

    print "ALLOCATION:"
    print allocation

    #It can also happen that the optimal solution does not saturate the budget. In that case, use the hungarian algorithm
    #place the remaining robots near the created network to minimize their travel distance (this can be done in the modified graph because the tree is completed)
    if(len(allocation) < (len(CUR_LOC) -1 + len(NOT_READY_COMM))):
        print "ADDING REMAINING POSITIONS..."
        (allocation, fake_frontiers) = allocate_remaining_robots(allocation, nodes + [bs], out_arcs, DIST, CUR_LOC_COPY, NOT_READY_COMM, CUR_DISTANCES_NR)
    
        selected_frontiers += fake_frontiers

        #tempTree = tempTree + fake_frontiers

    allocation = sorted(allocation, key=lambda x:x[0])

    print "FINAL ALLOCATION:"
    print allocation

    allocation_n_r_dict = {}
    allocation_r_n_dict = {}
    s = "0" + "\t" + str(bs + 1) + "\n"

    for a in allocation:
        s += str(a[0]) + "\t" + str(a[1] + 1) + "\n"
        allocation_n_r_dict[a[1]] = a[0]
        allocation_r_n_dict[a[0]] = a[1]

    if(MIXED):
        fmopt = open(my_dir + "m_optIlp.txt", "w")        
    else:
        fmopt = open(my_dir + "m_opt.txt", "w")
    fmopt.write(s)
    fmopt.close()

    #in any case the new currenTree is constructed like this
    newTree = deepcopy(currentTreeOnlyNotComm)
    newTree += map(lambda x: (x[0], x[1] + 1), allocation)
    newTree = sorted(newTree, key=lambda x:x[0])
    s = "0" + "\t" + str(bs + 1) + "\n"

    for c in newTree:
        s += str(c[0]) + "\t" + str(c[1]) + "\n"
    if(MIXED):
        fct = open(my_dir + "currentTreeIlp.txt", "w")
    else:
        fct = open(my_dir + "currentTree.txt", "w")
    fct.write(s)
    fct.close()

    if(MIXED):
        fcmin = open(my_dir + "costMinIlp.txt", "w")
    else:
        fcmin = open(my_dir + "costMin.txt", "w")
        
    fcmin.write("OK")
    fcmin.close()

    total_frontiers = selected_frontiers + old_frontiers
    frontier_paths = {}
    for frontier in total_frontiers:
        frontier_paths[frontier] = compute_min_path(G_ORIG, frontier, DIST, allocation_n_r_dict, allocation_r_n_dict, CUR_LOC, CUR_DISTANCES_NR, CUR_LOC_NCOMM_DEST, NOT_READY_COMM)

  
    print "FRONTIER PATHS:"
    print frontier_paths
    #Finally, write also the updated waiting list.
    if(MIXED):
        fw = open(my_dir + "waitingIlp.txt", "w")
    else:
        fw = open(my_dir + "waiting.txt", "w")   
    
    for fr in frontier_paths: #contains also the old ones
        s = ""
        s += str(fr) #+1?
        for relay in frontier_paths[fr]:
            if relay == bs: continue
            found = False
            for el in allocation:
                if(el[1] == relay):
                    s += "\t" + str(el[0]) #this is the actual hashmap used in java
                    found = True
                    break

            if(not found):
                for el in currentTreeOnlyNotComm:
                    if(el[1] - 1 == relay):
                        s += "\t" + str(el[0])
                        break

        s += "\n"
        fw.write(s)

    fw.close()
    
    #write also the dimension of the solved instance (for logging)
    edges = []
    for v in out_arcs:
        for e in out_arcs[v]:
            if(e not in edges and (e[1],e[0]) not in edges):
                edges.append(e)

    fdim = open(my_dir + "dim.txt", "w")
    fdim.write(str(len(out_arcs)) + " " + str(len(edges)) + " " + str(len(CUR_LOC) - 1))
    fdim.close()

    if(MIXED):
        fm = open(my_dir + "mixIlp.txt", "w")
        fm.write("DONE")
        fm.close()

if __name__ == "__main__":
    try:
        run()
    except :
        ferr = open("error.txt", "r")
        n = int(ferr.readlines()[0])
        ferr.close()
        os.system("cp G.txt algoErrors/iG_" + str(n) + ".txt")
        os.system("cp currentTree.txt algoErrors/icurrentTree_" + str(n) + ".txt")
        os.system("cp notReadyDist.txt algoErrors/inotReadyDist_" + str(n) + ".txt")
        os.system("cp currWaitingList.txt algoErrors/icurrWaitingList_" + str(n) + ".txt")        
        os.system("cp m_1.txt algoErrors/im_1_" + str(n) + ".txt")
        os.system("cp w_p.txt algoErrors/iw_p_" + str(n) + ".txt")
        os.system("cp F.txt algoErrors/iF_" + str(n) + ".txt")
        os.system("cp FA.txt algoErrors/iFA_" + str(n) + ".txt")

        ferr = open("error.txt", "w")
        ferr.write(str(n + 1))
        ferr.close()
