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

import gflags

gflags.DEFINE_string('root_path', '', "Path where to look for relevant files")
gflags.DEFINE_bool('mixed', False, "if true, mixed is used")

def powerset(iterable):
  xs = list(iterable)
  return itertools.chain.from_iterable( itertools.combinations(xs,n) for n in range(len(xs)+1) )

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

def addVariablesDepl(m, G, TERMINALS, X, F, edges, in_arcs, out_arcs):

    for v1 in range(G.shape[0]):
        for v2 in range(v1 + 1, G.shape[0]):
            if (G[v1,v2] == 1): #edge
                X[v1, v2] = m.addVar(vtype=GRB.CONTINUOUS, name="x[%d,%d]" %(v1, v2))
                edges.append((v1,v2))
    #The first frontier is the root
    for fr in TERMINALS[1:]:
        for v1 in range(G.shape[0]):
            for v2 in range(v1+1, G.shape[0]):
                if (G[v1,v2] == 1): #arc
                    F[fr, v1, v2] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS, name="f[%d,%d,%d]" %(fr, v1, v2))
                    F[fr, v2, v1] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS, name="f[%d,%d,%d]" %(fr, v2, v1))

    #print edges

    for v1 in range(G.shape[0]):
        in_arcs[v1] = []
        out_arcs[v1] = []
        for v2 in range(G.shape[0]):
            if (G[v1,v2] == 1 and v1 != v2):
                out_arcs[v1].append((v1,v2))         
                in_arcs[v1].append((v2,v1))

    #print in_arcs
    #print out_arcs

def addConstraintsDepl(m, G, TERMINALS, X, F, edges, in_arcs, out_arcs):

    for fr in TERMINALS[1:]:
        
        for edge in edges:
            m.addConstr(F[fr,edge[0],edge[1]]<= X[edge[0], edge[1]])
            m.addConstr(F[fr,edge[1],edge[0]]<= X[edge[0], edge[1]])
    
        for i in range(G.shape[0]):
            #print i
            if i == TERMINALS[0]:
                m.addConstr(quicksum(F[fr,arc[0],arc[1]] for arc in out_arcs[i]) - quicksum(F[fr,arc[0],arc[1]] for arc in in_arcs[i]) == 1)
            elif i == fr:
                m.addConstr(quicksum(F[fr,arc[0],arc[1]] for arc in out_arcs[i]) - quicksum(F[fr,arc[0],arc[1]] for arc in in_arcs[i]) == -1)
            else:
               m.addConstr(quicksum(F[fr,arc[0],arc[1]] for arc in out_arcs[i]) - quicksum(F[fr,arc[0],arc[1]] for arc in in_arcs[i]) == 0)

    #for edge in edges:
    #    m.addConstr(quicksum(F[fr,edge[0],edge[1]] + F[fr,edge[1],edge[0]] for fr in FRONTIERS[1:]) <= 1)

def setObjectiveDepl(m, G, X, edges):
    #print NORM
    m.setObjective(quicksum(X[v1,v2] for (v1,v2) in edges), sense = GRB.MINIMIZE)

def solve_relaxed_steiner(G, TERMINALS, max_dim):
    F = {}
    X = {}
    edges = []
    in_arcs = {}
    out_arcs = {}
    
    m = Model('Steiner_tree_relaxed')

    addVariablesDepl(m, G, TERMINALS, X, F, edges, in_arcs, out_arcs)
    m.update()
    addConstraintsDepl(m, G, TERMINALS, X, F, edges, in_arcs, out_arcs)

    setObjectiveDepl(m, G, X, edges)
    
    m.setParam( 'OutputFlag', False)

    m.setParam("TimeLimit", 1)
    #m.setParam("Threads", 1)
    m.setParam("Presolve", 2) #aggressive
    #m.setParam("MIPFocus", 1)
    m.update()

    m.optimize()

    if m.status == GRB.status.INF_OR_UNBD or m.status == GRB.status.INFEASIBLE:
        print "Model infeasible or unbounded."
        exit(1)

    elif m.status == GRB.status.OPTIMAL:
        return m.ObjVal

    elif m.status == GRB.status.TIME_LIMIT:
        return 0;

    else:
        print "Strange status returned."
        exit(1)


def compute_obj(solution, frontiers, utility):
    nodes_sol = list(itertools.chain.from_iterable(solution))
    obj = 0.0
    for f in frontiers:
        if f in nodes_sol:
            obj += utility[frontiers.index(f)]
    return obj   	

def compute_obj_est(candidates, frontiers, utility):
   obj = 0.0
   for c in candidates:
       obj += utility[frontiers.index(c)]

   return obj

def is_valid_sp(candidates, all_pair_sp, bs, max_dim):
    #print "THE MAXDIM IS " + str(max_dim)
    #print "THE BS IS " + str(bs)

    all_terminals = deepcopy(candidates)
    all_terminals.append(bs)
    for pair in itertools.combinations(all_terminals, 2):
        #print "THE PATH IS "
        #print all_pair_sp[pair]
        if(all_pair_sp[pair] is None):
            #print "RETURNING"
            return False
        cur_num = len(filter(lambda x: x != bs, all_pair_sp[pair]))
        #print "CUR NUM IS " + str(cur_num)
        if (cur_num > max_dim):
            #print "RETURNING"
            return False

    return True

def construct_tree(root, M, out_arcs, W, U):
    if M[(root, W.index(U))] == 0: 
      return []

    for arc in out_arcs[root]:
        u = arc[1]
        if(M[(root, W.index(set(U)))] == 1 + M[(u, W.index(set(U)))]):
            return [arc] + construct_tree(u, M, out_arcs, W, U)

    #Otherwise...
    Y = filter(lambda x: M[(root, W.index(U))] == M[(root, W.index(set(x)))] + M[(root, W.index(U - set(x)))], filter(lambda x: set(x) != set([]) and set(x) != U, powerset(U)))
    if len(Y):
        return construct_tree(root, M, out_arcs, W, set(Y[0])) + construct_tree(root, M, out_arcs, W, U - set(Y[0]))  
    else:
        return[]

def compute_optimal_steiner(G, in_arcs, out_arcs, U, max_dim):

    M = {}
    W = []
    W_it = powerset(U)
    
    for el in W_it:
        W.append(set(el))

    #Each subset is identified by its index in W
    for i in range(G.shape[0]):
        for w in W:
            M[(i, W.index(w))] = float('inf')
        M[(i, W.index(set([])))] = 0
    
    for u in U:
           M[(u, W.index(set([u])))] = 0

    for w in W:
        Q_w = priorityDictionary()
        for i in range(G.shape[0]):
            M[(i, W.index(w))] = min(map(lambda x: M[(i, W.index(set(x)))] + M[(i, W.index(w - set(x)))], powerset(w)))

            Q_w[i] = M[(i, W.index(w))]

        for v_min in Q_w:
            for arc in in_arcs[v_min]:
                v = arc[0]
                if(M[(v, W.index(w))] > M[(v_min, W.index(w))] + 1):
                    M[(v, W.index(w))] = M[(v_min, W.index(w))] + 1
                    Q_w[v] = M[(v, W.index(w))]

        #I can prune the search here if, for a given subset, the size of the minimum tree exceeds max_dim
        below_bound = False
        for i in range(G.shape[0]):
            if(M[(i, W.index(w))] <= max_dim):
                below_bound = True
                break

        if(not(below_bound)):
            return []

    root = 0
    for v in range(1, G.shape[0]):
        if (M[(v, W.index(set(U)))] < M[(root, W.index(set(U)))]):
          root = v


    solution = construct_tree(root, M, out_arcs, W, set(U))
    #print "W:"
    #print W
    #print "M:"
    #print M
    return solution

def search_recursive(candidates, included, bs, utility, max_dim, z_max, best_sol, all_pair_sp, G, in_arcs, out_arcs, size):
    if len(included) == size: return

    if(not(len(included))): cur_candidates = deepcopy(candidates)
    else:
        cur_candidates = filter(lambda x: x not in included and candidates.index(x) > candidates.index(included[-1]), candidates)
    #examine the candidates in order. If a set of frontier has already been expanded (e.g. A,B,C, from A,C the set A,C,B will not be considered.)

    for candidate in cur_candidates:
        
        new_candidates = included + [candidate]
        #new_candidates.sort()
        
        #Here I check If I have the potential to beat the current best sol
        estimate = compute_obj_est(new_candidates, candidates, utility)
        if estimate > z_max[0]:

            if not(is_valid_sp(new_candidates, all_pair_sp, bs, max_dim)):
                #print new_candidates
                #print "NOT VALID FOR SHORTEST PATH BOUND"
                
                continue

            lower_bound = solve_relaxed_steiner(G, [bs] + new_candidates, max_dim)
            if lower_bound > max_dim:
                #print "Lower bound too high."
                continue

            solution = compute_optimal_steiner(G, in_arcs, out_arcs, [bs] + new_candidates, max_dim)
            if(not(len(solution)) or len(solution) > max_dim):
                #print "The computation was interrupted, or the obtained tree is too big."
                
                continue
            #If I arrive here I check if I can update the best sol
            z_new = compute_obj(solution, candidates, utility)
            #print "Found solution with objective: " + str(z_new)          
               
            #Workaround does not work with global variables
            if z_new > z_max[0]:
                z_max.pop()
                z_max.append(z_new)
                best_sol.pop()
                best_sol.append(solution)
                #print "The new objective is: " + str(z_new)
                #print new_candidates
                    

        else:
            #print "The estimate " + str(estimate) + " is not enough."
            pass

        search_recursive(candidates, new_candidates, bs, utility, max_dim, z_max, best_sol, all_pair_sp, G, in_arcs, out_arcs, size)

def pad_solution(sol, nodes, num_available, all_pair_sp, frontiers):

    free_frontiers = deepcopy(frontiers)
    free_frontiers = filter(lambda x: x not in nodes, free_frontiers)
    new_frontiers = []

    #print "NODES"
    #print nodes

    #print "FREE FRONTIERS"
    #print free_frontiers
    #add greedily shortest paths to currently not covered frontiers
    while(num_available > 0 and len(free_frontiers)):
        cur_frontier = free_frontiers.pop(0)
        #print cur_frontier
        #print num_available
        min_cost = -1
        best_path = None
        for node in nodes:
            path = all_pair_sp[(node, cur_frontier)]
            #print node
            if path == None: continue

            cost = len(filter(lambda x: x not in nodes, path))
            if min_cost == -1 or cost < min_cost:
                min_cost = cost
                best_path = path

        #print min_cost
        #print best_path
        if min_cost != -1 and min_cost <= num_available:
            num_available -= len(filter(lambda x: x not in nodes, best_path))
            new_nodes = filter(lambda x: x not in nodes, best_path)
            nodes += new_nodes
            edges = zip(best_path[:-1], best_path[1:])
            for edge in edges:
                if (edge not in sol[0] and (edge[1], edge[0]) not in sol[0]): 
                    sol[0].append(edge)
            for nn in new_nodes:
                if nn in frontiers:
                    new_frontiers.append(nn)

        #print nodes

    #print "New frontiers"
    #print new_frontiers
    return new_frontiers

def allocate_remaining_robots(allocation, nodes, out_arcs, DIST, CUR_LOC, NOT_READY_COMM, dist_not_ready_comm):
    print "Allocating remaining robots"
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
    print "Allocating robots to current nodes"

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
        #print "Examining robot " + str(robot) + " at distance " + str(distance_dict[robot])
        #create a copy of the dictionaries containing only the robots with distance <= dist[robot]
        r_n_dict_temp = {}
        n_r_dict_temp = {}
        distance_dict_temp = {}
        for robot2 in distance_dict:
            if distance_dict[robot2] <= distance_dict[robot]:
                distance_dict_temp[robot2] = distance_dict[robot2]
                r_n_dict_temp[robot2] = r_n_dict[robot2]
                n_r_dict_temp[r_n_dict[robot2]] = robot2

        #create a graph restricted to the robots (nodes) in temp
        G = nx.Graph()
        for node in n_r_dict_temp:
            G.add_node(node)
        G.add_node(CUR_LOC[0]) #bs

        for n1 in n_r_dict_temp:
            for n2 in n_r_dict_temp:
                if(n1 != n2 and G_ORIG[n1,n2]==1):
                    G.add_edge(n1,n2)

        for n in n_r_dict_temp:
            if(G_ORIG[CUR_LOC[0], n] == 1):
                G.add_edge(n,CUR_LOC[0])
                G.add_edge(CUR_LOC[0],n)

        if(not(frontier in nx.descendants(G, source=CUR_LOC[0]))): continue #skip this config

        #since the robots are sorted by increasing distance order, as soon as I find a feasible path I can stop the search
        return nx.shortest_path(G, source=frontier, target=CUR_LOC[0])

    print "FATAL ERROR!"
    return None


def convert_path_nodes_robots(frontier_paths, frontier, allocation, currentTreeOnlyNotComm):
    robots_list = []
    for relay in frontier_paths[frontier]:
        found = False
        for el in allocation:
            if(el[1] == relay):
                robots_list.append(el[0]) #this is the actual hashmap used in java
                found = True
                break

        if(not found):
            for el in currentTreeOnlyNotComm:
                if(el[1] - 1 == relay):
                    robots_list.append(el[0])
                    break
    return robots_list


def run():
    my_dir = ""#"/home/banfi/NetBeansProjects/MRESim-master_4.0/"
    MIXED = gflags.FLAGS.mixed
    
    #LAMBDA = 1.0/20.0
    #LAMBDA = 1000000.0

    if(MIXED):
        fm = open(gflags.FLAGS.root_path + "mixApp.txt", "w")
        fm.write("COMPUTING")
        fm.close()
   
    f_g = open( gflags.FLAGS.root_path + "G.txt", "r")
    lines = f_g.readlines()
    s = ""
    for line in lines:
        linet = line.replace("\t"," ").replace("\n","").replace("\r","")
	linet = linet[:len(linet)-1]
	#print linet
	s += linet + ";"
    s = s[:len(s)-1]
    f_g.close()

    #print s
    G_ORIG = np.matrix(s)
    #print G_ORIG

    f_m1 = open(gflags.FLAGS.root_path + "m_1.txt", "r")
    lines = f_m1.readlines()
    replanning_robots = []
    CUR_LOC = {}
    bs = -1
    for i in range(len(lines)):
        line = lines[i]
        s = line.split()
        CUR_LOC[ int(s[0]) ] = int(s[1]) - 1

        #this will contain also the BS
        replanning_robots.append( int(s[0]))
        if(i == 0):
            bs = int(s[1]) - 1
            bs_key = int(s[0])
    #print CUR_LOC
    f_m1.close()

    CUR_DISTANCES_NR = {}
    CUR_LOC_NCOMM_DEST = {}

    NOT_READY_COMM = []
    if(os.path.exists(gflags.FLAGS.root_path + "notReadyDist.txt")):
       f_nrc = open(gflags.FLAGS.root_path + "notReadyDist.txt", "r")
       lines = f_nrc.readlines()
       for line in lines:
           s = line.split()
           if(s[0] != "COMM"):
               CUR_DISTANCES_NR[int(s[0])] = map(lambda x: int(x), s[1:])
           else:
               NOT_READY_COMM= map(lambda x: int(x), s[1:])

    #physical distances
    f_wp = open(gflags.FLAGS.root_path + "w_p.txt","r")
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

    currentTree = []
    if(os.path.exists(gflags.FLAGS.root_path + "currentTree.txt")):
        #fill currentTree, the list of the current Tree initially without the replanning robots
        fct = open(gflags.FLAGS.root_path + "currentTree.txt" , "r")

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
        G = G_ORIG

    print "CURRENT TREE:"
    print currentTree  

    print "COMM NODES TO REALLOCATE:"
    print comm_nodes_to_reallocate

    print "CURRENT DESTINATIONS OF NON COMM/NON READY ROBOTS:"
    print CUR_LOC_NCOMM_DEST

    #DEBUG
    """nods = comm_nodes_to_reallocate + [CUR_LOC[0]]
    print "NODS"
    print nods
    G_B = nx.Graph()
    for v1 in nods:
        G_B.add_node(v1)

    for v1 in nods:
        for v2 in nods:
            if(v1 != v2 and G_ORIG[v1,v2] == 1):
                G_B.add_edge(v1,v2)

    nod = 87
    if(nod in nx.descendants(G_B, source = CUR_LOC[0])):
        print "ESISTE"
    exit(1)"""
    #END DEBUG

    f_f = open(gflags.FLAGS.root_path + "F.txt", "r")
    lines = f_f.readlines()
    FRONTIERS = np.matrix(lines[0])
    FRONTIERS = FRONTIERS.tolist()[0]
    FRONTIERS = map(lambda x: x - 1,FRONTIERS)
    f_f.close()

    f_fa = open(gflags.FLAGS.root_path + "FA.txt", "r")
    lines = f_fa.readlines()
    F_AREAS = np.matrix(lines[0])
    F_AREAS = F_AREAS.tolist()[0]
    f_fa.close()

    #finally, load also the current forming frontiers (will be used to recompute the first forming path)
    f_old_frontiers = open(gflags.FLAGS.root_path + "currWaitingList.txt", "r")
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

    #Added
    all_pair_sp = {}
    G_B = nx.Graph()
    for v1 in range(G.shape[0]):
        G_B.add_node(v1)

    for v1 in range(G.shape[0]):
        for v2 in range(G.shape[0]):
            if(v1 != v2 and G[v1,v2] == 1):
                G_B.add_edge(v1,v2)

    for v1 in range(G.shape[0]):
        for v2 in range(v1, G.shape[0]):
            if(not(v1 in nx.descendants(G_B, source=v2))):
                all_pair_sp[(v1,v2)] = None
                all_pair_sp[(v2,v1)] = None

            else:
                all_pair_sp[(v1,v2)] = nx.shortest_path(G_B, source=v1, target=v2)
                all_pair_sp[(v2,v1)] = nx.shortest_path(G_B, source=v2, target=v1)

    #print "shortest path"
    #print all_pair_sp[(4,30)]
    #exit(1)
    #solve_current_steiner(G, FRONTIERS + [CUR_LOC[0]])
    true_frontiers = filter(lambda x: x != bs, FRONTIERS)

    #no frontiers taken with all robots ready
    if(len(true_frontiers) == 0):
        if(MIXED):
            fcmin = open(gflags.FLAGS.root_path + "costMinApp.txt", "w")
            fcmin.write("Inf")
            fcmin.close()
            fm = open(my_dir + "mixApp.txt", "w")
            fm.write("DONE")
            fm.close()
        else:
            fcmin = open(gflags.FLAGS.root_path + "costMin.txt", "w")
            fcmin.write("Inf")
            fcmin.close()      
        
        return 

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

    f_a = open(gflags.FLAGS.root_path + "approx.txt", "r")
    lines = f_a.readlines()
    
    SIZE = float(lines[0].split()[0])

    f_a.close()

    print "Utilities:"
    print utilities
    print "Frontiers:"
    print true_frontiers

    #print true_frontiers
    #print utilities

    z_best = [0]
    sol = [[]]

    #Precompute arcs
    in_arcs = {}
    out_arcs = {}

    #Counting edges for logging
    edges = []

    for v1 in range(G.shape[0]):
        in_arcs[v1] = []
        out_arcs[v1] = []
        for v2 in range(G.shape[0]):
            if (G[v1,v2] == 1 and v1 != v2):
                out_arcs[v1].append((v1,v2))         
                in_arcs[v1].append((v2,v1))

                if(v2 > v1):
                    edges.append((v2,v1))
    #print in_arcs
    #print out_arcs

    search_recursive(true_frontiers, [], bs, utilities, len(CUR_LOC) - 1, z_best, sol, all_pair_sp, G, in_arcs, out_arcs, SIZE)

    print "The best obj. found is: " + str(z_best[0])
    
    nodes = []
    selected_frontiers = []
    for edge in sol[0]:
        if not(edge[0]) in nodes and edge[0] != bs:
            nodes.append(edge[0])
            if((edge[0]) in true_frontiers and edge[0] not in selected_frontiers):
                selected_frontiers.append(edge[0])
        if not(edge[1]) in nodes and edge[1] != bs:
            nodes.append(edge[1])
            if((edge[1]) in true_frontiers and edge[1] not in selected_frontiers):
                selected_frontiers.append(edge[1])

    if(len(selected_frontiers) == 0 and len(CUR_DISTANCES_NR) == 0):
        #no frontiers taken with all robots ready
        if(MIXED):
            fcmin = open(gflags.FLAGS.root_path + "costMinApp.txt", "w")
            fcmin.write("Inf")
            fcmin.close()
            fm = open(my_dir + "mixApp.txt", "w")
            fm.write("DONE")
            fm.close()
        else:
            fcmin = open(gflags.FLAGS.root_path + "costMin.txt", "w")
            fcmin.write("Inf")
            fcmin.close()      
        
        return     

    print "#Solution before padding#"
    print "Taken nodes:"
    print nodes
    print "Edges:"
    print sol
    if len(nodes) < len(CUR_LOC) - 1:
        nodes = nodes + [bs]
        new_frontiers = pad_solution(sol, nodes, (len(CUR_LOC) - 1) - (len(nodes) - 1), all_pair_sp, true_frontiers)
        selected_frontiers += new_frontiers        
        nodes.remove(bs)

    print "#Solution after padding#"
    print "All nodes:"
    print nodes
    print "Selected frontiers:"
    print selected_frontiers
    print "Edges:"
    print sol
    
    CUR_LOC_COPY = deepcopy(CUR_LOC)
    del(CUR_LOC_COPY[bs_key])

    allocation = []
    all_nodes = nodes + comm_nodes_to_reallocate

    #metto anche i comm not ready qua. devo allocare sia i nuovi nodi che i vecchi che erano coperti da dei robot in comm.
    if(len(all_nodes) > 0):
        allocation = allocate_robots_min_cost(allocation, all_nodes, DIST, CUR_LOC_COPY, NOT_READY_COMM, CUR_DISTANCES_NR)

    print "ALLOCATION:"
    print allocation

    #It can also happen that the optimal solution does not saturate the robots. In that case, use the hungarian algorithm
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
        fmopt = open(gflags.FLAGS.root_path + "m_optApp.txt", "w")        
    else:
        fmopt = open(gflags.FLAGS.root_path + "m_opt.txt", "w")
    fmopt.write(s)
    fmopt.close()

    #in any case the new currenTree is constructed like this
    newTree = deepcopy(currentTreeOnlyNotComm)
    newTree += allocation
    newTree = sorted(newTree, key=lambda x:x[0])
    s = "0" + "\t" + str(bs + 1) + "\n"

    for c in newTree:
        s += str(c[0]) + "\t" + str(c[1] + 1) + "\n"
    if(MIXED):
        fct = open(gflags.FLAGS.root_path + "currentTreeApp.txt", "w")
    else:
        fct = open(gflags.FLAGS.root_path + "currentTree.txt", "w")
    fct.write(s)
    fct.close()

    if(MIXED):
        fcmin = open(gflags.FLAGS.root_path + "costMinApp.txt", "w")
    else:
        fcmin = open(gflags.FLAGS.root_path + "costMin.txt", "w")
        
    fcmin.write("OK")
    fcmin.close()

    total_frontiers = selected_frontiers + old_frontiers
    frontier_paths = {}
    for frontier in total_frontiers:
        frontier_paths[frontier] = compute_min_path(G_ORIG, frontier, DIST, allocation_n_r_dict, allocation_r_n_dict, CUR_LOC, CUR_DISTANCES_NR, CUR_LOC_NCOMM_DEST, NOT_READY_COMM)

  
    print "FRONTIER PATHS NODES:"
    print frontier_paths

    frontier_paths_robots = {}
    robots_covered = set()
    for frontier in frontier_paths:
        path_robots = convert_path_nodes_robots(frontier_paths, frontier, allocation, currentTreeOnlyNotComm)
        frontier_paths_robots[frontier] = path_robots
        robots_covered = robots_covered.union(set(path_robots))
    
    print "FRONTIER PATHS ROBOTS:"
    print frontier_paths_robots        

    print "ADDING AS FAKE FRONTIERS ALSO LONELY ROBOTS"
    #Finally, write also the updated waiting list.
    for n in newTree:
        robot = n[0]
        if robot not in robots_covered:
            frontier_paths[n[1]] = compute_min_path(G_ORIG, n[1], DIST, allocation_n_r_dict, allocation_r_n_dict, CUR_LOC, CUR_DISTANCES_NR, CUR_LOC_NCOMM_DEST, NOT_READY_COMM)
            path_robots = convert_path_nodes_robots(frontier_paths, n[1], allocation, currentTreeOnlyNotComm)
            frontier_paths_robots[n[1]] = path_robots
            robots_covered = robots_covered.union(set(path_robots))

    print "NEW FRONTIER PATHS ROBOTS:"
    print frontier_paths_robots

    if(MIXED):
        fw = open(gflags.FLAGS.root_path + "waitingApp.txt", "w")
    else:
        fw = open(gflags.FLAGS.root_path + "waiting.txt", "w")   
    
    for fr in frontier_paths_robots: #contains also the old ones
        s = ""
        s += str(fr) #+1?
        for robot in frontier_paths_robots[fr]:
            s += "\t" + str(robot) #this is the actual hashmap in java

        s += "\n"
        fw.write(s)

    fw.close()

    #write also the dimension of the solved instance (for logging)
    fdim = open(gflags.FLAGS.root_path + "dim.txt", "w")
    fdim.write(str(len(out_arcs)) + " " + str(len(edges)) + " " + str(len(CUR_LOC) - 1))
    fdim.close()

    if(MIXED):
        fm = open(gflags.FLAGS.root_path + "mixApp.txt", "w")
        fm.write("DONE")
        fm.close()

    #FINAL CHECK TO SEE IF THE NEW TREE IS CONNECTED
    nods = comm_nodes_to_reallocate + [CUR_LOC[0]]
    print "NODS"
    print nods
    G_B = nx.Graph()
    for v1 in nods:
        G_B.add_node(v1)

    for v1 in nods:
        for v2 in nods:
            if(v1 != v2 and G_ORIG[v1,v2] == 1):
                G_B.add_edge(v1,v2)

    nod = 87
    if(nod in nx.descendants(G_B, source = CUR_LOC[0])):
        print "ESISTE"
    exit(1)"""

if __name__ == "__main__":
    try:
        argv = gflags.FLAGS(sys.argv)  # parse flags
    except gflags.FlagsError, e:
        print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS)
        sys.exit(1)

    #try:
    run()
    """except :
        ferr = open(gflags.FLAGS.root_path + "error.txt", "r") # TODO correct file
        n = int(ferr.readlines()[0])
        ferr.close()
        os.system("cp " + gflags.FLAGS.root_path + "G.txt " + gflags.FLAGS.root_path + "algoErrors/aG_" + str(n) + ".txt")
        os.system("cp " + gflags.FLAGS.root_path + "currentTree.txt " + gflags.FLAGS.root_path + "algoErrors/acurrentTree_" + str(n) + ".txt")
        os.system("cp " + gflags.FLAGS.root_path + "notReadyDist.txt " + gflags.FLAGS.root_path + "algoErrors/anotReadyDist_" + str(n) + ".txt")
        os.system("cp " + gflags.FLAGS.root_path + "currWaitingList.txt " + gflags.FLAGS.root_path + "algoErrors/acurrWaitingList_" + str(n) + ".txt")
        os.system("cp " + gflags.FLAGS.root_path + "m_1.txt " + gflags.FLAGS.root_path + "algoErrors/am_1_" + str(n) + ".txt")
        os.system("cp " + gflags.FLAGS.root_path + "w_p.txt " + gflags.FLAGS.root_path + "algoErrors/aw_p_" + str(n) + ".txt")
        os.system("cp " + gflags.FLAGS.root_path + "F.txt " + gflags.FLAGS.root_path + "algoErrors/aF_" + str(n) + ".txt")
        os.system("cp " + gflags.FLAGS.root_path + "FA.txt " + gflags.FLAGS.root_path + "algoErrors/aFA_" + str(n) + ".txt")

        ferr = open(gflags.FLAGS.root_path + "error.txt", "w")
        ferr.write(str(n + 1))
        ferr.close()"""
    
