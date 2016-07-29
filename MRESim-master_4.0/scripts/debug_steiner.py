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
        print "adding robot " + str(robot) + " with id in map " + str(robot_matrix_index)
        if(not(robot in map(lambda x: x[0], allocation))):
            matrix.append(map(lambda x: DIST[x, CUR_LOC[robot]], candidate_positions))
            robot_ids_map[robot_matrix_index] = robot
            robot_matrix_index += 1

    for robot in NOT_READY_COMM:
        print "adding robot " + str(robot) + " with id in map " + str(robot_matrix_index)
        if(not(robot in map(lambda x: x[0], allocation))):
            matrix.append(map(lambda x: dist_not_ready_comm[robot][x], candidate_positions))
            
            robot_ids_map[robot_matrix_index] = robot
            robot_matrix_index += 1

    print robot_ids_map
    print candidate_positions
    print matrix
    
    m = Munkres()
    indexes = m.compute(matrix)
    #print_matrix(matrix, msg='Lowest cost through this matrix:')
    for row, column in indexes:
        #print column
        new_allocation.append((robot_ids_map[row], candidate_positions[column]))
        #new_allocation.append((robot_ids_map[column], candidate_positions[row]))
        #fake_frontiers.append(candidate_positions[column])
    
    print new_allocation
    return new_allocation

def powerset(iterable):
  xs = list(iterable)
  return itertools.chain.from_iterable( itertools.combinations(xs,n) for n in range(len(xs)+1) )

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

def compute_obj(solution, frontiers, utility):
    print "Computing objective"
    nodes_sol = set(list(itertools.chain.from_iterable(solution)))
    print nodes_sol
    obj = 0.0
    for f in frontiers:
        if f in nodes_sol:
            obj += utility[frontiers.index(f)]
    return obj   	

def compute_obj_est(candidates, frontiers, utility):
    print "Computing objective estimate"
    obj = 0.0
    for c in candidates:
        obj += utility[frontiers.index(c)]

    print obj
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

def search_recursive(candidates, included, bs, utility, max_dim, z_max, best_sol, all_pair_sp, G, in_arcs, out_arcs, size):
    if len(included) == size: return

    if(not(len(included))): cur_candidates = deepcopy(candidates)
    else:
        cur_candidates = filter(lambda x: x not in included and candidates.index(x) > candidates.index(included[-1]), candidates)
    #examine the candidates in order. If a set of frontier has already been expanded (e.g. A,B,C, from A,C the set A,C,B will not be considered.)

    for candidate in cur_candidates:
        
        new_candidates = included + [candidate]
        #new_candidates.sort()

        print "Examining: " 
        print new_candidates
        
        #Here I check If I have the potential to beat the current best sol
        estimate = compute_obj_est(new_candidates, candidates, utility)
        if estimate > z_max[0]:

            if not(is_valid_sp(new_candidates, all_pair_sp, bs, max_dim)):
                #print new_candidates
                print "NOT VALID FOR SHORTEST PATH BOUND"
                
                continue

            lower_bound = solve_relaxed_steiner(G, [bs] + new_candidates, max_dim)
            if lower_bound > max_dim:
                print "Lower bound too high."
                continue
            else:
                print "The lower bound is "+ str(lower_bound)

            solution = compute_optimal_steiner(G, in_arcs, out_arcs, [bs] + new_candidates, max_dim)
            if(not(len(solution)) or len(solution) > max_dim):
                print "The computation was interrupted, or the obtained tree is too big."
                
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
                print "Found the new best objective: " + str(z_new)
                #print new_candidates
                    

        else:
            print "The estimate " + str(estimate) + " is not enough."
            pass

        search_recursive(candidates, new_candidates, bs, utility, max_dim, z_max, best_sol, all_pair_sp, G, in_arcs, out_arcs, size)

def pad_solution(sol, nodes, num_available, all_pair_sp, frontiers):

    free_frontiers = deepcopy(frontiers)
    free_frontiers = filter(lambda x: x not in nodes, free_frontiers)
    new_frontiers = []

    print "NODES"
    print nodes

    #if DEBUG:
    print "FREE FRONTIERS"
    print free_frontiers
    #add greedily shortest paths to currently not covered frontiers
    while(num_available > 0 and len(free_frontiers)):
        cur_frontier = free_frontiers.pop(0)
        #if DEBUG:
        print "Cur frontier"
        print cur_frontier
        print "robots available"
        print num_available
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

        #if DEBUG:
        print "min cost"
        print min_cost
        print "best path"
        print best_path
        if min_cost != -1 and min_cost <= num_available:
            num_available -= len(filter(lambda x: x not in nodes, best_path))
            new_nodes = filter(lambda x: x not in nodes, best_path)
            print "new nodes"
            print new_nodes
            nodes += new_nodes

            print "all potential edges"
            edges = zip(best_path[:-1], best_path[1:])
            print edges
            for edge in edges:
                if (edge not in sol[0] and (edge[1], edge[0]) not in sol[0]): 
                    sol[0].append(edge)
                    print "added edge"
                    print edge

            for nn in new_nodes:
                if nn in frontiers:
                    new_frontiers.append(nn)

        #if DEBUG:
        #    print nodes

    print "New frontiers"
    print new_frontiers
    return new_frontiers


if __name__ == "__main__":

    G = np.zeros((10,10))
    G[0,1] = 1
    G[0,4] = 1
    G[0,5] = 1
    G[1,0] = 1
    G[4,0] = 1
    G[5,0] = 1

    G[1,9] = 1
    G[1,4] = 1
    G[4,1] = 1
    G[9,1] = 1

    G[2,4] = 1
    G[2,5] = 1
    G[4,2] = 1
    G[5,2] = 1

    G[3,7] = 1
    G[3,6] = 1
    G[7,3] = 1
    G[6,3] = 1

    G[4,5] = 1
    G[4,7] = 1
    G[5,4] = 1
    G[7,4] = 1

    G[6,7] = 1
    G[7,6] = 1

    G[7,8] = 1
    G[8,7] = 1

    G[2,9] = 1
    G[9,2] = 1

    G[1,3] = 1
    G[3,1] = 1

    #Precompute arcs
    in_arcs = {}
    out_arcs = {}

    for v1 in range(G.shape[0]):
        in_arcs[v1] = []
        out_arcs[v1] = []
        for v2 in range(G.shape[0]):
            if (G[v1,v2] == 1 and v1 != v2):
                out_arcs[v1].append((v1,v2))         
                in_arcs[v1].append((v2,v1))

    terminals = [9,8,6]
    utilities = [90,80,60]
    bs = 3

    #sol = compute_optimal_steiner(G,in_arcs,out_arcs,terminals,4)

    z_best = [0]
    sol = [[]]

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

    search_recursive(terminals, [], bs, utilities, 5, z_best, sol, all_pair_sp, G, in_arcs, out_arcs, 1)

    nodes = []
    selected_frontiers = []
    for edge in sol[0]:
        if not(edge[0]) in nodes and edge[0] != bs:
            nodes.append(edge[0])
            if((edge[0]) in terminals and edge[0] not in selected_frontiers):
                selected_frontiers.append(edge[0])
        if not(edge[1]) in nodes and edge[1] != bs:
            nodes.append(edge[1])
            if((edge[1]) in terminals and edge[1] not in selected_frontiers):
                selected_frontiers.append(edge[1]) 

    print "Occupied nodes:"
    print nodes
    print "Selected frontiers:"
    print selected_frontiers
    print "Solution:"
    print sol[0]

    nodes = nodes + [bs]
    new_frontiers = pad_solution(sol, nodes, 5 - (len(nodes) - 1), all_pair_sp, terminals)
    selected_frontiers += new_frontiers        
    nodes.remove(bs)

    print "Solution after padding:"
    print "All nodes:"
    print nodes
    print "Selected frontiers:"
    print selected_frontiers
    print "Edge solution:"
    print sol

    comm_nodes_to_reallocate = [1]

    allocation = []
    all_nodes = nodes + comm_nodes_to_reallocate
    CUR_LOC_COPY = {}
    #CUR_LOC_COPY[1] = 1
    CUR_LOC_COPY[12] = 2
    CUR_LOC_COPY[13] = 3
    CUR_LOC_COPY[14] = 4
    CUR_LOC_COPY[15] = 5

    NOT_READY_COMM = [1]
    CUR_LOC_NR = {}
    CUR_LOC_NR[1] = range(10)

    DIST = np.ones((10,10))

    allocation = allocate_robots_min_cost(allocation, all_nodes, DIST, CUR_LOC_COPY, NOT_READY_COMM, CUR_LOC_NR)

    print "ALLOCATION"
    print allocation
