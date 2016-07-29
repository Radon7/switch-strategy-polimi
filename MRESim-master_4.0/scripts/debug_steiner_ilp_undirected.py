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

def lazy_constraint(model, where):
    MAXCUTS = 10    
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
            if i == bs: continue
            cut_number = 0
            added_constraint = True
            
            while cut_number < MAXCUTS and added_constraint:
                added_constraint = False

                #print "ESAMINO NODO " + str(i)
                #print Y[i]
            
                
                G_W = nx.Graph()
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
                                if((node,j) in edges):
                                    cut_set.add((node,j))
                                else:
                                    cut_set.add((j,node))
                    expr = 0.0
                    #print "CUT SET"
                    #print cut_set
                    
                    for edge in cut_set:
                        #print edge
                        X_W[(edge[0], edge[1])] == 1.0
                        expr += X[edge[0], edge[1]]

                    model.cbLazy(expr >= Y[i])                   
        

def addVariablesDepl(m, G, TERMINALS, X, Y, edges, out_arcs):

    for v in range(G.shape[0]):
        if v == TERMINALS[0]: continue
        Y[v] = m.addVar(vtype=GRB.BINARY, name="y[%d]" %(v))
            
    for v1 in range(G.shape[0]):
        for v2 in range(v1 + 1, G.shape[0]):
            if v1 == v2: continue

            if (G[v1,v2] == 1): #edge
                X[v1, v2] = m.addVar(vtype=GRB.BINARY, name="x[%d,%d]" %(v1,v2))
                edges.append((v1,v2))

    #for v in range(G.shape[0]):
    #    if(G[TERMINALS[0], v] == 1):
    #        X[TERMINALS[0], v] = m.addVar(vtype=GRB.BINARY, name="x[%d,%d]" %(v1,v2))
    #        edges.append((TERMINALS[0], v))

    for v1 in range(G.shape[0]):
        out_arcs[v1] = []
        for v2 in range(G.shape[0]):
            if (G[v1,v2] == 1 and v1 != v2):
                out_arcs[v1].append((v1,v2))        


def addConstraintsDepl(m, G, TERMINALS, X, Y, edges, max_dim):
    #The BS is at a fixed position
    #m.addConstr(R[0, CUR_LOC[0]] == 1)
    #m.addConstr(quicksum(X[edge[0], edge[1]] for edge in filter(lambda x: x[0] == TERMINALS[0], edges)) >= 1)    

    #m.addConstr(Y[6] == 1)

    #Mapping X-Y variables
    for v in range(G.shape[0]):
        if v == TERMINALS[0]: continue 
        #m.addConstr(Y[v] <= 1)      
        #m.addConstr(quicksum(X[edge[0], edge[1]] for edge in filter(lambda x: x[1] == v, edges)) == Y[v])

    #The number of edges is at most NROBOTS - 1

    m.addConstr(quicksum(X[edge[0], edge[1]] for edge in edges) <= max_dim)

    #Optional constraints...
    #for v in range(G.shape[0]):
        #in degree <= out degree for non terminal nodes.
        #if v in TERMINALS: continue
        #m.addConstr(quicksum(X[edge[0], edge[1]] for edge in filter(lambda x: x[1] == v, edges)) <= quicksum(X[edge[0], edge[1]] for edge in filter(lambda x: x[0] == v, edges)))

    #for v in range(G.shape[0]):
    #    if v == TERMINALS[0]: continue
    #    for edge in filter(lambda x: x[0] == v, edges):
    #        m.addConstr(X[edge[0],edge[1]] + X[edge[1], edge[0]] <= Y[v])

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
    m._frontiers = true_frontiers
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

        for v in range(G.shape[0]):
            if v == bs: continue
            if(y[v] > 0):
                print "vertex " + str(v) + " considered"

        for fr in true_frontiers:
            if (y[fr] > 0.5):
                selected_frontiers.append(fr)

        #print "THE SOLUTION IS:"
        #print solution
        return ([solution], selected_frontiers)



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

    true_frontiers = [9,8]
    utilities = [90,80]
    bs = 3

    sol,a = solve_bpcst(true_frontiers, bs, utilities, 3, G, out_arcs)

    print sol
