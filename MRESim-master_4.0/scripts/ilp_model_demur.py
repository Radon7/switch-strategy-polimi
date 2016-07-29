from gurobipy import *
import numpy as np
import itertools
from priodict import priorityDictionary
from copy import deepcopy
import math
import networkx as nx
#TODO: change with IGraph!
import sys

def powerset(lst):
    return reduce(lambda result, x: result + [subset + [x] for subset in result],
                  lst, [[]])

def lazy_constraint(model, where):
    MAXCUTS = 1
    if((where == GRB.callback.MIPNODE and GRB.status.OPTIMAL == model.cbGet(GRB.callback.MIPNODE_STATUS) ) or where == GRB.callback.MIPSOL):
        X_W = {}
        V_VAL = {}
        G = model._G
        CUR_LOC = model._CUR_LOC
        X = model._X
        Y = model._Y
        edges = model._edges
        G_W = model._G_W

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
            if i == CUR_LOC[0]: continue
            if(where == GRB.callback.MIPNODE):
                y_val[i] = model.cbGetNodeRel(Y[i])
            else:
                y_val[i] = model.cbGetSolution(Y[i])

        #Only if X_W is not updated
        for edge in edges:
            G_W[edge[0]][edge[1]]['capacity'] = X_W[(edge[0],edge[1])]        

        for i in range(G.shape[0]):
            if i == CUR_LOC[0]: continue
            cut_number = 0
            added_constraint = True
            #Altrimenti qui
            
            while cut_number < MAXCUTS and added_constraint:
                added_constraint = False

                #print "ESAMINO NODO " + str(i)
                #print Y[i]

                cut_value, partition = nx.minimum_cut(G_W, CUR_LOC[0], i)

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
                        #G_W[edge[0]][edge[1]]['capacity'] = 1.0
                        expr += X[edge[0], edge[1]]

                    model.cbLazy(expr >= Y[i])                   

def addVariablesDepl(m, G, CUR_LOC, X, R, Y, edges):
    for r in CUR_LOC:
        if (r == 0): continue
        for v in range(G.shape[0]):
            if v == CUR_LOC[0]: continue
            R[r,v] = m.addVar(vtype=GRB.BINARY, name="r[%d,%d]" %(r,v))

    for v in range(G.shape[0]):
        if v == CUR_LOC[0]: continue
        Y[v] = m.addVar(vtype=GRB.BINARY, name="y[%d]" %(v))
            
    for v1 in range(G.shape[0]):
        for v2 in range(G.shape[0]):
            if v1 == v2 or v1 == CUR_LOC[0] or v2 == CUR_LOC[0]: continue

            if (G[v1,v2] == 1): #edge
                X[v1, v2] = m.addVar(vtype=GRB.BINARY, name="x[%d,%d]" %(v1,v2))
                edges.append((v1,v2))

    for v in range(G.shape[0]):
        if(G[CUR_LOC[0], v] == 1):
            X[CUR_LOC[0], v] = m.addVar(vtype=GRB.BINARY, name="x[%d,%d]" %(v1,v2))
            edges.append((CUR_LOC[0], v))


def addConstraintsDepl(m, G, X, R, Y, edges, CUR_LOC, FRONTIERS):
    #The BS is at a fixed position
    #m.addConstr(R[0, CUR_LOC[0]] == 1)
    #m.addConstr(quicksum(X[edge[0], edge[1]] for edge in filter(lambda x: x[0] == CUR_LOC[0], edges)) >= 1)    

    #Mapping X-Y variables
    for v in range(G.shape[0]):
        if v == CUR_LOC[0]: continue 
        #m.addConstr(Y[v] <= 1)      
        m.addConstr(quicksum(X[edge[0], edge[1]] for edge in filter(lambda x: x[1] == v, edges)) == Y[v])

    #Each node can contain at most one robot
    for v in range(G.shape[0]):
        if v == CUR_LOC[0]: continue  
        m.addConstr(quicksum(R[r,v] for r in filter(lambda x: x != 0, CUR_LOC)) == Y[v])

    #Each robot is placed in exactly one node
    for r in filter(lambda x: x != 0, CUR_LOC):
        m.addConstr(quicksum(R[r,v] for v in filter(lambda x: x != CUR_LOC[0], range(G.shape[0]))) == 1)
    
def setObjectiveDepl(m, G, R, U, CUR_LOC):
    m.setObjective(quicksum(quicksum(U[r][v]*R[r,v] for v in filter(lambda x: x != CUR_LOC[0], range(G.shape[0]))) for r in filter(lambda x: x != 0, CUR_LOC)), sense = GRB.MAXIMIZE)

def solve_bpcst_walloc(true_frontiers, CUR_LOC, utilities, max_dim, G):
    X = {}
    R = {}
    Y = {}

    edges = []
    in_arcs = {}
    
    m = Model('Bpcst_opt')

    addVariablesDepl(m, G, CUR_LOC, X, R, Y, edges)
    m.update()
    addConstraintsDepl(m, G, X, R, Y, edges, CUR_LOC, true_frontiers)

    setObjectiveDepl(m, G, R, utilities, CUR_LOC)
    
    m.setParam( 'OutputFlag', True )
    m.setParam("TimeLimit", 3600)
    #m.setParam("Threads", 3)
    #m.setParam("Presolve", 2) #aggressive
    #m.setParam("LazyConstraints", 1)
    #m.setParam("MIPFocus", 1)
    m.params.LazyConstraints = 1
    m._G = G
    m._R = R
    m._X = X
    m._Y = Y
    m._edges = edges
    m._CUR_LOC = CUR_LOC

    G_W = nx.DiGraph()
    for v1 in range(G.shape[0]):
        G_W.add_node(v1)

    for edge in edges:
        G_W.add_edge(edge[0],edge[1])

    m._G_W = G_W

    m.update()

    m.optimize(lazy_constraint)
    if m.status == GRB.status.INF_OR_UNBD or m.status == GRB.status.INFEASIBLE:
        return ([],[])
    
    elif m.status == GRB.status.INTERRUPTED:

        return ([],[])

    elif m.status == GRB.status.OPTIMAL or (m.status == GRB.status.TIME_LIMIT):
        solution = []

        x = m.getAttr('x', X) 
        r = m.getAttr('x', R)
        y = m.getAttr('x', Y)

        for edge in edges:
            if x[edge[0], edge[1]] > 0.2: #edge
                solution.append(edge)

        for v in range(G.shape[0]):
            if v == CUR_LOC[0]: continue
            if(y[v] > 0.2):
                print "NODE " + str(v) + " SELECTED"
            
        allocation = []
        for robot in CUR_LOC:
            if robot == 0: continue
            for fr in range(G.shape[0]):
                if fr == CUR_LOC[0]: continue
                if(r[robot, fr] > 0.5):
                    allocation.append((robot, fr))

        print "THE SOLUTION IS:"
        print solution
        print "THE ALLOCATION IS:"
        print allocation
        return (solution, allocation)

def run():
    my_dir = ""#"/home/banfi/NetBeansProjects/MRESim-master_4.0/"
    
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
    print G_ORIG

    f_m1 = open(my_dir + "m_1.txt", "r")
    lines = f_m1.readlines()
    CUR_LOC = {}
    bs = -1
    for i in range(len(lines)):
        line = lines[i]
        s = line.split()
        CUR_LOC[ int(s[0]) ] = int(s[1]) - 1
        if(i == 0):
            bs = int(s[1]) - 1
            bs_key = int(s[0])
    #print CUR_LOC

    ROBOTS = range(len(CUR_LOC))

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
    
    DIST = np.matrix(s)
    #print DIST

    f_f = open(my_dir + "F.txt", "r")
    lines = f_f.readlines()
    FRONTIERS = np.matrix(lines[0])
    FRONTIERS = FRONTIERS.tolist()[0]
    FRONTIERS = map(lambda x: x - 1,FRONTIERS)
    print FRONTIERS

    f_fa = open(my_dir + "FA.txt", "r")
    lines = f_fa.readlines()
    F_AREAS = np.matrix(lines[0])
    F_AREAS = F_AREAS.tolist()[0]

    true_frontiers = filter(lambda x: x != bs, FRONTIERS)
    if bs in FRONTIERS:
        info_gain = []
        for i in range(len(F_AREAS)):
            if i != FRONTIERS.index(bs):
                info_gain.append(F_AREAS[i])
    else:
        info_gain = F_AREAS
    
    #utilities = []                                                                                                           
    #for g in range(len(info_gain)):                                                                                          
    #    utilities.append(info_gain[g])                                                     
    
    #print true_frontiers
    #true_frontiers = sorted(true_frontiers, key = lambda x: utilities[true_frontiers.index(x)])
    #true_frontiers.reverse()
    #utilities.sort()
    #utilities.reverse()
    #print "UTILITIES"
    #print utilities

    #utility construction
    U = {}
    
    #print "UTILITY"
    #print U
    min_gain = min(info_gain)

    max_dist = -1
    for r in filter(lambda x: x != 0, CUR_LOC):
        U[r] = {}
        for v in range(G_ORIG.shape[0]):	
            length = DIST[CUR_LOC[r], v]
            U[r][v] = float(-length)
            if(length > max_dist): max_dist = length
	
    print "MAX dist is " + str(max_dist)

    NORM = float(min_gain - float(min_gain)/10.0)/float(len(filter(lambda x: x != 0, CUR_LOC))*max_dist)
    #NORM = 1/float(len(ROBOTS)*max_dist)
    for r in filter(lambda x: x != 0, CUR_LOC):
        for v in range(G_ORIG.shape[0]):
            U[r][v] = U[r][v]*NORM
	    if v in FRONTIERS:
                U[r][v] += info_gain[true_frontiers.index(v)]
    
    (sol, allocation) = solve_bpcst_walloc(true_frontiers, CUR_LOC, U, len(ROBOTS) - 1, G_ORIG)
    
    allocation = sorted(allocation, key=lambda x:x[0])
    s = "0" + "\t" + str(bs + 1) + "\n"

    all_equal = True
    for a in allocation:
        s += str(a[0]) + "\t" + str(a[1] + 1) + "\n"
        if CUR_LOC[a[0]] != a[1]: all_equal = False

    fmopt = open(my_dir + "m_opt.txt", "w")
    fmopt.write(s)
    fmopt.close()

    print "CUR_LOC"
    print CUR_LOC    

    if(not(all_equal)):
        fcmin = open(my_dir + "costMin.txt", "w")    
        fcmin.write("OK")
        fcmin.close()
    else: #end
        fcmin = open(my_dir + "costMin.txt", "w")    
        fcmin.write("Inf")
        fcmin.close

if __name__ == "__main__":
    run()
    os.system("pkill " + sys.argv[0])
