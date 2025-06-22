import numpy as np
from graph_wrapper import PyGraph, PyLabelManager
from gurobipy import Model, GRB, quicksum as qsm
# Define problem parameters
n = 30        # number of nodes
m = 5              # number of resources
r_max = np.array([24]*m, dtype=np.float64)  # maximum resources for each type

# Create resource tensor (n x n x (m+1)) where [:,:,0] = cost, [:,:,1:] = resources
r = np.random.randint(2,9, size=(n, n, m + 1))  # Random costs and resources
r[:,:,0] = np.random.randint(1, 40, size=(n, n))  # Random costs
 # Set diagonal to zero for costs
# print(r)
r[:,:,0] = -r[:,:,0]  # Use negative costs for minimization
#   # Make costs symmetric
# for i in range(r.shape[2]):
#     r[:,:,i] = 0.5*(r[:,:,i] + r[:,:,i].T)  # Make resources symmetric
# r[:,:,0] = -np.ones((n, n))  # Use negative costs for minimization
np.fill_diagonal(r[:, :, 0], 0) 
# for i in range(n-1):
#     r[i,i+1,0] = -1  # Set costs for direct connections
# r[n-1,0,0] = -1  # Set cost for the last node to the first node
# print(r[:,:,0])

r = r.astype(np.float64)  # Ensure the array is of type double
# print("Resource tensor (cost and resources):")
# print(r)
# print("Graph created with the following parameters:")
# # Create the graph

# print("Graph created with the following parameters:")
# # Optional: display graph info
# print("Graph:")
# g.display()

def path_make(edges):
    path =[0]
    while len(path) <= len(edges):
        for edge in edges:
            if edge[0] == path[-1]:
                path.append(edge[1])
    return path
       

def solve_with_gurobi():
    # Create a Gurobi model
    model = Model("ResourceAllocation")
    model.setParam('OutputFlag', 0)  # Suppress output
    X = [(i,j) for i in range(n) for j in range(n) if i != j]  # All pairs of nodes
    # Create variables for each solution
    x = model.addVars(X, vtype=GRB.BINARY, name="x")
    u = model.addVars(range(1,n), vtype=GRB.CONTINUOUS, name="u")  # Flow variables
    # Objective: Minimize the total cost of selected solutions
    model.setObjective(qsm(x[i,j]*r[i,j,0] for i,j in X), GRB.MINIMIZE)

    # Constraints: Ensure that resources do not exceed limits

    model.addConstrs(qsm(x[i,j]*r[i,j,k+1] for i,j in X) <= r_max[k] for k in range(m))
    model.addConstr(qsm(x[0,i] for i in range(1,n)) == 1, "StartNode")  # Start from node 0
    model.addConstr(qsm(x[i,0] for i in range(1,n)) == 1, "EndNode")  # End at node n-1
    model.addConstrs(qsm(x[i,j] for j in range(n) if (i,j) in X)==qsm(x[j,i] for j in range(n) if (j,i) in X) for i in range(1,n))  # Flow conservations
    model.addConstrs(u[i]+1<=u[j]+n*(1-x[i,j]) for i,j in X if i>0 and j>0) # Flow constraints
    # Optimize the model
    model.optimize()
    edges = [(i,j) for i,j in X if x[i,j].x > 0.5]  # Get edges with positive flow
    if model.status == GRB.OPTIMAL:
        print("\nOptimal solution found:")
        # for v in model.getVars():
        #     if v.x > 0.5:  # If the variable is selected
        #         print(f"{v.varName} = {v.x}")
        print("path:", path_make(edges))
        print(f"Optimal Cost: {model.objVal}")
        for k in range(m):
            print(f"Resource {k+1} used: {qsm(x[i,j].x * r[i,j,k+1] for i,j in X)} / {r_max[k]}")
    else:
        print("No optimal solution found.")
    return path_make(edges), model.objVal
# Run the label-setting algorithm
import time 
g = PyGraph(r, r_max)
start_time_bls = time.time()
print("Running BLS algorithm...")

manager = PyLabelManager(g)
print("manager created")

manager.run()
time_bls = time.time()-start_time_bls

solutions = manager.get_solutions()
cost_bls = min(s[1] for s in solutions)
path_bls = [s[0] for s in solutions if s[1] == cost_bls][0]
# print(solutions)
start_time_grb = time.time()
p_grb,cost_grb = solve_with_gurobi()
time_grb = time.time() - start_time_grb

print(f"cost of Gurobi: {cost_grb}, cost of BLS: {cost_bls}")
print(f"gurobipath: {p_grb}, blspath: {path_bls}")
if cost_grb != cost_bls:
    print("Gurobi and BLS solutions differ in cost!")
if time_bls < time_grb:
    print(f"BLS is faster: {time_bls:.4f}s vs {time_grb:.4f}s by factor of {time_grb/time_bls:.2f}")
else:
    print(f"Gurobi is faster: {time_grb:.4f}s vs {time_bls:.4f}s by factor of {time_bls/time_grb:.2f}")
# for idx, (path, cost) in enumerate(solutions):
#     print(f"Solution {idx+1}: Path = {path}, Cost = {cost}")
# print(f"gurobipath: {p_grb}, blspath: {solutions[-1][0]}")

