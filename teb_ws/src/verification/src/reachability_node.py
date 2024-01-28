
import numpy as np
from StarV.set.probstar import ProbStar
import math

robot_width = 0.281
robot_length = 0.306
std_dev=2 
steps=3

    
def compute_reachability( bb_vertices_human,  X_initial, std_initial,  A ):  
    


    # Convert Initial State to Probstar 
    c = (np.expand_dims(X_initial, axis=0)).transpose()
    V = np.diag(std_initial)
    n_sigma = np.diag(np.ones(X_initial.shape[0]))
    n_mu = np.zeros(X_initial.shape[0])
    l = np.ones(X_initial.shape[0]) * std_dev * -1
    u = np.ones(X_initial.shape[0]) * std_dev
    
    probstars = []

    # Iterate Model Until k = reachability_start_idx
    for i in range(steps):
        for j in range(10):
            c = np.matmul(A, c)
            V = np.matmul(A, V)

    
    for i in range(steps):
        for j in range(10):
            c = np.matmul(A,c)
            V = np.matmul(A, V)

        # Apply Collision Bounding Predicate
        H,g = predicates(i,bb_vertices_human, robot_width, robot_length)
        C = np.matmul(H,V)
        d = g-np.matmul(H,c)
        d = np.asarray(d).squeeze()
        c_V_Combine =  np.concatenate([c,V], axis=1)
        c_V_Combine = np.asarray(c_V_Combine)
        V = np.asarray(V)
        C = np.asarray(C)
        probstar = ProbStar(c_V_Combine,C,d,n_mu,n_sigma)
        probstars.append(probstar)
    return probstars





def bb_vertices(width, length, angle):
   
    x1 = (length/2) * math.cos(angle) - (width/2) * math.sin(angle)
    y1 = (width/2) * math.cos(angle) + (length/2) * math.sin(angle)
    x2 = -(length/2) * math.cos(angle) - (width/2) * math.sin(angle)
    y2 = (width/2) * math.cos(angle) - (length/2) * math.sin(angle)
    x3 = (length/2) * math.cos(angle) + (width/2) * math.sin(angle)
    y3 = -(width/2) * math.cos(angle) + (length/2) * math.sin(angle)
    x4 = -(length/2) * math.cos(angle) + (width/2) * math.sin(angle)
    y4 = -(width/2) * math.cos(angle) - (length/2) * math.sin(angle)
    bounding_box_vertices = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
    return bounding_box_vertices

def add_edges(edges, bb_vertices, box_index):
        for i in range(len(bb_vertices)):
            next_idx = (i + 1) % len(bb_vertices)
            dx = bb_vertices[next_idx][0] - bb_vertices[i][0]
            dy = bb_vertices[next_idx][1] - bb_vertices[i][1]
            edges.append([dx, dy, box_index, i])

def minkowski_difference_2d_convex_hull(bb1, bb2):
    
    # Convert to minkowski addition problem by negating vertices of the second bounding box
    bb2_negated = [[-x, -y] for x, y in bb2]

    # Sort vertices by polar angle
    bb1.sort(key=lambda vertex: math.atan2(vertex[1], vertex[0]))
    bb2_negated.sort(key=lambda vertex: math.atan2(vertex[1], vertex[0]))

    # Create edges for both bounding boxes
    edges = []        

    add_edges(edges, bb1, 1)
    add_edges(edges, bb2_negated, 2)

    # Sort edges by polar angle
    edges.sort(key=lambda edge: math.atan2(edge[1], edge[0]))

    # Calculate the starting point as the sum of positions of starting vertices in the first two edges
    first_edge_bb1 = next(edge for edge in edges if edge[2] == 1)
    first_edge_bb2 = next(edge for edge in edges if edge[2] == 2)
    starting_x = bb1[first_edge_bb1[3]][0] + bb2[first_edge_bb2[3]][0]
    starting_y = bb1[first_edge_bb1[3]][1] + bb2[first_edge_bb2[3]][1]

    minkowski_sum_bbox = []
    current_point = (starting_x, starting_y)

    # Calculate Minkowski sum
    for edge in edges:
        current_point = (current_point[0] + edge[0], current_point[1] + edge[1])
        minkowski_sum_bbox.append(current_point)

    return minkowski_sum_bbox

def convex_hull_vertex_array_to_linear_constraint(convex_hull_array):
    num_vertices = len(convex_hull_array)
    C = np.zeros((num_vertices, 2))
    d = np.zeros((num_vertices, 1))

    for idx in range(num_vertices):
        x1, y1 = convex_hull_array[idx]
        x2, y2 = convex_hull_array[(idx + 1) % num_vertices]

        C[idx, 0] = -(y2 - y1)
        C[idx, 1] = x2 - x1
        d[idx, 0] = -(y1 * (x2 - x1) - x1 * (y2 - y1))

    return C, d

def predicates(angle, bb_vertices_human, robot_width, robot_length,):  
    bb_vertices_robot = bb_vertices(robot_width, robot_length,  angle)    
    minkowski_difference = minkowski_difference_2d_convex_hull(bb_vertices_robot,bb_vertices_human)
    C_s,d_s = convex_hull_vertex_array_to_linear_constraint(minkowski_difference)
    C = np.hstack([C_s, np.zeros((C_s.shape[0],2)),-1*C_s,np.zeros((C_s.shape[0],2))])
    d=d_s   
    return C,d