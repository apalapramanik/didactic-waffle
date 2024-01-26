import matplotlib.pyplot as plt
import math

def add_edges(edges, bounding_box_vertices, box_index):
            for i in range(len(bounding_box_vertices)):
                next_idx = (i + 1) % len(bounding_box_vertices)
                dx = bounding_box_vertices[next_idx][0] - bounding_box_vertices[i][0]
                dy = bounding_box_vertices[next_idx][1] - bounding_box_vertices[i][1]
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

# Example convex hulls
bb1 = [(3, 1), (1, 1), (3, 3), (1, 3)]
bb2 = [(5, 5), (7, 5), (7, 7), (5, 7)]

# Calculate Minkowski Difference
resulting_points = minkowski_difference_2d_convex_hull(bb1, bb2)

# Plotting
bb1.append(bb1[0])  # Closing the loop
bb2.append(bb2[0])  # Closing the loop
resulting_points.append(resulting_points[0])  # Closing the loop

plt.plot(*zip(*bb1), label='bb1', color='blue')
plt.plot(*zip(*bb2), label='bb2', color='orange')
plt.plot(*zip(*resulting_points), label='Minkowski Sum', color='green')

plt.scatter(*zip(*bb1), color='blue')
plt.scatter(*zip(*bb2), color='orange')
plt.scatter(*zip(*resulting_points), color='green')

plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Minkowski Sum of Convex Hulls')
plt.legend()
plt.grid(True)
plt.show()


