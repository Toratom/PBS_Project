import open3d as o3d
import numpy as np
import random

nb_of_particules = 5
# edges = [[random.randint(0, nb_of_particules-1), random.randint(1, nb_of_particules-1)] for _ in range(10)]
edges = [[3, 0], [0,2], [0,1], [2,1]]
adjacency_list = [[] for _ in range(nb_of_particules)]

for e in edges :
    a, b = e
    adjacency_list[a].append(b)
    adjacency_list[b].append(a)

max_p_neighbors = 0
for i in range(nb_of_particules) :
    p_neighbors = adjacency_list[i]
    if len(p_neighbors) > max_p_neighbors :
        max_p_neighbors = len(p_neighbors)
    adjacency_list[i] = [len((p_neighbors))] + p_neighbors

adjacency_list_np = np.zeros((nb_of_particules, max_p_neighbors + 1), dtype = int) #+1 as added the len at begining
for i in range(nb_of_particules) :
    p_neighbors = adjacency_list[i]
    for j in range(len(p_neighbors)) :
        adjacency_list_np[i, j] = p_neighbors[j]

print(adjacency_list_np)
adj = adjacency_list_np[4]
for i in range(1, adj[0] + 1) :
    print(adj[i])


# bodies = [0, 0, 2, 2, 2, 2, 2, 3, 4, 5, 8,8,8]

# bodies_indexes = []
# body_idx = 0
# begin_index = 0
# ending_pos = 1
# i = 0
# while (i < len(bodies)) :
#     cur_body = bodies[i]
#     while (i + 1 < len(bodies)) and (cur_body == bodies[i + 1]) :
#         i += 1
#         ending_pos += 1
#     #Body found
#     bodies_indexes.append([begin_index, ending_pos])
#     #Next body
#     body_idx += 1
#     begin_index = ending_pos
#     ending_pos += 1
#     i += 1

# bodies_indexes = np.array(bodies_indexes)

# print(bodies_indexes)

# for i in range(body_idx) :
#     beg, end = bodies_indexes[i]
#     print(bodies[beg : end])


# print("Let\'s draw a cubic using o3d.geometry.LineSet")
# points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1],
#             [0, 1, 1], [1, 1, 1]])
# lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
#             [0, 4], [1, 5], [2, 6], [3, 7]]
# colors = [[1, 0, 0] for i in range(len(lines))]
# line_set = o3d.geometry.LineSet()
# line_set.points = o3d.utility.Vector3dVector(points)
# line_set.lines = o3d.utility.Vector2iVector(lines)
# line_set.colors = o3d.utility.Vector3dVector(colors)

# print("TYPE : ", type(line_set))

# vis = o3d.visualization.VisualizerWithKeyCallback()
# vis.create_window()

# vis.add_geometry(line_set)

# t = np.array([1., 1., 1.]) / 10000.
# while True :
#     points = points + t
#     line_set.points = o3d.utility.Vector3dVector(points)
#     vis.update_geometry(line_set)
#     if not vis.poll_events():
#         break
#     vis.update_renderer()