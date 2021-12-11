import open3d as o3d
import numpy as np
import random

mesh = o3d.io.read_triangle_mesh("Meshes/duck_smooth_green.obj", True)
frame = o3d.io.read_triangle_mesh("Meshes/duck_smooth_blue.obj", True)

T = np.zeros((4, 4), dtype = float)
T[3, 3] = 1.
T[0, 0] = 0.5
T[1, 1] = 1.
T[2, 2] = 0.5
mesh.transform(T)
vertexes = np.asarray(mesh.vertices)
print(vertexes)
vertexes += np.array([0., 5., 0.])
mesh.vertices = o3d.utility.Vector3dVector(vertexes)

mesh.compute_vertex_normals()
mesh.compute_triangle_normals()

# axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
frame += mesh
# o3d.visualization.draw_geometries([axis, frame])

o3d.io.write_triangle_mesh("test.obj", frame)


# frame = o3d.geometry.TriangleMesh()
# radii = [1., 0.5, 1.]
# visual_mesh = o3d.geometry.TriangleMesh.create_sphere(radius = 1.0, resolution = 120)
# frame += visual_mesh
# T = np.zeros((4, 4), dtype = float)
# T[3, 3] = 1.0
# T[0, 0] = radii[0]
# T[1, 1] = radii[1]
# T[2, 2] = radii[2]
# visual_mesh.transform(T)
# visual_mesh.translate((1., 5., 2.))
# frame += visual_mesh

# # visual_mesh.compute_vertex_normals()
# # visual_mesh.compute_triangle_normals()
# # o3d.visualization.draw_geometries([visual_mesh, axis])

# # vertexes = visual_mesh.vertices
# # vertexes += np.array([0., 5., 0.])
# # visual_mesh.vertices = o3d.utility.Vector3dVector(vertexes)

# visual_mesh.compute_vertex_normals()
# visual_mesh.compute_triangle_normals()
# axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
# # o3d.visualization.draw_geometries([frame, axis])

# o3d.io.write_triangle_mesh("test.ply", frame)

# a = np.array([])
# try :
#     temp = o3d.utility.Vector3dVector(np.array([]))
# except:
#     print("error")


# def quaternion_to_matrix(q):
#     # First row of the rotation matrix
#     r00 = 2 * (q[3] * q[3] + q[0] * q[0]) - 1
#     r01 = 2 * (q[0] * q[1] - q[3] * q[2])
#     r02 = 2 * (q[0] * q[2] + q[3] * q[1])

#     # Second row of the rotation matrix
#     r10 = 2 * (q[0] * q[1] + q[3] * q[2])
#     r11 = 2 * (q[3] * q[3] + q[1] * q[1]) - 1
#     r12 = 2 * (q[1] * q[2] - q[3] * q[0])

#     # Third row of the rotation matrix
#     r20 = 2 * (q[0] * q[2] - q[3] * q[1])
#     r21 = 2 * (q[1] * q[2] + q[3] * q[0])
#     r22 = 2 * (q[3] * q[3] + q[2] * q[2]) - 1

#     # 3x3 rotation matrix
#     rot_matrix = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
#     return rot_matrix

# def matrix_to_quaternion2(M) :
#     tr = M[0, 0] + M[1, 1] + M[2, 2]

#     if (tr > 0) : 
#         S = np.sqrt(tr+1.0) * 2 # S=4*qw 
#         qw = 0.25 * S
#         qx = (M[2, 1] - M[1, 2]) / S
#         qy = (M[0, 2] - M[2, 0]) / S
#         qz = (M[1, 0] - M[0, 1]) / S 
#     elif ((M[0, 0] > M[1, 1])&(M[0, 0] > M[2, 2])) :
#         S = np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2 # S=4*qx 
#         qw = (M[2, 1] - M[1, 2]) / S
#         qx = 0.25 * S
#         qy = (M[0, 1] + M[1, 0]) / S
#         qz = (M[0, 2] + M[2, 0]) / S 
#     elif (M[1, 1] > M[2, 2]) :
#         S = np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2 # S=4*qy
#         qw = (M[0, 2] - M[2, 0]) / S
#         qx = (M[0, 1] + M[1, 0]) / S
#         qy = 0.25 * S
#         qz = (M[1, 2] + M[2, 1]) / S
#     else :
#         S = np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2 # S=4*qz
#         qw = (M[1, 0] - M[0, 1]) / S
#         qx = (M[0, 2] + M[2, 0]) / S
#         qy = (M[1, 2] + M[2, 1]) / S
#         qz = 0.25 * S

#     q = np.array([qx, qy, qz, qw])
#     return q


# coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

# radii = [1., 0.05, 0.5]
# visual_mesh = o3d.geometry.TriangleMesh.create_sphere(radius = 1.0, resolution = 120)
# elli_axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
# T = np.zeros((4, 4), dtype = float)
# T[3, 3] = 1.0
# T[0, 0] = radii[0]
# T[1, 1] = radii[1]
# T[2, 2] = radii[2]
# visual_mesh.transform(T)
# #adding rotation
# theta = np.radians(90.)
# u = np.array([0., 0., -1.])
# q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
# R = quaternion_to_matrix(q)
# T = np.zeros((4, 4), dtype = float)
# T[:3, :3] = R
# T[3, 3] = 1.0
# visual_mesh.transform(T)
# elli_axis.transform(T)

# visual_mesh.compute_vertex_normals()
# visual_mesh.compute_triangle_normals()

# #print(np.asarray(visual_mesh.vertices))

# obb = o3d.geometry.OrientedBoundingBox.create_from_points(points=visual_mesh.vertices)
# big_r = obb.extent[0] / 2.
# middle_r = obb.extent[1] / 2.
# small_r = obb.extent[2] / 2.
# e_radii = np.array([big_r, small_r, middle_r]) #change order to follow convention normal on y

# obb_R = np.zeros((3, 3))
# obb_R[:, 0] = obb.R[:, 0]
# obb_R[:, 1] = obb.R[:, 2]
# obb_R[:, 2] = obb.R[:, 1]

# obb_q = matrix_to_quaternion2(obb_R)

# # R_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
# # T = np.zeros((4, 4), dtype = float)
# # T[:3, :3] = R
# # T[3, 3] = 1.0
# # R_vis.transform(T)
# # obb_R_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
# # T = np.zeros((4, 4), dtype = float)
# # T[:3, :3] = obb_R
# # T[3, 3] = 1.0
# # obb_R_vis.transform(T)
# # #o3d.visualization.draw_geometries([visual_mesh, elli_axis])
# # o3d.visualization.draw_geometries([R_vis])

# visual_mesh_rebuilt = o3d.geometry.TriangleMesh.create_sphere(radius = 1.0, resolution = 120)
# obb_R_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
# T = np.zeros((4, 4), dtype = float)
# T[3, 3] = 1.0
# T[0, 0] = e_radii[0]
# T[1, 1] = e_radii[1]
# T[2, 2] = e_radii[2]
# visual_mesh_rebuilt.transform(T)
# #adding rotation
# # theta = np.radians(90.)
# # u = np.array([0., 0., -1.])
# # q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
# # R = quaternion_to_matrix(q)
# # T = np.zeros((4, 4), dtype = float)
# T[:3, :3] = quaternion_to_matrix(obb_q) #obb_R
# T[3, 3] = 1.0
# visual_mesh_rebuilt.transform(T)
# obb_R_vis.transform(T)
# visual_mesh_rebuilt.compute_vertex_normals()
# visual_mesh_rebuilt.compute_triangle_normals()
# o3d.visualization.draw_geometries([visual_mesh_rebuilt, obb_R_vis])



# def quaternion_multiply(p, q):
#     ret = np.array(
#         [
#             p[0] * q[3] + p[3] * q[0] + p[1] * q[2] - p[2] * q[1],
#             p[1] * q[3] + p[3] * q[1] + p[2] * q[0] - p[0] * q[2],
#             p[2] * q[3] + p[3] * q[2] + p[0] * q[1] - p[1] * q[0],
#             p[3] * q[3] - p[0] * q[0] - p[1] * q[1] - p[2] * q[2],
#         ]
#     )
#     return ret

# def quaternion_inverse(q):
#     qInv = np.array([-q[0], -q[1], -q[2], q[3]]) / np.linalg.norm(q)
#     return qInv

# def matrix_to_quaternion(M):
#     qw = 0.5 * np.sqrt(1 + M[0, 0] + M[1, 1] + M[2, 2])
#     qx = (M[2, 1] - M[1, 2]) / (4 * qw)
#     qy = (M[0, 2] - M[2, 0]) / (4 * qw)
#     qz = (M[1, 0] - M[0, 1]) / (4 * qw)

#     q = np.array([qx, qy, qz, qw])
#     return q

# def quaternion_to_matrix2(q) :
#     sqw = q[3] * q[3]
#     sqx = q[0] * q[0]
#     sqy = q[1] * q[1]
#     sqz = q[2] * q[2]

#     # invs (inverse square length) is only required if quaternion is not already normalised
#     invs = 1 / (sqx + sqy + sqz + sqw)
#     m00 = ( sqx - sqy - sqz + sqw) * invs # since sqw + sqx + sqy + sqz =1/invs*invs
#     m11 = (-sqx + sqy - sqz + sqw) * invs
#     m22 = (-sqx - sqy + sqz + sqw) * invs
    
#     tmp1 = q[0] * q[1]
#     tmp2 = q[2] * q[3]
#     m10 = 2.0 * (tmp1 + tmp2) * invs
#     m01 = 2.0 * (tmp1 - tmp2) * invs
    
#     tmp1 = q[0] * q[2]
#     tmp2 = q[1] * q[3]
#     m20 = 2.0 * (tmp1 - tmp2) * invs
#     m02 = 2.0 * (tmp1 + tmp2) * invs
#     tmp1 = q[1] * q[2]
#     tmp2 = q[0] * q[3]
#     m21 = 2.0 * (tmp1 + tmp2) * invs
#     m12 = 2.0 * (tmp1 - tmp2) * invs

#     return np.array([[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]])


# def matrix_to_quaternion2(M) :
#     tr = M[0, 0] + M[1, 1] + M[2, 2]

#     if (tr > 0) : 
#         S = np.sqrt(tr+1.0) * 2 # S=4*qw 
#         qw = 0.25 * S
#         qx = (M[2, 1] - M[1, 2]) / S
#         qy = (M[0, 2] - M[2, 0]) / S
#         qz = (M[1, 0] - M[0, 1]) / S 
#     elif ((M[0, 0] > M[1, 1])&(M[0, 0] > M[2, 2])) :
#         S = np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2 # S=4*qx 
#         qw = (M[2, 1] - M[1, 2]) / S
#         qx = 0.25 * S
#         qy = (M[0, 1] + M[1, 0]) / S
#         qz = (M[0, 2] + M[2, 0]) / S 
#     elif (M[1, 1] > M[2, 2]) :
#         S = np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2 # S=4*qy
#         qw = (M[0, 2] - M[2, 0]) / S
#         qx = (M[0, 1] + M[1, 0]) / S
#         qy = 0.25 * S
#         qz = (M[1, 2] + M[2, 1]) / S
#     else :
#         S = np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2 # S=4*qz
#         qw = (M[1, 0] - M[0, 1]) / S
#         qx = (M[0, 2] + M[2, 0]) / S
#         qy = (M[1, 2] + M[2, 1]) / S
#         qz = 0.25 * S

#     q = np.array([qx, qy, qz, qw])
#     return q

# theta = np.radians(90.)
# u = np.array([0., 0., -1])
# # u = u / np.linalg.norm(u)
# q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
# R = quaternion_to_matrix(q)

# print(R)
# print("")
# print("")

# elli = np.array([[0.5, 0., 0.], [0., 0.05, 0.], [0., 0., 0.5]])

# print(R@elli@R.T)





# print(quaternion_to_matrix2(q))
# print(quaternion_to_matrix2(-q))

# theta = 2 * np.arccos(q[3])
# s = np.sqrt(1 - q[3]**2)
# u = np.array([q[0] / s, q[1] / s, q[2] / s])
# if theta > np.pi :
#     theta = 2 * np.pi - theta
#     u = -u
# print(theta)
# print(u)
# theta2 = 2 * np.arccos(-q[3])
# u2 = np.array([-q[0] / s, -q[1] / s, -q[2] / s])
# print(theta, " VS ", theta2)
# print(u, " VS ", u2)

# R = quaternion_to_matrix(q)
# print("Det ", np.linalg.det(R))
# q_back = matrix_to_quaternion(R)
# R_back = quaternion_to_matrix(q_back)
# print("Det back ", np.linalg.det(R_back))
# print(R, " VS ", R_back)
# print(q, " VS ", q_back)


# theta_back = 2 * np.arccos(q_back[3])
# s = np.sqrt(1 - q_back[3]**2)
# u_back = np.array([q_back[0] / s, q_back[1] / s, q_back[2] / s])


# id = quaternion_multiply(q, quaternion_inverse(q_back))

# print(u, " VS ", u_back)
# print(theta, " VS ", theta_back)
# print(2*np.pi - theta)
# print(id)




# s = np.sqrt(q_back[0]**2 + q_back[1]**2 + q_back[2]**2)
# theta_back = 2 * np.arctan2(s, q_back[3])
# u_back = np.array([q_back[0] / s, q_back[1] / s, q_back[2] / s])


#q = [x, y, z, w] = w + ix + jy + kz
# theta = np.radians(189.)
# u = np.array([0.5, 1., 0.5])
# u = u / np.linalg.norm(u)
# q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])

# theta_back = 2 * np.arccos(q[3])
# s = np.sqrt(1 - q[3]**2)
# u_back = np.array([q[0] / s, q[1] / s, q[2] / s])

# print(u, " VS ", u_back)
# print(theta, " VS ", theta_back)


# nb_of_particules = 5
# # edges = [[random.randint(0, nb_of_particules-1), random.randint(1, nb_of_particules-1)] for _ in range(10)]
# edges = [[3, 0], [0,2], [0,1], [2,1]]
# adjacency_list = [[] for _ in range(nb_of_particules)]

# for e in edges :
#     a, b = e
#     adjacency_list[a].append(b)
#     adjacency_list[b].append(a)

# max_p_neighbors = 0
# for i in range(nb_of_particules) :
#     p_neighbors = adjacency_list[i]
#     if len(p_neighbors) > max_p_neighbors :
#         max_p_neighbors = len(p_neighbors)
#     adjacency_list[i] = [len((p_neighbors))] + p_neighbors

# adjacency_list_np = np[2]eros((nb_of_particules, max_p_neighbors + 1), dtype = int) #+1 as added the len at begining
# for i in range(nb_of_particules) :
#     p_neighbors = adjacency_list[i]
#     for j in range(len(p_neighbors)) :
#         adjacency_list_np[i, j] = p_neighbors[j]

# print(adjacency_list_np)
# adj = adjacency_list_np[4]
# for i in range(1, adj[0] + 1) :
#     print(adj[i])


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