import pickle
import sys

import open3d as o3d
import numpy as np
from scipy.spatial.distance import pdist
from sklearn.neighbors import KDTree
import math
import itertools


class MeshGenerator(object):
    def __init__(self, mesh_path, distance, num_ellips, r_connections, d_connections):
        self.graph = {}
        '''self.graph["centers"] = all_positions
        self.graph["radii"] = all_radii
        self.graph["rotations"] = all_rotations
        self.graph["connections"] = all_connections'''
        self.pcd = None
        self.vis_particles = []
        self.vis_connections = o3d.geometry.LineSet()
        self.distance = distance
        self.num_ellips = num_ellips
        self.r_connections = r_connections
        self.d_connections = d_connections

        # Code adapted from Open3D tutorial
        print('Loading the mesh:')
        self.mesh = o3d.io.read_triangle_mesh(mesh_path)
        # Rotating the mesh. Blender inverts y and z
        #R = self.mesh.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0))
        #self.mesh.rotate(R, center=(0, 0, 0))
        print(self.mesh)
        print('Vertices:')
        print(np.asarray(self.mesh.vertices))
        print('Triangles:')
        print(np.asarray(self.mesh.triangles))
        return

    def visualize_mesh(self):
        print("Try to render a mesh with normals:")
        print("Computing normal and rendering it.")
        self.mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([self.mesh])

    def create_point_cloud(self):
        # self.pcd = self.mesh.sample_points_uniformly(number_of_points=self.num_ellips)
        self.pcd = self.mesh.sample_points_poisson_disk(number_of_points=self.num_ellips)

    def visualize_point_cloud(self):
        if self.pcd is None:
            self.create_point_cloud()
        o3d.visualization.draw_geometries([self.pcd])

    def create_graph(self):
        if self.pcd is None:
            # The inferred point cloud gives the position of the ellipsoids
            self.create_point_cloud()

        # Creation of arrays of vertices and ellipsoid centers
        vertices = self.mesh.vertices
        centers = self.pcd.points

        # Create of support structure for fastly find position of vertexes of the mesh.
        # The KDTree is contained inside a point cloud
        support_structure = o3d.geometry.PointCloud()
        points = np.concatenate((vertices, centers))
        support_structure.points = o3d.utility.Vector3dVector(points)
        points_copy = np.copy(points)  # Create a copy of points. KDTree don't do that
        support_tree = KDTree(points_copy)

        colored = []
        self.vis_particles = []
        self.vis_connections = o3d.geometry.LineSet()

        all_positions = []
        all_rotations = []
        all_quaternions = []
        all_radii = []
        all_connections = []

        for point in centers:

            # Find nearest vertices inside the cloud
            computed_distance = self.find_overlapping(point, self.distance, len(all_positions),
                                                      all_radii, all_positions)
            neighbors = support_tree.query_radius(point.reshape(1, -1), computed_distance)

            # From the indexes, recover the points
            temp = []
            for ind in neighbors[0]:
                # Controlling that the point has not already been taken
                if not colored.__contains__(ind):
                    # Add point to target points.
                    temp.append(points[ind])

            # Creating OBB, when possible
            try:
                temp = o3d.utility.Vector3dVector(np.array(temp))

                obb = o3d.geometry.OrientedBoundingBox.create_from_points(points=temp)

                # We put the radiis back in our coordinate system
                big_r = min(max(obb.extent[0] / 2, 0.2), computed_distance)
                middle_r = min(max(obb.extent[1] / 2, 0.2), computed_distance)
                small_r = min(max(obb.extent[2] / 2, 0.2), computed_distance)

                e_radii = np.array([big_r, small_r, middle_r])

                if np.max(e_radii) > 0.3:  # not too small particle. Here, I am avoiding the inference of small spheres
                    # Creation of ellipsoid
                    e_center = obb.center
                    obb_R = np.zeros((3, 3))
                    obb_R[:, 0] = obb.R[:, 0]
                    obb_R[:, 1] = obb.R[:, 2]
                    obb_R[:, 2] = obb.R[:, 1]

                    if np.linalg.det(obb_R) < 0: # avoid improper rotations
                        e_rotation = - obb_R
                    else:
                        e_rotation = obb_R

                    all_positions.append(e_center)
                    all_quaternions.append(self.matrix_to_quaternion(e_rotation))  # Our solver works on quaternions
                    all_rotations.append(e_rotation)
                    all_radii.append(e_radii)

                    # Coloring points
                    for ind in neighbors[0]:
                        if not colored.__contains__(ind):
                            colored.append(ind)

                    # Coloring the center
                    center_index = np.where(np.all(centers == point, axis=1))
                    colored.append(center_index[0][0])

            except RuntimeError as e:  # faces may be coplanar. Need to retry
                continue

        for i in range(all_positions.__len__()):
            # self.vis_particles.append(self.create_ellipsoid_mesh(all_radii[i], all_positions[i], all_rotations[i]))
            self.vis_particles.append(self.create_ellipsoid_mesh(all_radii[i], all_positions[i],
                                                                 all_rotations[i]))
        # Create connections between particles

        # Another KDTree to simplify queries
        support_structure = o3d.geometry.PointCloud()
        points = np.array(all_positions)
        support_structure.points = o3d.utility.Vector3dVector(points)
        support_tree = KDTree(points)

        for point in points:
            # find the closest neighbors and create connections
            # Combination of two approached: by distance and by neighbors, to optimize results
            dist, neighbors_num = support_tree.query(point.reshape(1, -1), k = (self.d_connections + 1))  # +1 because one is the point itself
            neighbors_num = neighbors_num[0]
            neighbors_dist = support_tree.query_radius(point.reshape(1, -1), self.r_connections)
            neighbors_dist = neighbors_dist[0]
            neighbors = neighbors_dist[np.isin(neighbors_dist, neighbors_num)]
            # neighbor is the point itself
            point_index = np.where(np.all(points == point, axis=1))
            for i in neighbors:
                connection = [point_index[0][0], i]
                if (point_index[0][0] != i) and (not all_connections.__contains__(connection)):
                    all_connections.append(connection)

        self.vis_connections.points = o3d.utility.Vector3dVector(points)
        self.vis_connections.lines = o3d.utility.Vector2iVector(all_connections)

        self.graph["centers"] = np.array(all_positions)
        self.graph["radii"] = np.array(all_radii)
        self.graph["rotations"] = np.array(all_quaternions)
        self.graph["connections"] = np.array(all_connections)
        return

    def find_overlapping(self, pos1, r1, nb_of_ellipsoids, radii, positions, separation=0.001):
        for j in range(nb_of_ellipsoids):

            radii2 = radii[j]
            r2 = max(radii2[0], radii2[1], radii2[2])  # approximating particles with spheres
            pos2 = positions[j]

            distance_vector = pos2 - pos1
            distance = np.linalg.norm(distance_vector)

            if distance < (r1 + r2):  # possible overlapping
                distance = r1 + r2 - distance
                r1 = max(r1 - distance - separation, 0)

        return r1

    '''def compute_radius(self, radii, n, R):
        a = radii[0]
        b = radii[1]
        c = radii[2]
        elli_mat = np.array([[1. / (a * a), 0., 0.], [0., 1. / (b * b), 0.], [0., 0., 1. / (c * c)]])
        A = R @ elli_mat @ R.T
        radius = np.sqrt(1. / (n.T @ A @ n))
        return radius

    def precise_collisions(self, radii1, radii2, n):
        A1 = self.ellips_matrix(radii1)
        A2 = self.ellips_matrix(radii2)
        lmbd = - np.sqrt(n.T @ A1 @ np.linalg.inv(A2) @ n)
        B = np.linalg.inv(lmbd * A2 - A1) @ A2
        d = np.sqrt( 1 / ((lmbd ** 2) * n.T @ B.T @ A1 @ B @ n))
        x = np.linalg.inv(lmbd * A2 - A1) @ A2 @ n * lmbd * d
        return d, x'''

    def ellips_matrix(self, radii):
        return np.array([[1. / (radii[0] * radii[0]), 0., 0.], [0., 1. / (radii[1] * radii[1]), 0.],
                         [0., 0., 1. / (radii[2] * radii[2])]])

    def visualize_graph(self):
        geometries = self.vis_particles.copy()
        connections = self.vis_connections
        geometries.append(connections)
        o3d.visualization.draw_geometries(geometries)

    def create_ellipsoid_mesh(self, radii, translation, rotation):
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=120)

        scale = np.identity(4, dtype=float)
        scale[3, 3] = 1.0
        scale[0, 0] = max(radii[0], 0.1)
        scale[1, 1] = max(radii[1], 0.1)
        scale[2, 2] = max(radii[2], 0.1)
        tr = np.identity(4, dtype=float)
        tr[0, 3] = translation[0]
        tr[1, 3] = translation[1]
        tr[2, 3] = translation[2]

        rot = np.identity(4, dtype=float)
        rot[:3, :3] = rotation

        # T = rot @ scale
        mesh.transform(scale)
        mesh.rotate(rotation)
        # mesh.transform(T)
        mesh.translate(translation)

        mesh.compute_vertex_normals()
        mesh.compute_triangle_normals()

        if np.linalg.det(rotation) < 0:
            mesh.paint_uniform_color([1., 0., 0.])
        return mesh

    def export_particle_graph(self, name):
        if self.graph.keys().__len__() == 0:
            self.create_graph()
        with open(name + '.pkl', 'wb') as out:
            pickle.dump(self.graph, out, pickle.HIGHEST_PROTOCOL)

    def matrix_to_quaternion(self, M):
        tr = M[0, 0] + M[1, 1] + M[2, 2]

        q = np.array([0., 0., 0., 1.])
        if (tr > 0):
            S = np.sqrt(tr + 1.0) * 2.  # S=4*qw
            q[3] = 0.25 * S
            q[0] = (M[2, 1] - M[1, 2]) / S
            q[1] = (M[0, 2] - M[2, 0]) / S
            q[2] = (M[1, 0] - M[0, 1]) / S
        elif ((M[0, 0] > M[1, 1]) & (M[0, 0] > M[2, 2])):
            S = np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2  # S=4*qx
            q[3] = (M[2, 1] - M[1, 2]) / S
            q[0] = 0.25 * S
            q[1] = (M[0, 1] + M[1, 0]) / S
            q[2] = (M[0, 2] + M[2, 0]) / S
        elif (M[1, 1] > M[2, 2]):
            S = np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2  # S=4*qy
            q[3] = (M[0, 2] - M[2, 0]) / S
            q[0] = (M[0, 1] + M[1, 0]) / S
            q[1] = 0.25 * S
            q[2] = (M[1, 2] + M[2, 1]) / S
        else:
            S = np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2  # S=4*qz
            q[3] = (M[1, 0] - M[0, 1]) / S
            q[0] = (M[0, 2] + M[2, 0]) / S
            q[1] = (M[1, 2] + M[2, 1]) / S
            q[2] = 0.25 * S

        return q

    def quaternion_to_matrix(self, q):
        # First row of the rotation matrix
        r00 = 2 * (q[3] * q[3] + q[0] * q[0]) - 1
        r01 = 2 * (q[0] * q[1] - q[3] * q[2])
        r02 = 2 * (q[0] * q[2] + q[3] * q[1])

        # Second row of the rotation matrix
        r10 = 2 * (q[0] * q[1] + q[3] * q[2])
        r11 = 2 * (q[3] * q[3] + q[1] * q[1]) - 1
        r12 = 2 * (q[1] * q[2] - q[3] * q[0])

        # Third row of the rotation matrix
        r20 = 2 * (q[0] * q[2] - q[3] * q[1])
        r21 = 2 * (q[1] * q[2] + q[3] * q[0])
        r22 = 2 * (q[3] * q[3] + q[2] * q[2]) - 1

        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
        return rot_matrix


def main():  # TODO: add arguments to main. Also allow to rotate mesh to import pre-rotated one in simulator

    # main arguments:
    arguments = sys.argv
    if arguments.__len__() < 5:
        raise RuntimeError("Too few arguments")
    candidate_radius = float(arguments[1])
    n_candidate_centers = int(arguments[2])
    candidate_conn_radius = float(arguments[3])
    candidate_conn_number = int(arguments[4])

    mesh_name = input("Insert mesh name (with extension and path):\n")
    print("Loading...")
    generator = MeshGenerator(mesh_name, candidate_radius, n_candidate_centers,
                              candidate_conn_radius, candidate_conn_number)
    generator.create_graph()
    print("Done!")

    while True:
        print("Welcome to Mesh Generator. Your mesh has been converted.")
        command = input("Commands: q - quit, vm - visualize mesh, vg - visualize graph - g name (with path, no ext) - "
                        "export mesh\n")
        if command == "q":
            exit()
        elif command == "vm":
            generator.visualize_mesh()
        elif command == "vg":
            generator.visualize_graph()
        elif command[0] == "g" and command.__len__() >= 3:
            name = command[2:command.__len__()]
            generator.export_particle_graph(name)
        else:
            print("Wrong command")

if __name__ == "__main__":
    # Parameters of main:
    # candidate_radius = maximum radius of the inferred particles
    # n_candidate_centers = maximum number of inferred particles
    # candidate_conn_radius = radius inside which we look for neighbors for connections
    # candidate_conn_number = maximum number of neighbors of each particle
    main()
