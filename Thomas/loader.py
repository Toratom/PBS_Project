import pickle
import numpy as np
import open3d as o3d
from ellipsoid_field import EllipsoidField

class Loader(object) :
    '''
    Allows to concatenate several meshes + ellipsoids graphs with their initial position
    ''' 

    #For skinning
    vis_mesh_list = []
    vis_rotations_list = []
    vis_center_list = []
    bodies_offset = []

    nb_of_bodies = 0

    #For generating ellipsfield
    radii_list = [] #list of list of size 3
    centers_list = [] #list of list of size 3
    rotations_list = [] #list of list of  size 4
    connections_list = [] #list of list of  size 2
    bodies_list = [] #list of int
    velocities_list = [] #list of list of  size 3
    angular_velocities_list = [] #list of list of  size 3
    masses_list = [] #list of float
    nb_of_ellipsoids = 0
    
    def __init__(self, keep_vis_mesh = False) :
        '''
        Put keep_vis_mesh to true if skinning
        '''
        self.keep_vis_mesh = keep_vis_mesh

    def add_body(self, path_to_vis_mesh, path_to_graph, init_q, init_t) :
        if self.keep_vis_mesh :
            self.vis_mesh_list.append(o3d.io.read_triangle_mesh(path_to_vis_mesh))

        with open(path_to_graph, 'rb') as inp:
            graph = pickle.load(inp)

        curr_nb_of_ellipsoids = graph["centers"].shape[0]

        #nb_of_pairs = graph["connections"].shape[0]
        radii_array = graph["radii"]
        self.radii_list.extend(radii_array.tolist())

        ini_centers = graph["centers"]
        #Takes into account init_t (and init_q)
        R = self.quaternion_to_matrix(init_q)
        for i in range(curr_nb_of_ellipsoids) :
            ini_centers[i] = np.dot(R, ini_centers[i]) + init_t
        self.centers_list.extend(ini_centers.tolist())

        ini_rotation = graph["rotations"]
        #Takes into account init_q
        for i in range(curr_nb_of_ellipsoids) :
            ini_rotation[i] = self.quaternion_multiply(init_q, ini_rotation[i])
        self.rotations_list.extend(ini_rotation.tolist())

        connections = graph["connections"]
        #Needs to take into account the new index
        self.bodies_offset.append(self.nb_of_ellipsoids)
        connections = connections + np.array([self.nb_of_ellipsoids, self.nb_of_ellipsoids])
        self.connections_list.extend(connections.tolist())

        bodies = self.nb_of_bodies * np.ones(curr_nb_of_ellipsoids, dtype = int)
        self.bodies_list.extend(bodies.tolist())

        self.nb_of_ellipsoids += graph["centers"].shape[0]
        self.nb_of_bodies += 1


    def generate_ellipsoids_field(self) :
        #List to array
        radii_array = np.array(self.radii_list)
        ini_centers = np.array(self.centers_list)
        ini_rotation = np.array(self.rotations_list)
        connections = np.array(self.connections_list)
        bodies = np.array(self.bodies_list)
        ini_velocities = np.zeros((self.nb_of_ellipsoids, 3))
        ini_angular_velocities = np.zeros((self.nb_of_ellipsoids, 3))
        ini_mass = 10. * np.ones(self.nb_of_ellipsoids, dtype = float)
        gravity = np.array([0., -9.8, 0.])
        res = 5
        shape = (self.nb_of_ellipsoids,)
        return EllipsoidField(radii_array, ini_centers, ini_rotation, connections, bodies, ini_velocities, ini_angular_velocities, ini_mass, gravity, res, shape)


    def get_nb_of_ellipsoids(self) :
        return self.nb_of_ellipsoids
    

    def get_nb_of_edges(self) :
        return len(self.connections_list)
    
    #----utils-----
    def quaternion_multiply(self, p, q):
        ret = np.array(
            [
                p[0] * q[3] + p[3] * q[0] + p[1] * q[2] - p[2] * q[1],
                p[1] * q[3] + p[3] * q[1] + p[2] * q[0] - p[0] * q[2],
                p[2] * q[3] + p[3] * q[2] + p[0] * q[1] - p[1] * q[0],
                p[3] * q[3] - p[0] * q[0] - p[1] * q[1] - p[2] * q[2],
            ]
        )
        return ret
    
    def quaternion_to_matrix(self, q) :
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