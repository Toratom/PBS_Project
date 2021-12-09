import pickle
import numpy as np
import open3d as o3d
from ellipsoid_field import EllipsoidField
from sklearn.neighbors import KDTree
from tqdm import tqdm

class Loader(object) :
    '''
    Allows to concatenate several meshes + ellipsoids graphs with their initial position
    ''' 

    #For skinning
    vis_meshes_list = []
    skinning_weights_list = []

    # vis_rotations_list = []
    # vis_center_list = []
    # bodies_offset = []

    nb_of_bodies = 0
    bodies_nb_of_vertexes_list = [] # [nb_of_bodies]

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
    
    def __init__(self, do_skinning = False) :
        '''
        Put do_skinning to true if skinning
        '''
        self.do_skinning = do_skinning

    def add_body(self, path_to_vis_mesh, path_to_graph, init_q, init_t, nb_of_influencing_particles = 4, sig = 1.) :
        '''
        ini_q and ini_t are the solid transformation to initialize the position of the mesh in the world
        init_q is a quaternion describing the rotation np.array([x, y, z, w])
        init_t is a translation np.array([x, y, z])
        '''

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
        #self.bodies_offset.append(self.nb_of_ellipsoids)
        connections = connections + np.array([self.nb_of_ellipsoids, self.nb_of_ellipsoids])
        self.connections_list.extend(connections.tolist())

        bodies = self.nb_of_bodies * np.ones(curr_nb_of_ellipsoids, dtype = int)
        self.bodies_list.extend(bodies.tolist())

        if self.do_skinning :
            #Pre cumputation of skinning weights
            mesh = o3d.io.read_triangle_mesh(path_to_vis_mesh)
            nb_of_vertexes = len(mesh.vertices)
            ellips_position = graph["centers"] #shape [Nb Elli, 3]
            ellips_rotation = graph["rotations"] #shape [Nb Elli, 4]
            print("Nb of Vertices of Loading Mesh : ", nb_of_vertexes)
            print("Nb of Particles of Loading Mesh : ", curr_nb_of_ellipsoids)
            if (curr_nb_of_ellipsoids < nb_of_influencing_particles) :
                print("ERROR : nb_of_influencing_particles must be smaller than ", curr_nb_of_ellipsoids)
                return

            #Each vertex is influenced by the nb_of_influencing_particles nearest particles
            vertexes_weights = [None] * nb_of_vertexes #Each entry None will be replaced by a list of 3 entries 1 : a list of ellips_id, 2 : a list of their associated weights, 3 : a list of the coord of v in their associated local space
            tree = KDTree(ellips_position, leaf_size = 2)
            for v_ind in tqdm(range(nb_of_vertexes)) :
                v = mesh.vertices[v_ind]
                distances, ellips_local_id = tree.query(v.reshape(1, -1), k = nb_of_influencing_particles)
                distances = distances.reshape(-1)
                ellips_local_id = ellips_local_id.reshape(-1)
                #Convert distances to weights
                weights = np.exp(- distances**2 / (2. * sig**2))
                weights /= weights.sum()
                #Convert ellips_local_ids to the global ids i.e. that will be used in the ellipsoid_field
                ellips_global_id = ellips_local_id + self.nb_of_ellipsoids
                #Compute the coordinate of v : v_local_i in the local space of each ellipsoid i
                local_coordinates = [None] * nb_of_influencing_particles
                for ellip_ind in range(nb_of_influencing_particles) :
                    ellip_local_ind = ellips_local_id[ellip_ind]
                    ellip_center = ellips_position[ellip_local_ind]
                    ellip_rot_matrix = self.quaternion_to_matrix(ellips_rotation[ellip_local_ind])

                    v_local = ellip_rot_matrix.T.dot(v - ellip_center)

                    local_coordinates[ellip_ind] = v_local
                
                #Create the 3 entries list and append it to vertexes_weights
                hyper_weights = [ellips_global_id, weights, local_coordinates]
                vertexes_weights[v_ind] = hyper_weights
            
            self.skinning_weights_list.append(vertexes_weights)
            self.vis_meshes_list.append(mesh)
        
        self.bodies_nb_of_vertexes_list.append(nb_of_vertexes)
        self.nb_of_ellipsoids += curr_nb_of_ellipsoids
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
    

    def get_nb_of_bodies(self) :
        return self.nb_of_bodies
    
    def get_body_nb_of_vertex(self, body_ind) :
        '''
        Gives the number of vertexes of the visual mesh associated to the body of index body_ind
        '''
        return self.bodies_nb_of_vertexes_list[body_ind]

    def get_hyper_weights(self, body_ind, vertex_ind) :
        '''
        Return a list of 3 entries, each entry is list of size nb_of_influencing_particles :
        0 - a list of the global id (the one used in the ellipsoids_field) of the ellips wich influenced the position of the vertex_ind
        1 - a list of the corresponding weights
        2 - a list of the vertex position in the local coordinate system of the corresponding particle
        '''
        return self.skinning_weights_list[body_ind][vertex_ind]

    
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