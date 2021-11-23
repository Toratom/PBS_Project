import taichi as ti
import open3d as o3d
import numpy as np
import math
import itertools
import utils


class ObjType:
    STATIC = 0
    DYNAMIC = 1


@ti.data_oriented
class EllipsoidField(object):
    #TO DO :
    #   ## Add Intertia tensor cf page 4 oriented particules

    def __init__(self,
     radii_array,
     center_array,
     rot_array,
     connections,
     bodies, #List of bodies index, gives for each particules the body it belongs to, should be incresing [0,0,1,1] OK [0,1,0,1] NOPE [0,0,2,2] OK
     velocity_array,
     angular_velocity_array,
     mass_array,
     gravity,
     res = 120, shape=()):
        self.shape = shape #Shape de la "k-grille" d'ellipsoid, si k = 5 signifie une liste de 5 ellipsoids
        self.nb_of_particules = np.prod(self.shape)
        self.meshes = np.ndarray(shape = self.shape, dtype=o3d.geometry.TriangleMesh) #Array of references to the ellipsoids meshes
        self.lines = o3d.geometry.LineSet() #Ref to the lineSet
        #Do we really needs to stored them ?
        self.radii_array = radii_array
        self.center_array = center_array
        self.rot_array = rot_array
        self.connections_np = connections
        self.nb_of_edges = self.connections_np.shape[0]
        self.velocity_array = velocity_array
        self.mass_array = mass_array
        self.gravity = ti.Vector([gravity[0], gravity[1], gravity[2]])

        if (len(shape) != 1) :
            print("ERROR Shape must be 1D") #Because bodies_indexes assumes 1D
            return

        # create ranges to iterate over
        self.shape_ranges = []
        for dim in shape:
            self.shape_ranges.append(list(range(dim)))

        #create the lines
        self.lines.points = o3d.utility.Vector3dVector(self.center_array)
        self.lines.lines = o3d.utility.Vector2iVector(self.connections_np)

        # create the meshes
        n_vertices = np.ndarray(shape = self.shape, dtype=int)
        n_faces = np.ndarray(shape = self.shape, dtype=int)
        self.num_meshes = 0
        for e in itertools.product(*self.shape_ranges):
            radii = self.radii_array[e]
            self.meshes[e] = o3d.geometry.TriangleMesh.create_sphere(radius = 1.0, resolution = res)
            # elli.paint_uniform_color([1.0, 0.4, 0.2])
            #Non uni scaling to sphere to ellipsoid
            T = np.zeros((4, 4), dtype = float)
            T[3, 3] = 1.0
            T[0, 0] = radii[0]
            T[1, 1] = radii[1]
            T[2, 2] = radii[2]
            self.meshes[e].transform(T)
            self.meshes[e].compute_vertex_normals()
            self.meshes[e].compute_triangle_normals()
            n_vertices[e] = len(self.meshes[e].vertices)
            n_faces[e] = len(self.meshes[e].triangles)
            self.num_meshes += 1

        self.max_num_vertices = n_vertices.max()
        self.max_num_faces = n_faces.max()
        self.sum_num_vertices = n_vertices.sum()
        self.sum_num_faces = n_faces.sum()

        #Fields used for rendering not for simulating
        self.nV = ti.field(dtype=ti.i32, shape=self.shape) #Field of number of vertices of each meshes
        self.nF = ti.field(dtype=ti.i32, shape=self.shape)
        self.V = ti.Vector.field(3, ti.f32, (*self.shape, self.max_num_vertices)) #Field of intital vertices array of each ellipsoid
        self.new_V = ti.Vector.field(3, ti.f32, (*self.shape, self.max_num_vertices)) #Field of current vertices array of each ellipsoid, after model transfo
        self.F = ti.Vector.field(3, ti.i32, (*self.shape, self.max_num_faces))
        self.C = ti.Vector.field(3, ti.f32, (*self.shape, self.max_num_vertices))

        vertices = np.zeros(shape=(*self.shape, self.max_num_vertices, 3), dtype=float)
        faces = np.zeros(shape=(*self.shape, self.max_num_faces, 3), dtype=int)
        colors = np.tile(
            np.array([0.1, 0.3, 1.0]), (*self.shape, self.max_num_vertices, 1)
        )
        for e in itertools.product(*self.shape_ranges):
            vertices[e][: n_vertices[e]] = np.asarray(self.meshes[e].vertices)
            faces[e][: n_faces[e]] = np.asarray(self.meshes[e].triangles)

        #Init of the previous field from numpy arrays
        self.nV.from_numpy(n_vertices)
        self.nF.from_numpy(n_faces)
        self.V.from_numpy(vertices)
        self.F.from_numpy(faces)
        self.C.from_numpy(colors)

        #init connections
        self.connections = ti.Vector.field(2, dtype = ti.i32, shape = self.connections_np.shape[0])
        self.connections.from_numpy(self.connections_np)

        #init adjacency list, create a list then a field
        adjacency_list = [[] for _ in range(self.nb_of_particules)]
        for e in self.connections_np :
            a, b = e
            adjacency_list[a].append(b)
            adjacency_list[b].append(a)

        max_p_neighbors = 0
        for i in range(self.nb_of_particules) :
            p_neighbors = adjacency_list[i]
            if len(p_neighbors) > max_p_neighbors :
                max_p_neighbors = len(p_neighbors)
            adjacency_list[i] = [len((p_neighbors))] + p_neighbors

        adjacency_list_np = np.zeros((self.nb_of_particules, max_p_neighbors + 1), dtype = int) #+1 as add the len at begining
        for i in range(self.nb_of_particules) :
            p_neighbors = adjacency_list[i]
            for j in range(len(p_neighbors)) :
                adjacency_list_np[i, j] = p_neighbors[j]
        #Init the scalar field associated
        self.adjacency = ti.field(dtype=ti.i32, shape = (self.nb_of_particules, max_p_neighbors + 1))
        self.adjacency.from_numpy(adjacency_list_np)
        
        #init bodies_indexes
        #Create the numpy array corresponding to the field
        bodies_indexes_list = []
        body_idx = 0
        begin_index = 0
        ending_pos = 1
        i = 0
        while (i < len(bodies)) :
            cur_body = bodies[i]
            while (i + 1 < len(bodies)) and (cur_body == bodies[i + 1]) :
                i += 1
                ending_pos += 1
            #Body found
            bodies_indexes_list.append([begin_index, ending_pos])
            #Next body
            body_idx += 1
            begin_index = ending_pos
            ending_pos += 1
            i += 1
        bodies_indexes_np = np.array(bodies_indexes_list)
        #At the end body_idx gives the number of bodies
        self.nb_of_bodies = body_idx
        #Create the field bodies_index : it is a field of 2D int vector, of len nb_of_bodies.
        #Each vectors gives for its corresponding body, index_of_first_particule, index_of_last_particule + 1
        #Such that to scan a body k : for idx in range(bodies_indexes[k][0], bodies_indexes[k][1]) : ...
        self.bodies_indexes = ti.Vector.field(2, dtype = ti.i32, shape = body_idx)
        self.bodies_indexes.from_numpy(bodies_indexes_np)

        #init base object properties : x, p, velocities, quat, rot, mass, radii
        self.init_x = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)
        self.init_quat = ti.Vector.field(4, dtype=ti.f32, shape=self.shape)
        self.init_rot = ti.Matrix.field(3, 3, dtype=ti.f32, shape=self.shape)
        self.init_v = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)
        self.init_w = ti.Vector.field(3, dtype = ti.f32, shape = self.shape)
        self.radii = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)
        self.mass = ti.field(dtype=ti.f32, shape=self.shape)
        self.massInv = ti.field(dtype=ti.f32, shape=self.shape)
        self.ext_force = ti.Vector.field(3, dtype=ti.f32, shape=self.shape) # force on body
        for e in itertools.product(*self.shape_ranges) :
            center = self.center_array[e]
            q = self.rot_array[e]
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
            rot_matrix = ti.Matrix([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
            v = self.velocity_array[e]
            w = angular_velocity_array[e]
            m = self.mass_array[e]
            radii = self.radii_array[e]

            self.init_x[e] = center
            self.init_quat[e] = q
            self.init_rot[e] = rot_matrix
            self.init_v[e] = v
            self.init_w[e] = w
            self.radii[e] = radii
            self.mass[e] = m
            self.massInv[e] = 1.0 / m
            self.ext_force[e] = m * self.gravity


        #base object properties, fields for simulating 
        self.x = ti.Vector.field(3, dtype=ti.f32, shape=self.shape) #x
        self.p = ti.Vector.field(3, dtype=ti.f32, shape=self.shape) #p for guessing step before projection
        self.quat = ti.Vector.field(4, dtype=ti.f32, shape=self.shape)
        self.rot = ti.Matrix.field(3, 3, dtype=ti.f32, shape=self.shape)
        self.new_quat = ti.Vector.field(4, dtype=ti.f32, shape=self.shape)
        self.new_rot = ti.Matrix.field(3, 3, dtype=ti.f32, shape=self.shape)
        self.v = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)  # linear velocity
        self.w = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)  # angular velocity

        #TO DO :
        # rigid object properties
        # self.type = ti.field(dtype=ti.i32, shape=self.shape)
        # self.inertia = ti.Matrix.field(3, 3, dtype=ti.f32, shape=self.shape)
        # self.inertiaInv = ti.Matrix.field(3, 3, dtype=ti.f32, shape=self.shape)
        # self.torque = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)  # torque

        self.reset_members()

    @ti.kernel
    def reset_members(self):
        '''
        Init with init_rot et init_x
        '''
        for idx in ti.grouped(self.x):
            for i in range(self.nV[idx]):
                self.new_V[idx, i] = self.init_rot[idx] @ self.V[idx, i] + self.init_x[idx]

        for idx in ti.grouped(self.x):
            self.x[idx] = self.init_x[idx]
            self.p[idx] = self.init_x[idx]
            self.quat[idx] = self.init_quat[idx]
            self.rot[idx] = self.init_rot[idx]
            self.new_quat[idx] = self.init_quat[idx]
            self.new_rot[idx] = self.init_rot[idx]
            self.v[idx] = self.init_v[idx]
            self.w[idx] = self.init_w[idx]
            self.ext_force[idx] = self.mass[idx] * self.gravity
            # self.mass[idx] = 1.0
            # self.massInv[idx] = 1.0 / self.mass[idx]
            #TO DO :
            # self.type[idx] = ObjType.DYNAMIC
            # self.scale[idx] = 1.0
            # self.w[idx] = [0.0, 0.0, 0.0]
            # self.torque[idx] = [0.0, 0.0, 0.0]

    @ti.kernel
    def recompute_COM(self):
        com = ti.Vector([0.0, 0.0, 0.0])
        for idx in ti.grouped(self.x):
            for i in range(self.nV[idx]):
                com += self.V[idx, i]
            com /= self.V.shape[-1]
            for i in range(self.nV[idx]):
                self.V[idx, i] -= com
            com = ti.Vector([0.0, 0.0, 0.0])

    @ti.func
    def update_new_positions(self):
        for idx in ti.grouped(self.x):
            for i in range(self.nV[idx]):
                self.new_V[idx, i] = self.rot[idx] @ self.V[idx, i] + self.x[idx]

    # @ti.func
    # def get_type(self, idx=ti.Vector([])):
    #     return self.type[idx]

    @ti.func
    def get_nb_of_particules(self) :
        return self.nb_of_particules
    
    @ti.func
    def get_nb_of_edges(self) :
        return self.nb_of_edges

    @ti.func
    def get_edge(self, idx) :
        return self.connections[idx]

    @ti.func
    def get_rest_distance(self, idx1, idx2) :
        return (self.init_x[idx1] - self.init_x[idx2]).norm()
    
    @ti.func
    def get_nb_of_bodies(self) :
        return self.nb_of_bodies
    
    @ti.func
    def get_body_indexes(self, idx) :
        '''
        Give a 2D int vector v such that
            - v.x = index_of_first_particule_of_body_idx,
            - v.y = index_of_last_particule_of_body_idx + 1
        To scan a body k :
        v = get_body_indexes(k)
        for idx in range(v.x, v.y) :
            ...
        '''
        return self.bodies_indexes[idx]
    
    @ti.func
    def get_adjacency(self, idx) :
        '''
        Give a 1D scalar field F, such that 
            - Its first value F[0] is the number of neighbors of particule idx
            - The following value are the index of its neighbors
        adj = get_adjacency(idx)
        for i in range(1, adj[0] + 1) :
            print(adj[i])
        '''
        return self.adjacency[idx]
    
    @ti.func
    def get_nb_of_neighbors(self, idx) :
        return self.adjacency[idx][0]

    @ti.func
    def get_x(self, idx = ti.Vector([])):
        return self.x[idx]
    
    @ti.func
    def set_x(self, x, idx = ti.Vector([])):
        self.x[idx] = x

    @ti.func
    def get_p(self, idx = ti.Vector([])):
        return self.p[idx]
    
    @ti.func
    def set_p(self, p, idx = ti.Vector([])):
        self.p[idx] = p
    
    @ti.func
    def get_rotation(self, idx = ti.Vector([])):
        return self.quat[idx]

    @ti.func
    def get_rotation_matrix(self, idx = ti.Vector([])):
        return self.rot[idx]
    
    @ti.func
    def set_rotation(self, q, idx = ti.Vector([])):
        self.quat[idx] = q
        self.rot[idx] = utils.quaternion_to_matrix(q)
    
    @ti.func
    def set_rotation_matrix(self, R, idx=ti.Vector([])):
        self.rot[idx] = R
        self.quat[idx] = utils.matrix_to_quaternion(R)
    
    @ti.func
    def get_predicted_rotation(self, idx = ti.Vector([])) :
        return self.new_quat[idx]
    
    @ti.func
    def get_predicted_rotation_matrix(self, idx = ti.Vector([])) :
        return self.new_rot[idx]
    
    @ti.func
    def set_predicted_rotation(self, q, idx = ti.Vector([])) : #From a q useful during the prediction/integration step
        self.new_quat[idx] = q
        self.new_rot[idx] = utils.quaternion_to_matrix(q)
    
    @ti.func
    def set_predicted_rotation_matrix(self, mat, idx = ti.Vector([])) : #From a mat as shape matching gives a matrix
        self.new_rot[idx] = mat
        self.new_quat[idx] = utils.matrix_to_quaternion(mat)
    
    @ti.func
    def set_color(self, c, idx=ti.Vector([])):
        for i in range(self.nV[idx]):
            self.C[(*idx, i)] = c

    @ti.func
    def set_colors(self, C, idx=ti.Vector([])):
        for i in range(self.nV[idx]):
            self.C[(*idx, i)] = C[i]

    @ti.func
    def get_radii(self, idx=ti.Vector([])):
        '''
        Return a taichi 3D Vector v such that v.x gives the principal axis in x, v.y the third (the normal) in y, v.z the second in z
        '''
        return self.radii[idx]

    @ti.func
    def get_mass(self, idx=ti.Vector([])):
        return self.mass[idx]

    #Pour l'instant on ne peut pas modifier la masse apres init
    # @ti.func
    # def set_mass(self, m, idx=ti.Vector([])):
    #     if self.type[idx] == int(ObjType.DYNAMIC):
    #         self.mass[idx] = m
    #         self.massInv[idx] = 1.0 / self.mass[idx]

    @ti.func
    def get_massInv(self, idx=ti.Vector([])):
        return self.massInv[idx]
    
    @ti.func
    def get_velocity(self, idx=ti.Vector([])):
        return self.v[idx]
    
    @ti.func
    def set_velocity(self, v, idx=ti.Vector([])) :
        self.v[idx] = v
    
    @ti.func
    def get_angular_velocity(self, idx=ti.Vector([])):
        return self.w[idx]
    
    @ti.func
    def set_angular_velocity(self, w, idx = ti.Vector([])) :
        self.w[idx] = w

    @ti.func
    def get_ext_force(self, idx=ti.Vector([])):
        return self.ext_force[idx]
    
    @ti.func
    def set_ext_force(self, f, idx=ti.Vector([])):
        self.ext_force[idx] = f
    
    # @ti.func
    # def get_scale(self, idx=ti.Vector([])):
    #     return self.scale[idx]

    # @ti.func
    # def get_inertia(self, idx=ti.Vector([])):
    #     return self.inertia[idx]

    # @ti.func
    # def get_inertiaInv(self, idx=ti.Vector([])):
    #     return self.inertiaInv[idx]

    # @ti.func
    # def get_inertia_world(self, idx=ti.Vector([])):
    #     return self.rot[idx] @ self.inertia[idx] @ self.rot[idx].inverse()

    # @ti.func
    # def get_inertiaInv_world(self, idx=ti.Vector([])):
    #     return self.rot[idx] @ self.inertiaInv[idx] @ self.rot[idx].inverse()

    # @ti.func
    # def get_angular_momentum(self, idx=ti.Vector([])):
    #     return self.get_inertia_world(idx) @ self.w[idx]

    # @ti.func
    # def get_velocity(self, p, idx=ti.Vector([])):
    #     return self.get_linear_velocity(idx) + self.get_angular_velocity(idx).cross(
    #         p - self.x[idx]
    #     )

    # @ti.func
    # def get_torque(self, idx=ti.Vector([])):
    #     return self.torque[idx]

    # @ti.func
    # def apply_force_to_COM(self, f, idx=ti.Vector([])):
    #     if self.type[idx] == int(ObjType.DYNAMIC):
    #         self.force[idx] += f

    # @ti.func
    # def apply_force(self, f, p, idx=ti.Vector([])):
    #     if self.type[idx] == int(ObjType.DYNAMIC):
    #         self.force[idx] += f
    #         self.torque[idx] += (p - self.x[idx]).cross(f)

    # @ti.func
    # def apply_torque(self, t, idx=ti.Vector([])):
    #     if self.type[idx] == int(ObjType.DYNAMIC):
    #         self.torque[idx] += t

    # @ti.func
    # def set_type(self, t, idx=ti.Vector([])):
    #     self.type[idx] = t

    #     if self.type[idx] == ObjType.STATIC:
    #         self.mass[idx] = 3e38
    #         self.massInv[idx] = 0
    #         self.inertia[idx] = ti.Matrix([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
    #         self.inertiaInv[idx] = ti.Matrix([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
    #         self.force[idx] = ti.Vector([0.0, 0.0, 0.0])
    #         self.torque[idx] = ti.Vector([0.0, 0.0, 0.0])

    # @ti.func
    # def set_scale(self, s, idx=ti.Vector([])):
    #     self.scale[idx] = s

    # @ti.func
    # def set_inertia(self, _I, idx=ti.Vector([])):
    #     if self.type[idx] == int(ObjType.DYNAMIC):
    #         self.inertia[idx] = _I
    #         self.inertiaInv[idx] = self.inertia[idx].inverse()

    # @ti.func
    # def set_angular_momentum(self, _l, idx=ti.Vector([])):
    #     if self.type[idx] == int(ObjType.DYNAMIC):
    #         self.w[idx] = self.get_inertiaInv_world(idx) @ _l

    # @ti.func
    # def set_linear_velocity(self, v, idx=ti.Vector([])):
    #     if self.type[idx] == int(ObjType.DYNAMIC):
    #         self.v[idx] = v

    # @ti.func
    # def set_angular_velocity(self, w, idx=ti.Vector([])):
    #     if self.type[idx] == int(ObjType.DYNAMIC):
    #         self.w[idx] = w

    # @ti.func
    # def set_torque(self, t, idx=ti.Vector([])):
    #     if self.type[idx] == int(ObjType.DYNAMIC):
    #         self.torque[idx] = t

    # @ti.func
    # def reset_force(self):
    #     for idx in ti.grouped(self.force):
    #         self.force[idx] = ti.Vector([0.0, 0.0, 0.0])

    # @ti.func
    # def reset_torque(self):
    #     for idx in ti.grouped(self.torque):
    #         self.torque[idx] = ti.Vector([0.0, 0.0, 0.0])
