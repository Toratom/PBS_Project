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
    #   ## Add stifness per constraints

    def __init__(self,
     radii_array,
     center_array,
     rot_array,
     connections,
     velocity_array,
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

        if (self.radii_array.shape[:-1] != self.shape) :
            print("Error: radii_array does not have the correct shape!")
            print(self.radii_array.shape[:-1], "instead of", self.shape)
            return

        if (self.center_array.shape[:-1] != self.shape) :
            print("Error: center_array does not have the correct shape!")
            print(self.center_array.shape[:-1], "instead of", self.shape)
            return
        
        if (self.rot_array.shape[:-1] != self.shape) :
            print("Error: rot_array does not have the correct shape!")
            print(self.rot_array.shape[:-1], "instead of", self.shape)
            return
        
        if (self.radii_array.shape[-1] != 3) :
            print("Error: radii_array does not have the correct shape!")
            print(self.radii_array.shape[-1], "instead of", 3)
            return
        
        if (self.center_array.shape[-1] != 3) :
            print("Error: center_array does not have the correct shape!")
            print(self.center_array.shape[-1], "instead of", 3)
            return
        
        if (self.rot_array.shape[-1] != 4) :
            print("Error: rot_array does not have the correct shape!")
            print(self.rot_array.shape[-1], "instead of", 4)
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
            self.meshes[e] = o3d.geometry.TriangleMesh.create_sphere(radius = radii[0], resolution = res)
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

        #init base object properties : x, p, velocities, quat, rot, mass, radii
        self.init_x = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)
        self.init_quat = ti.Vector.field(4, dtype=ti.f32, shape=self.shape)
        self.init_rot = ti.Matrix.field(3, 3, dtype=ti.f32, shape=self.shape)
        self.init_v = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)
        self.radii = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)
        self.mass = ti.field(dtype=ti.f32, shape=self.shape)
        self.massInv = ti.field(dtype=ti.f32, shape=self.shape)
        self.ext_force = ti.Vector.field(3, dtype=ti.f32, shape=self.shape) # force on body
        for e in itertools.product(*self.shape_ranges):
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
            m = self.mass_array[e]
            radii = self.radii_array[e]

            self.init_x[e] = center
            self.init_quat[e] = q
            self.init_rot[e] = rot_matrix
            self.init_v[e] = v
            self.radii[e] = radii
            self.mass[e] = m
            self.massInv[e] = 1.0 / m
            self.ext_force[e] = m * self.gravity


        #base object properties, fields for simulating 
        self.x = ti.Vector.field(3, dtype=ti.f32, shape=self.shape) #x
        self.p = ti.Vector.field(3, dtype=ti.f32, shape=self.shape) #p for guessing step before projection
        self.quat = ti.Vector.field(4, dtype=ti.f32, shape=self.shape)
        self.rot = ti.Matrix.field(3, 3, dtype=ti.f32, shape=self.shape)
        self.v = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)  # linear velocity

        #TO DO :
        # rigid object properties
        # self.type = ti.field(dtype=ti.i32, shape=self.shape)
        # self.inertia = ti.Matrix.field(3, 3, dtype=ti.f32, shape=self.shape)
        # self.inertiaInv = ti.Matrix.field(3, 3, dtype=ti.f32, shape=self.shape)
        # self.w = ti.Vector.field(3, dtype=ti.f32, shape=self.shape)  # angular velocity
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
            self.v[idx] = self.init_v[idx]
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
    def get_x(self, idx = ti.Vector([])):
        return self.x[idx]
    
    @ti.func
    def set_x(self, x, idx = ti.Vector([])):
        self.x[idx] = x

    @ti.func
    def get_p(self, idx = ti.Vector([])):
        return self.p[idx]
    
    @ti.func
    def set_p(self, p, idx=ti.Vector([])):
        self.p[idx] = p
    
    @ti.func
    def get_rotation(self, idx=ti.Vector([])):
        return self.quat[idx]

    @ti.func
    def get_rotation_matrix(self, idx=ti.Vector([])):
        return self.rot[idx]
    
    @ti.func
    def set_rotation(self, q, idx=ti.Vector([])):
        self.quat[idx] = q
        self.rot[idx] = utils.quaternion_to_matrix(q)

    @ti.func
    def set_rotation_matrix(self, R, idx=ti.Vector([])):
        self.rot[idx] = R
        self.quat[idx] = utils.matrix_to_quaternion(R)
    
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
        Return a taichi 3D Vector v such that v.x gives the principal axis, v.y the third (the normal), v.z the second
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
    # def get_linear_velocity(self, idx=ti.Vector([])):
    #     return self.v[idx]

    # @ti.func
    # def get_angular_velocity(self, idx=ti.Vector([])):
    #     return self.w[idx]

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
