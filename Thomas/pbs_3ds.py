import pickle

import numpy as np
import itertools
import taichi as ti
import open3d as o3d
from taichi.lang.ops import sqrt
# from ellipsoid_field import EllipsoidField
from loader import Loader
import utils

ti.init(arch=ti.cpu)


@ti.data_oriented
class Simulation(object):
    ''' 
    A simulation is made of ellipsoid and a PBS solver
    '''

    # nb_of_ellipsoids = 4
    # nb_of_pairs = 6  # (nb_of_ellipsoids * (nb_of_ellipsoids - 1)) / 2
    # #radii_array = np.array([[0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2]])
    # #radii_array = np.array([[0.5, 0.1, 0.5], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    # #radii_array = np.array([[0.1, 0.5, 0.1], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    # radii_array = np.array([[0.45, 0.05, 0.45], [0.45, 0.05, 0.45],  [0.45, 0.05, 0.45],  [0.45, 0.05, 0.45]])
    # #radii_array = np.array([[0.45, 0.05, 0.45], [0.05, 0.45, 0.45],  [0.45, 0.05, 0.45],  [0.05, 0.45, 0.45]])
    # #ini_centers = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]]) + np.array([0., 5., 0.])
    # ini_centers = np.array([[0., 0., 0.], [0.5, 0.5, 0.], [0., 1., 0.], [-0.5, 0.5, 0.]]) + np.array([0., 5., 0.])
    # ini_rotation = np.array([[0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.]])
    # theta = np.radians(90.)
    # u = np.array([0., 0., -1.])
    # q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
    # ini_rotation[1] = q
    # u = np.array([0., 0., 1.])
    # q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
    # ini_rotation[3] = q
    # # connections = np.array([[0, 1], [1, 3], [3, 2], [2, 0]])
    # connections = np.array([[0, 1], [1, 2], [2, 3], [3, 0]])
    # #connections = np.array([[]])
    # bodies = np.array([0, 0, 0, 0])
    # ini_velocities = np.zeros(ini_centers.shape)
    # ini_angular_velocities = np.zeros(ini_centers.shape)
    # # ini_angular_velocities[3] = np.array([0., 0., 50.])
    # ini_mass = np.array([1., 1., 1., 1.])
    # gravity = np.array([0., -9.8, 0.])

    # Initialization for DEBUGGING -----------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # nb_of_ellipsoids = 8
    # nb_of_pairs = 12#(nb_of_ellipsoids * (nb_of_ellipsoids - 1)) / 2
    # #radii_array = np.array([[0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2]])
    # radii_array = np.array([[0.5, 0.1, 0.5], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1],[0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2]])
    # #radii_array = np.array([[0.1, 0.5, 0.1], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    # ini_centers = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.],[1.5, 1.5, 0.], [2.5, 0., 0.], [0., 2.5, 0.], [2.5, 2.5, 0.]]) + np.array([0., 5., 0.])
    # ini_rotation = np.array([[0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.]])
    # connections = np.array([[0, 1], [1, 3], [3, 2], [2, 0], [4, 5], [5, 7], [7, 6], [6, 4]])
    # bodies = np.array([0, 0, 0, 0, 1, 1, 1, 1])
    # ini_velocities = np.zeros(ini_centers.shape)
    # ini_angular_velocities = np.zeros(ini_centers.shape)
    # # ini_angular_velocities[0] = np.array([5., 0., 0.]) # For testing
    # ini_mass = np.array([1., 1., 1., 10.,1., 10., 1., 1.])
    # gravity = np.array([0., -9.8, 0.])
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------

    # nb_of_ellipsoids = 1
    # nb_of_pairs = 1 #(nb_of_ellipsoids * (nb_of_ellipsoids - 1)) / 2
    # radii_array = np.array([[0.5, 0.05, 0.5]]) #np.array([[0.5, 0.1, 0.5]])
    # ini_centers = np.array([[0., 0., 0.]]) + np.array([0., 5., 0.])
    # theta = np.radians(90)
    # u = np.array([0., 0., 1.])
    # q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
    # #q = np.array([0., 0., 0., 1.])
    # # q = q / np.linalg.norm(q)
    # ini_rotation = np.array([q]) #[x, y, z, w]
    # connections = np.array([[]])
    # bodies = np.array([0])
    # ini_velocities = np.zeros(ini_centers.shape)
    # ini_velocities[0] = 0. * np.array([1., -1., 0.])
    # ini_angular_velocities = np.zeros(ini_centers.shape)
    # ini_angular_velocities[0] = np.array([0., 0., 0.]) # np.array([0., 0., 0.]) # For testing
    # ini_mass = np.array([1.])
    # gravity = np.array([0., -9.8, 0.])




    # Initialization for PRESENTATION --------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------

    #----------- DUCK:
    # with open('Meshes/duck.pkl', 'rb') as inp:
    #     graph = pickle.load(inp)

    # nb_of_ellipsoids = graph["centers"].shape[0]
    # nb_of_pairs = graph["connections"].shape[0] #582
    # print(nb_of_pairs)
    # height = np.array([0,8,0])
    # radii_array = graph["radii"]
    # ini_centers = graph["centers"] + height
    # ini_rotation = graph["rotations"]
    # connections = graph["connections"]
    # bodies = np.array([0 for i in range(nb_of_ellipsoids)])  # To be changed for multiple ducks
    # ini_velocities = np.zeros(ini_centers.shape)
    # ini_angular_velocities = np.zeros(ini_centers.shape)
    # # ini_angular_velocities[0] = np.array([5., 0., 0.]) # For testing
    # ini_mass = np.array([10 for i in range(nb_of_ellipsoids)]) * 100
    # gravity = np.array([0., -9.8, 0.])

    '''nb_of_ellipsoids = 28
    nb_of_pairs = 72  # (nb_of_ellipsoids * (nb_of_ellipsoids - 1)) / 2
    # radii_array = np.array([[0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2]])
    offset = -0.5
    height = 2
    radii_array = np.array(
        [[0.05, 0.05, 0.05],  # Cube1 vertexes
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.5, 0.5, 0.05],  # front
         [0.5, 0.5, 0.05],  # back
         [0.5, 0.05, 0.5],  # up
         [0.5, 0.05, 0.5],  # down
         [0.05, 0.5, 0.5],  # right
         [0.05, 0.5, 0.5],
         [0.05, 0.05, 0.05],  # Cube2 vertexes
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.5, 0.5, 0.05],  # front
         [0.5, 0.5, 0.05],  # back
         [0.5, 0.05, 0.5],  # up
         [0.5, 0.05, 0.5],  # down
         [0.05, 0.5, 0.5],  # right
         [0.05, 0.5, 0.5]
         ])  # left
    # radii_array = np.array([[0.1, 0.5, 0.1], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    ini_centers = np.array(
        [[0., 0., 0.],  #0 CUBE 1
         [1., 0., 0.],  #1
         [0., 1., 0.],  #2
         [1., 1., 0.],  #3
         [0., 0., 1.],  #4
         [1., 0., 1.],  #5
         [0., 1., 1.],  #6
         [1., 1., 1.],  #7
         [0.5, 0.5, 0.],  #front 8
         [0.5, 0.5, 1.],  #back 9
         [0.5, 1., 0.5],  #up 10
         [0.5, 0., 0.5],  #down 11
         [0., 0.5, 0.5],  #right 12
         [1., 0.5, 0.5],  #left 13
         [0. + offset, 0. + height, 0.],  # 0 CUBE 2
         [1. + offset, 0. + height, 0.],  # 1
         [0. + offset, 1. + height, 0.],  # 2
         [1. + offset, 1. + height, 0.],  # 3
         [0. + offset, 0. + height, 1.],  # 4
         [1. + offset, 0. + height, 1.],  # 5
         [0. + offset, 1. + height, 1.],  # 6
         [1. + offset, 1. + height, 1.],  # 7
         [0.5 + offset, 0.5 + height, 0.],  # front 8
         [0.5 + offset, 0.5 + height, 1.],  # back 9
         [0.5 + offset, 1. + height, 0.5],  # up 10
         [0.5 + offset, 0. + height, 0.5],  # down 11
         [0. + offset, 0.5 + height, 0.5],  # right 12
         [1. + offset, 0.5 + height, 0.5]  # left 13
         ]) + np.array([0., 1., 0.])
    ini_rotation = np.array([[0., 0., 0., 1.], # CUBE1
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],  # CUBE2
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.]
                             ])
    shift = 14
    connections = np.array([[0, 1], [0, 2], [0, 4], [0, 8], [0, 11], [0, 13], # CUBE1
                            [1, 3], [1, 5], [1, 8], [1, 11], [1, 12],
                            [2, 6], [2, 3], [2, 8], [2, 10], [2, 13],
                            [3, 7], [3, 8], [3, 10], [3, 12],
                            [4, 6], [4, 5], [4, 13], [4, 11], [4, 9],
                            [5, 7], [5, 11], [5, 12], [5, 9],
                            [6, 7], [6, 10], [6, 13], [6, 9],
                            [7, 10], [7, 12], [7, 9],
                            [0 + shift, 1 + shift], [0 + shift, 2 + shift], [0 + shift, 4 + shift], [0 + shift, 8 + shift], [0 + shift, 11 + shift], [0 + shift, 13 + shift],  # CUBE2
                            [1 + shift, 3 + shift], [1 + shift, 5 + shift], [1 + shift, 8 + shift], [1 + shift, 11 + shift], [1 + shift, 12 + shift],
                            [2 + shift, 6 + shift], [2 + shift, 3 + shift], [2 + shift, 8 + shift], [2 + shift, 10 + shift], [2 + shift, 13 + shift],
                            [3 + shift, 7 + shift], [3 + shift, 8 + shift], [3 + shift, 10 + shift], [3 + shift, 12 + shift],
                            [4 + shift, 6 + shift], [4 + shift, 5 + shift], [4 + shift, 13 + shift], [4 + shift, 11 + shift], [4 + shift, 9 + shift],
                            [5 + shift, 7 + shift], [5 + shift, 11 + shift], [5 + shift, 12 + shift], [5 + shift, 9 + shift],
                            [6 + shift, 7 + shift], [6 + shift, 10 + shift], [6 + shift, 13 + shift], [6 + shift, 9 + shift],
                            [7 + shift, 10 + shift], [7 + shift, 12 + shift], [7 + shift, 9 + shift]
                            ])
    bodies = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
    ini_velocities = np.zeros(ini_centers.shape)
    ini_angular_velocities = np.zeros(ini_centers.shape)
    # ini_angular_velocities[0] = np.array([5., 0., 0.]) # For testing
    ini_mass = np.array([10., 10., 10., 10., 10., 10., 10., 10., 40., 40., 40., 40., 40., 40., 10., 10., 10., 10., 10., 10., 10., 10., 40., 40., 40., 40., 40., 40.]) * 100
    gravity = np.array([0., -9.8, 0.])'''

    #-------- USING THE LAODER --------
    loader = Loader()

    #Duck 1
    theta = np.radians(90.)
    u = np.array([-1., 0., 0.])
    q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
    loader.add_body('Meshes/duck_pbs.glb', 'Meshes/duck.pkl', q, np.array([0., 8., 0.]))

    #Duck 2
    theta = np.radians(90.)
    u = np.array([1., 0., 0.])
    q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
    loader.add_body('Meshes/duck_pbs.glb', 'Meshes/duck.pkl', q, np.array([0., 18., 0.]))

    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------

    def __init__(self, res = 5):
        # create objects in the scene

        # self.ellips_field = EllipsoidField(self.radii_array,
        #                                    self.ini_centers,
        #                                    self.ini_rotation,
        #                                    self.connections,
        #                                    self.bodies,
        #                                    self.ini_velocities,
        #                                    self.ini_angular_velocities,
        #                                    self.ini_mass,
        #                                    self.gravity,
        #                                    res=res,
        #                                    shape=(self.nb_of_ellipsoids,))
        self.ellips_field = self.loader.generate_ellipsoids_field()
        self.nb_of_ellipsoids = self.loader.get_nb_of_ellipsoids()
        self.nb_of_pairs = self.loader.get_nb_of_edges() #582

        self.dt = 3e-3
        self.t = 0.0
        self.cur_step = 0
        self.paused = True

        self.M_ground = ti.field(dtype=ti.i32, shape = ())
        self.ground_contacts = ti.Vector.field(2, dtype=ti.f32, shape=self.nb_of_ellipsoids)

        self.M_particles = ti.field(dtype=ti.i32, shape = ())
        self.particle_contacts = ti.Vector.field(3, dtype=ti.f32, shape=self.nb_of_pairs) #modified to get the pairs i,j of particle colliding
        self.direction_contacts = ti.Vector.field(3, dtype=ti.f32, shape=self.nb_of_pairs)

        self.init()

    # @ti.kernel
    # def set_sim_init(self,dt: ti.f32):
    #     # set initial condition of simulation
    #     # momentum = self.force * ti.Vector([ti.cos(self.angle), ti.sin(self.angle), 0])
    #     # self.ellips_field.set_linear_momentum(momentum, 0)
    #     pass

    def update_meshes_and_lines(self):
        # Update meshes (ellipsoids)
        new_V = self.ellips_field.new_V.to_numpy()
        C = self.ellips_field.C.to_numpy()
        for e in itertools.product(*self.ellips_field.shape_ranges):
            self.ellips_field.meshes[e].vertices = o3d.utility.Vector3dVector(
                new_V[e][: self.ellips_field.nV[e]]
            )
            self.ellips_field.meshes[e].vertex_colors = o3d.utility.Vector3dVector(
                C[e][: self.ellips_field.nV[e]]
            )
        # Updates lines
        centers = self.ellips_field.x.to_numpy()
        self.ellips_field.lines.points = o3d.utility.Vector3dVector(centers)

    def init(self):
        # reset non taichi-scope variables here
        self.t = 0.0
        self.cur_step = 0
        self.ellips_field.reset_members()
        # self.set_sim_init(self.dt)
        self.update_meshes_and_lines()

    def step(self):
        if self.paused:
            return
        self.t += self.dt
        self.cur_step += 1
        self.advance(
            self.dt,
            self.t,
        )
        self.update_meshes_and_lines()
        # self.paused = True

    @ti.kernel
    def advance(self, dt: ti.f32, t: ti.f32) :
        # Corresponds to one iteration of the position based algo page 3
        #Here stiffness defined for a one loop solver (cf page 5 position based dynamics)

        self.prologue_velocities()
        # self.damp_velocities(0.1) #0.1 (stiff), 1 (soft) #Needs to be comment if there is only one ellipsoid...
        self.prologue_positions()
        #Seems there is not need to have a prologue for angular velocties because there is not ext. torque
        self.prologue_rotations()
        self.gen_collision_ground()
        self.generate_collisions_particle()
        self.solve_collisions_particles()
        self.solve_collisions_ground()
        # self.project_distance_constr(1.)
        self.project_shape_matching_constr(0.1) # 0.35 (stiff), 0.01 soft
        self.epilogue(2.) #the bigger, less there is damping during collision #1.
        self.friction_ground(0.5, 0.1) # 0.5, 0.5 in 2D #4, 4 3D cube #0.01, 1
        self.friction_particles(0.5, 0.5)

        self.ellips_field.update_new_positions()  # IMPORTANT TO KEEP, NEEDED TO COMPUTE V_NEW !!


    @ti.func
    def prologue_velocities(self):

        for i in range(self.nb_of_ellipsoids):
            # Update velocities using forces external
            v = self.ellips_field.get_velocity(i)
            f = self.ellips_field.get_ext_force(i)
            inv_m = self.ellips_field.get_massInv(i)

            # Decoment for a tastier animation
            # if (i == 0 or i == 1) : f = 5*f

            v = v + self.dt * inv_m * f

            self.ellips_field.set_velocity(v, i)

    @ti.func
    def damp_velocities(self, stiffness):
        '''
        Damp velocities body per body
        stiffness must be a float between 0 and 1
        '''

        for index_body in range(len(self.bodies)):
            bodies_indexes = self.ellips_field.get_body_indexes(index_body)
            idx_last_ellips_body = bodies_indexes[1]
            idx_first_ellips_body = bodies_indexes[0]

            x_cm = ti.Vector([0., 0., 0.])
            v_cm = ti.Vector([0., 0., 0.])
            sum_mass = 0.
            I = ti.Matrix([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
            L = ti.Vector([0., 0., 0.])
            w = ti.Vector([0., 0., 0.])
            for i in range(idx_first_ellips_body, idx_last_ellips_body):
                mass = self.ellips_field.get_mass(i)
                x_cm += self.ellips_field.get_x(i)*mass
                v_cm += self.ellips_field.get_velocity(i)*mass

                sum_mass += mass

            x_cm/=sum_mass
            v_cm/=sum_mass

            for i in range(idx_first_ellips_body, idx_last_ellips_body):
                mass = self.ellips_field.get_mass(i)
                r_i = self.ellips_field.get_x(i) - x_cm

                L += r_i.cross(self.ellips_field.get_velocity(i)*mass)

                r_i_tilde = utils.skew(r_i)
                I += mass*r_i_tilde@(r_i_tilde.transpose())

            w = (I.inverse())@L

            for i in range(idx_first_ellips_body, idx_last_ellips_body):
                v = self.ellips_field.get_velocity(i)

                r_i = self.ellips_field.get_x(i) - x_cm
                dv_i = v_cm + w.cross(r_i) - v
                #print(v,r_i,x_cm,v_cm,w.cross(r_i),dv_i)
                v += stiffness*dv_i
                self.ellips_field.set_velocity(v, i)

    @ti.func
    def prologue_positions(self):
        for i in range(self.nb_of_ellipsoids):
            x = self.ellips_field.get_x(i)
            v = self.ellips_field.get_velocity(i)

            p = x + self.dt * v

            self.ellips_field.set_p(p, i)

    @ti.func
    def prologue_rotations(self):
        for i in range(self.nb_of_ellipsoids):
            #I use here the integration of q saw in lectures, not the one propose
            w = self.ellips_field.get_angular_velocity(i)
            wq = ti.Vector([w.x, w.y, w.z, 0])
            q = self.ellips_field.get_rotation(i)
            dq = 0.5 * self.dt * utils.quaternion_multiply(wq, q)

            predicted_q = q + dq
            predicted_q = predicted_q.normalized()

            self.ellips_field.set_predicted_rotation(predicted_q, i)

    @ti.func
    def gen_collision_ground(self):
        k = 0  # M is the number of collisions
        for i in range(self.nb_of_ellipsoids):
            # Looking for collision candidates
            d = self.possible_ground_coll(i)
            if d > 0. :
                self.ground_contacts[k][0] = i
                self.ground_contacts[k][1] = d
                k += 1
        self.M_ground[None] = k  # rectification of the number of collisions with the ground after for cycle

    @ti.func
    def possible_ground_coll(self, idx: ti.i32):
        radii = self.ellips_field.get_radii(idx)
        a = radii.x #radii[0] corresponds to a : raddius in X_elli
        b = radii.y #radii[1] corresponds to b : raddius in Y_elli
        c = radii.z #radii[2] corresponds to c : raddius in Z_elli
        distance_ground = self.ellips_field.get_p(idx)[1]
        return_value = None #Maybe should be initialized differently it is a float
        if distance_ground < max(a, b, c):  # approximation of particle with a sphere
            R = self.ellips_field.get_predicted_rotation_matrix(idx) #R is the predicted R as the rotation is updated in the prologue
            n = ti.Vector([0., 1., 0.])
            elip_matrix = ti.Matrix([[a * a, 0., 0.], [0., b * b, 0.], [0., 0., c * c]])
            inv_A = R @ elip_matrix @ R.transpose()

            x1_loc = (1. / (ti.sqrt(n.transpose() @ inv_A @ n))[0]) * (inv_A @ n)
            x2_loc = - x1_loc
            x1 = self.ellips_field.get_p(idx) + x1_loc
            x2 = self.ellips_field.get_p(idx) + x2_loc
            x = x1[1] if x1[1] < x2[1] else x2[1]

            return_value = -x if x < 0. else 0. #return_value is 0 if no collision
        return return_value

    @ti.func
    def solve_collisions_ground(self):
        for i in range(self.M_ground[None]):
            idx = int(self.ground_contacts[i][0])
            d = self.ground_contacts[i][1]

            p = self.ellips_field.get_p(idx)
            p[1] += d
            self.ellips_field.set_p(p, idx)

    @ti.func
    def generate_collisions_particle(self):
        k = 0
        for i in range(self.nb_of_ellipsoids):  # approximate collisions using spheres
            for j in range(i + 1, self.nb_of_ellipsoids):
                radii1 = self.ellips_field.get_radii(i)
                max1 = max(radii1[0], radii1[1], radii1[2])
                pos1 = self.ellips_field.get_p(i)

                radii2 = self.ellips_field.get_radii(j)
                max2 = max(radii2[0], radii2[1], radii2[2])
                pos2 = self.ellips_field.get_p(j)

                distance_vector = pos2 - pos1
                n = distance_vector.normalized()
                distance = distance_vector.norm()

                if distance < (max1 + max2):  # selection of candidates for collisions
                    # computing ellipsoids x_1 and x_2 the 
                    radius1 = self.compute_radius(i, n)
                    radius2 = self.compute_radius(j, n)
                    distance = radius1 + radius2 - distance
                    if distance > 0:  # detected an approximate collisions.
                        # The distance between the particles is smaller than the sum of the radii
                        self.particle_contacts[k][0] = j
                        self.particle_contacts[k][1] = distance
                        self.particle_contacts[k][2] = i
                        self.direction_contacts[k] = n
                        k += 1
        self.M_particles[None] = k

    @ti.func
    def compute_radius(self, idx: ti.i32, n):
        radii = self.ellips_field.get_radii(idx)
        a = radii.x
        b = radii.y
        c = radii.z
        elli_mat = ti.Matrix([[1./(a * a), 0., 0.], [0., 1./(b * b), 0.], [0., 0., 1./(c * c)]])

        R = self.ellips_field.get_predicted_rotation_matrix(idx)

        A = R @ elli_mat @ R.transpose()

        radius = sqrt(1. / (n.transpose() @ A @ n))[0]
        return radius

    @ti.func
    def solve_collisions_particles(self):
        for i in range(self.M_particles[None]) :
            # idx = int(self.particle_contacts[i][0])
            # d = self.particle_contacts[i][1]
            # n = self.direction_contacts[i]
            # p = self.ellips_field.get_p(idx)
            # p = p + d * n
            # self.ellips_field.set_p(p, idx)

            #maybe better to move both particles by d/2 ?

            idxj = int(self.particle_contacts[i][0])
            idxi = int(self.particle_contacts[i][2])
            d = self.particle_contacts[i][1]
            n = self.direction_contacts[i]
            pi = self.ellips_field.get_p(idxi)
            pj = self.ellips_field.get_p(idxj)
            pi = pi - (d/2) * n
            pj = pj + (d/2) * n
            self.ellips_field.set_p(pi, idxi)
            self.ellips_field.set_p(pj, idxj)


    @ti.func
    def project_distance_constr(self, stiffness):
        '''
        stiffness must be a float between 0 and 1
        '''
        for i in range(self.ellips_field.get_nb_of_edges()):
            edge = self.ellips_field.get_edge(i)  # gives a 2D taichi vector

            p1_idx = edge.x
            p2_idx = edge.y
            p1 = self.ellips_field.get_p(p1_idx)
            p2 = self.ellips_field.get_p(p2_idx)
            n = (p1 - p2).normalized()
            distance = (p1 - p2).norm() - self.ellips_field.get_rest_distance(p1_idx, p2_idx)
            inv_m1 = self.ellips_field.get_massInv(p1_idx)
            inv_m2 = self.ellips_field.get_massInv(p2_idx)
            inv_m_total = inv_m1 + inv_m2

            delta1 = - (inv_m1 / inv_m_total) * distance * n
            delta2 = (inv_m2 / inv_m_total) * distance * n

            p1 += stiffness * delta1
            p2 += stiffness * delta2

            self.ellips_field.set_p(p1, p1_idx)
            self.ellips_field.set_p(p2, p2_idx)

    @ti.func
    def project_shape_matching_constr(self, stiffness) :
        '''
        Stiffness between 0. and 1., 1. gives rigid bodies, close to 0. gives more springy, soft bodies
        '''
        for idx in range(self.nb_of_ellipsoids):
            x_0 = self.ellips_field.get_rest_x(idx)
            p = self.ellips_field.get_p(idx)
            m = self.ellips_field.get_mass(idx)
            radii = self.ellips_field.get_radii(idx)
            R_0 = self.ellips_field.get_rest_rotation_matrix(idx) #Test
            new_R = self.ellips_field.get_predicted_rotation_matrix(idx)

            nb_of_neighbors = self.ellips_field.get_nb_of_neighbors(idx)

            #Compute A
            #Init with the particule itself
            #A_elli = 0.2 * m * (new_R @ ti.Matrix([[radii.x**2., 0., 0.], [0., radii.y**2, 0.], [0., 0., radii.z**2]]) @ new_R.transpose()) #Test
            #A_elli = 0.2 * m * ti.Matrix([[radii.x**2., 0., 0.], [0., radii.y**2, 0.], [0., 0., radii.z**2]]) @ new_R
            A_elli = 0.2 * m * ti.Matrix([[radii.x**2., 0., 0.], [0., radii.y**2, 0.], [0., 0., radii.z**2]]) @ new_R @ R_0.transpose() #Test
            A = A_elli + m * (p @ x_0.transpose())
            new_c = m * p
            c_0 = m * x_0
            M = m
            if (nb_of_neighbors > 0) :
                for i in range(nb_of_neighbors) :
                    neighbor_idx = self.ellips_field.get_neighbor(idx, i)
                    x_0 = self.ellips_field.get_rest_x(neighbor_idx)
                    p = self.ellips_field.get_p(neighbor_idx)
                    m = self.ellips_field.get_mass(neighbor_idx)
                    radii = self.ellips_field.get_radii(neighbor_idx)
                    R_0 = self.ellips_field.get_rest_rotation_matrix(neighbor_idx) #Test
                    new_R = self.ellips_field.get_predicted_rotation_matrix(neighbor_idx)

                    # A_elli = 0.2 * m * (new_R @ ti.Matrix([[radii.x**2., 0., 0.], [0., radii.y**2, 0.], [0., 0., radii.z**2]]) @ new_R.transpose()) #Test BOF BOF
                    # A_elli = 0.2 * m * ti.Matrix([[radii.x**2., 0., 0.], [0., radii.y**2, 0.], [0., 0., radii.z**2]]) @ new_R
                    A_elli = 0.2 * m * ti.Matrix([[radii.x**2., 0., 0.], [0., radii.y**2, 0.], [0., 0., radii.z**2]]) @ new_R @ R_0.transpose() #Test
                    A += A_elli + m * (p @ x_0.transpose())
                    new_c += m * p
                    c_0 += m * x_0
                    M += m

            new_c /= M
            c_0 /= M
            A -= M * (new_c @ c_0.transpose())

            #Polar Decomposition of A
            R, _ = ti.polar_decompose(A, ti.f32) #ti.f32 or ti.f64 ?

            #Update position by moving each particule toward its g_i, begin with idx
            x_0 = self.ellips_field.get_rest_x(idx)
            p = self.ellips_field.get_p(idx)
            g = R @ (x_0 - c_0) + new_c

            p += stiffness * (g - p) #Need to check the math
            self.ellips_field.set_p(p, idx)
            if (nb_of_neighbors > 0) :
                for i in range(nb_of_neighbors) :
                    neighbor_idx = self.ellips_field.get_neighbor(idx, i)
                    x_0 = self.ellips_field.get_rest_x(neighbor_idx)
                    p = self.ellips_field.get_p(neighbor_idx)
                    g = R @ (x_0 - c_0) + new_c

                    p += stiffness * (g - p)
                    self.ellips_field.set_p(p, neighbor_idx)

            #Update Orientation of idx
            R_0 = self.ellips_field.get_rest_rotation_matrix(idx) #Test
            R = R @ R_0 #Test
            self.ellips_field.set_predicted_rotation_matrix(R, idx)

    # Note: inside body class, I could add additional constraints (local constraints) to
    # impose certain properties, like a fixed position in space. To do so, I should add a
    # small function called "project additional constraint"

    @ti.func
    def epilogue(self, sig):
        for i in range(self.nb_of_ellipsoids):
            p = self.ellips_field.get_p(i)
            x = self.ellips_field.get_x(i)
            new_v = (p - x) / self.dt

            old_vel = self.ellips_field.get_velocity(i)
            delta_vel = new_v - old_vel
            lamb = (delta_vel).norm() / old_vel.norm()
            self.ellips_field.set_color([lamb * 1., (1 - lamb) * 1., 0.], [i]) #DEBUG
            new_v = old_vel + ti.exp(- lamb**2 / sig**2) * delta_vel #1.2 the bigger the less damping #Test

            self.ellips_field.set_velocity(new_v, i)
            self.ellips_field.set_x(p, i)

            q_inv = utils.quaternion_inverse(self.ellips_field.get_rotation(i))
            new_q = self.ellips_field.get_predicted_rotation(i).normalized()
            qq = utils.quaternion_multiply(new_q, q_inv)
            #There is an ambiguity : two valid rotations to go from q to new_q, qq (theta / u) et -qq (2PI - theta / -u)
            #Choose the shorter one i.e. smallest theta, besides qq.w = cos(theta / 2) so shortest has qq.w > 0
            #REALLY IMPORTANT, ortherwise lot of instabillity if new_q ~= -q can lead to huge angular velocity...
            if (qq.w < 0.) :
                qq *= -1.
            w = utils.quaternion_to_angle(qq) / self.dt * utils.quaternion_to_axis(qq)

            self.ellips_field.set_rotation(new_q, i)
            # self.ellips_field.set_rotation_matrix(self.ellips_field.get_predicted_rotation_matrix(i), i)
            self.ellips_field.set_angular_velocity(w, i)

    @ti.func
    def friction_ground(self,slin,srot):

        # #colission with solid object (here ground)
        for i in range(self.M_ground[None]):
            idx = int(self.ground_contacts[i][0])

            v = self.ellips_field.get_velocity(idx)
            n = ti.Vector([0., 1., 0.])
            v_solid = ti.Vector([0., 0., 0.]) #ground
            v_diff_n_orthog = v_solid - v - (v_solid - v).dot(n) * n #perpendicular to n
            v += v_diff_n_orthog * slin

            r = - self.compute_radius(idx, n) * n #Test adding -1*

            w = self.ellips_field.get_angular_velocity(idx)
            # w *= (1. - srot) #scale down w
            w += r.cross(v_solid - v - w.cross(r)) * srot / r.norm()**2

            self.ellips_field.set_velocity(v, idx)
            self.ellips_field.set_angular_velocity(w, idx)

    @ti.func
    def friction_particles(self,slin,srot):
        #collision with particles

        for idxpairs in range(self.M_particles[None]) :
            j = int(self.particle_contacts[idxpairs][0])
            i = int(self.particle_contacts[idxpairs][2])

            n = self.direction_contacts[idxpairs]
            radius1 = self.compute_radius(i, n)*n
            radius2 = -self.compute_radius(j, n)*n

            v1 = self.ellips_field.get_velocity(i)
            v2 = self.ellips_field.get_velocity(j)

            v1_diff_n_orthog = (v1+v2)/2 - v1 - ((v1+v2)/2 - v1).dot(n)*n #perpendicular to n
            v2_diff_n_orthog = (v1+v2)/2 - v2 - ((v1+v2)/2 - v2).dot(n)*n #perpendicular to n

            v1 += v1_diff_n_orthog*slin
            v2 += v2_diff_n_orthog*slin

            w1 = self.ellips_field.get_angular_velocity(i)
            w2 = self.ellips_field.get_angular_velocity(j)

            v_avg = (v1 + w1.cross(radius1) + v2 + w2.cross(radius2))/2
            w1 += radius1.cross(v_avg-v1-w1.cross(radius1))*srot/radius1.norm()**2
            w2 += radius2.cross(v_avg-v2-w2.cross(radius2))*srot/radius2.norm()**2

            self.ellips_field.set_velocity(v1,i)
            self.ellips_field.set_angular_velocity(w1, i)

            self.ellips_field.set_velocity(v2,j)
            self.ellips_field.set_angular_velocity(w2, j)


    #@ti.func
    # def generate_collisions_particle_D(self):
    #     k = 0
    #     for i in range(self.nb_of_ellipsoids):  # approximate collisions using spheres
    #         for j in range(i + 1, self.nb_of_ellipsoids):
    #             radii1 = self.ellips_field.get_radii(i)
    #             max1 = max(radii1[0], radii1[1], radii1[2])
    #             pos1 = self.ellips_field.get_x(i) #x or p ?

    #             radii2 = self.ellips_field.get_radii(j)
    #             max2 = max(radii2[0], radii2[1], radii2[2])
    #             pos2 = self.ellips_field.get_x(j) #x or p ?

    #             distance_vector = pos1 - pos2
    #             n = distance_vector.normalized()
    #             distance = distance_vector.norm()

    #             if distance < (max1 + max2):  # selection of candidates for collisions
    #                 # computing ellipsoids radius in direction n
    #                 radius1 = self.compute_radius_D(i, n)
    #                 radius2 = self.compute_radius_D(j, n)
    #                 distance = radius1 + radius2 - distance
    #                 if distance > 0:  # detected an approximate collisions.
    #                     # The distance between the particles is smaller than the sum of the radii
    #                     self.particle_contacts[k][0] = i
    #                     self.particle_contacts[k][1] = distance
    #                     self.direction_contacts[k] = n
    #                     k += 1
    #     self.M_particles[None] = k

    # @ti.func
    # def compute_radius_D(self, idx: ti.i32, n):
    #     radii = self.ellips_field.get_radii(idx)
    #     first_radius = radii[0]
    #     third_radius = radii[1]
    #     second_radius = radii[2]

    #     R = self.ellips_field.get_rotation_matrix(idx)
    #     elip_matrix = ti.Matrix([[first_radius ** 2, 0, 0], [0, second_radius ** 2, 0], [0, 0, third_radius ** 2]]) #R is in the same order as radii x,y,z? Yes cf ground collision
    #     inv_A = R @ elip_matrix @ R.transpose()

    #     x_loc = (1 / (ti.sqrt(n.transpose() @ inv_A @ n))[0]) * (inv_A @ n)
    #     radius = (x_loc).norm()
    #     return radius


def main():
    sim = Simulation()

    # setup gui
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()

    # key callback functions
    def init(vis):
        print("reset simulation")
        sim.init()
        vis.reset_view_point(True)

    def pause(vis):
        sim.paused = not sim.paused

    vis.register_key_callback(ord("R"), init)
    vis.register_key_callback(ord(" "), pause)  # space

    # add some default primitives
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1, origin=[0, 0, 0]
    )
    vis.add_geometry(coordinate_frame)  # coordinate frame

    points = (
            [[i, 0, -10] for i in range(-10, 11)]
            + [[i, 0, 10] for i in range(-10, 11)]
            + [[-10, 0, i] for i in range(-10, 11)]
            + [[10, 0, i] for i in range(-10, 11)]
    )
    lines = [[i, i + 21] for i in range(21)] + [[i + 42, i + 63] for i in range(21)]
    colors = [[0.7, 0.7, 0.7] for i in range(len(lines))]
    ground_plane = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    ground_plane.colors = o3d.utility.Vector3dVector(colors)
    vis.add_geometry(ground_plane, True)  # ground plane

    # aabb = o3d.geometry.AxisAlignedBoundingBox(
    #     min_bound=np.array([-10, 0, -10]), max_bound=np.array([10, 20, 10])
    # )
    # aabb.color = [0.7, 0.7, 0.7]
    # vis.add_geometry(aabb)  # bounding box

    # add ellipsoids to visualization
    for i in range(sim.nb_of_ellipsoids):
        vis.add_geometry(sim.ellips_field.meshes[i])

    # add lines to vizualtiztion
    vis.add_geometry(sim.ellips_field.lines)

    # for i in range(5):  # wireframes of the walls
    #     shell = o3d.geometry.LineSet.create_from_triangle_mesh(
    #         sim.ellips_field.meshes[i + sim.NUM_CUBES]
    #     )
    #     vis.add_geometry(shell)

    # print(vis.get_render_option().line_width)
    # vis.get_render_option().line_width = 20.
    # print(vis.get_render_option().line_width)

    while True:
        sim.step()

        # Update of meshes and then of lines
        for mesh in sim.ellips_field.meshes.ravel():
            vis.update_geometry(mesh)
        vis.update_geometry(sim.ellips_field.lines)

        if not vis.poll_events():
            break
        vis.update_renderer()


if __name__ == "__main__":
    main()

# # render and view options
# rdr = vis.get_render_option()
# rdr.mesh_show_back_face = True
# #rdr.mesh_show_wireframe = True
# ctr = vis.get_view_control()
# ctr.set_lookat([0.0, 0.5, 0.0])
# ctr.set_up([0.0, 1.0, 0.0])