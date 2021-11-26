import numpy as np
import itertools
import taichi as ti
import open3d as o3d
from taichi.lang.ops import sqrt
from ellipsoid_field import EllipsoidField
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
    # radii_array = np.array([[0.5, 0.1, 0.5], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    # #radii_array = np.array([[0.1, 0.5, 0.1], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    # ini_centers = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]]) + np.array([0., 5., 0.])
    # ini_rotation = np.array([[0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.]])
    # connections = np.array([[0, 1], [1, 3], [3, 2], [2, 0]])
    # bodies = np.array([0, 0, 0, 0])
    # ini_velocities = np.zeros(ini_centers.shape)
    # ini_mass = np.array([1., 1., 1., 10.])

    nb_of_ellipsoids = 8
    nb_of_pairs = 12#(nb_of_ellipsoids * (nb_of_ellipsoids - 1)) / 2
    #radii_array = np.array([[0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2]])
    radii_array = np.array([[0.5, 0.1, 0.5], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1],[0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2]])
    #radii_array = np.array([[0.1, 0.5, 0.1], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    ini_centers = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.],[1.5, 1.5, 0.], [2.5, 0., 0.], [0., 2.5, 0.], [2.5, 2.5, 0.]]) + np.array([0., 5., 0.])
    ini_rotation = np.array([[0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.]])
    connections = np.array([[0, 1], [1, 3], [3, 2], [2, 0], [4, 5], [5, 7], [7, 6], [6, 4]])
    bodies = np.array([0, 0, 0, 0, 1, 1, 1, 1])
    ini_velocities = np.zeros(ini_centers.shape)
    ini_angular_velocities = np.zeros(ini_centers.shape)
    ini_angular_velocities[0] = np.array([5., 0., 0.]) # For testing
    ini_mass = np.array([1., 1., 1., 10.,1., 10., 1., 1.])
    gravity = np.array([0., -9.8, 0.])

    def __init__(self, res=25):
        # create objects in the scene

        self.ellips_field = EllipsoidField(self.radii_array,
                                           self.ini_centers,
                                           self.ini_rotation,
                                           self.connections,
                                           self.bodies,
                                           self.ini_velocities,
                                           self.ini_angular_velocities,
                                           self.ini_mass,
                                           self.gravity,
                                           res=res,
                                           shape=(self.nb_of_ellipsoids,))

        self.dt = 3e-3
        self.t = 0.0
        self.cur_step = 0
        self.paused = True

        self.M_ground = ti.field(dtype=ti.i32, shape = ())
        self.ground_contacts = ti.Vector.field(2, dtype=ti.f32, shape=self.nb_of_ellipsoids)

        self.M_particles = ti.field(dtype=ti.i32, shape = ())
        self.particle_contacts = ti.Vector.field(2, dtype=ti.f32, shape=self.nb_of_pairs)
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

    @ti.kernel
    def advance(self, dt: ti.f32, t: ti.f32) :
        # Corresponds to one iteration of the position based algo page 3
        #Here stiffness defined for a one loop solver (cf page 5 position based dynamics)

        self.prologue_velocities()
        self.damp_velocities(1.)
        self.prologue_positions()
        #Seems there is not need to have a prologue for angular velocties because there is not ext. torque
        self.prologue_rotations()
        self.gen_collision_ground()
        self.generate_collisions_particle_T()
        #self.generate_collisions_particle()
        self.solve_collisions_particles()
        self.solve_collisions_ground()
        self.project_distance_constr(1.)
        self.epilogue()

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
            sum_mass = 0
            I = ti.Matrix([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
            L = ti.Vector([0., 0., 0.])
            w = ti.Vector([0., 0., 0.])
            for i in range(idx_first_ellips_body,idx_last_ellips_body):
                mass = self.ellips_field.get_mass(i)
                x_cm += self.ellips_field.get_x(i)*mass
                v_cm += self.ellips_field.get_velocity(i)*mass
 
                sum_mass += mass

            x_cm/=sum_mass
            v_cm/=sum_mass

            for i in range(idx_first_ellips_body,idx_last_ellips_body):
                mass = self.ellips_field.get_mass(i)
                r_i = self.ellips_field.get_x(i) - x_cm

                L += r_i.cross(self.ellips_field.get_velocity(i)*mass)

                r_i_tilde = utils.skew(r_i)
                I += mass*r_i_tilde@(r_i_tilde.transpose())

            w = (I.inverse())@L

            for i in range(idx_first_ellips_body,idx_last_ellips_body):
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
        a = radii.x #radii[0] #? radii[0] corresponds to a : raddius in X_elli
        b = radii.y #radii[1] #? radii[1] corresponds to b : raddius in Y_elli
        c = radii.z #radii[2] #? radii[2] corresponds to c : raddius in Z_elli
        distance_ground = self.ellips_field.get_p(idx)[1]
        return_value = None #Maybe should be initialized differently it is a float
        if distance_ground < max(a, b, c):  # approximation of particle with a sphere
            R = self.ellips_field.get_rotation_matrix(idx)
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
            p[1] = p[1] + d
            self.ellips_field.set_p(p, idx)

    @ti.func
    def generate_collisions_particle(self):
        k = 0
        for i in range(self.nb_of_ellipsoids):  # approximate collisions using spheres
            for j in range(i + 1, self.nb_of_ellipsoids):
                radii1 = self.ellips_field.get_radii(i)
                max1 = max(radii1[0], radii1[1], radii1[2])
                pos1 = self.ellips_field.get_x(i) #x or p ?

                radii2 = self.ellips_field.get_radii(j)
                max2 = max(radii2[0], radii2[1], radii2[2])
                pos2 = self.ellips_field.get_x(j) #x or p ?

                distance_vector = pos1 - pos2
                n = distance_vector.normalized()
                distance = distance_vector.norm()

                if distance < (max1 + max2):  # selection of candidates for collisions
                    # computing ellipsoids radius in direction n
                    radius1 = self.compute_radius(i, n)
                    radius2 = self.compute_radius(j, n)
                    distance = radius1 + radius2 - distance
                    if distance > 0:  # detected an approximate collisions.
                        # The distance between the particles is smaller than the sum of the radii
                        self.particle_contacts[k][0] = i
                        self.particle_contacts[k][1] = distance
                        self.direction_contacts[k] = n
                        k += 1
        self.M_particles[None] = k

    @ti.func
    def compute_radius(self, idx: ti.i32, n):
        radii = self.ellips_field.get_radii(idx)
        first_radius = radii[0]
        third_radius = radii[1]
        second_radius = radii[2]

        R = self.ellips_field.get_rotation_matrix(idx)
        elip_matrix = ti.Matrix([[first_radius ** 2, 0, 0], [0, second_radius ** 2, 0], [0, 0, third_radius ** 2]]) #R is in the same order as radii x,y,z? Yes cf ground collision
        inv_A = R @ elip_matrix @ R.transpose()

        x_loc = (1 / (ti.sqrt(n.transpose() @ inv_A @ n))[0]) * (inv_A @ n)
        radius = (x_loc).norm()
        return radius
    
    #A Second Version :

    @ti.func
    def generate_collisions_particle_T(self):
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
                    radius1 = self.compute_radius_T(i, n)
                    radius2 = self.compute_radius_T(j, n)
                    distance = radius1 + radius2 - distance
                    if distance > 0:  # detected an approximate collisions.
                        # The distance between the particles is smaller than the sum of the radii
                        self.particle_contacts[k][0] = j
                        self.particle_contacts[k][1] = distance
                        self.direction_contacts[k] = n
                        k += 1
        self.M_particles[None] = k

    @ti.func
    def compute_radius_T(self, idx: ti.i32, n):
        radii = self.ellips_field.get_radii(idx)
        a = radii.x
        b = radii.y
        c = radii.z
        elli_mat = ti.Matrix([[1./(a * a), 0., 0.], [0., 1./(b * b), 0.], [0., 0., 1./(c * c)]])

        R = self.ellips_field.get_rotation_matrix(idx)

        A = R @ elli_mat @ R.transpose()

        radius = sqrt(1. / (n.transpose() @ A @ n))[0]
        return radius

    @ti.func
    def solve_collisions_particles(self):
        for i in range(self.M_particles[None]) :    
            idx = int(self.particle_contacts[i][0])
            d = self.particle_contacts[i][1]
            n = self.direction_contacts[i]
            p = self.ellips_field.get_p(idx)
            p = p + d * n
            self.ellips_field.set_p(p, idx)
            pass

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

    # Note: inside body class, I could add additional constraints (local constraints) to
    # impose certain properties, like a fixed position in space. To do so, I should add a
    # small function called "project additional constraint"

    @ti.func
    def epilogue(self):
        for i in range(self.nb_of_ellipsoids):
            p = self.ellips_field.get_p(i)
            x = self.ellips_field.get_x(i)

            self.ellips_field.set_velocity((p - x) / self.dt, i)
            self.ellips_field.set_x(p, i)

            q_inv = utils.quaternion_inverse(self.ellips_field.get_rotation(i))
            new_q = self.ellips_field.get_predicted_rotation(i)
            qq = utils.quaternion_multiply(new_q, q_inv)
            w = utils.quaternion_to_angle(qq) / self.dt * utils.quaternion_to_axis(qq)

            self.ellips_field.set_rotation(new_q, i)
            self.ellips_field.set_angular_velocity(w, i)


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
