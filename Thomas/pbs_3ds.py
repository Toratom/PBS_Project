import numpy as np
import itertools
import taichi as ti
import open3d as o3d
from ellipsoid_field import EllipsoidField

ti.init(arch=ti.cpu)


@ti.data_oriented
class Simulation(object):
    ''' 
    A simulation is made of ellipsoid and a PBS solver
    '''

    nb_of_ellipsoids = 4
    nb_of_pairs = 6  # (nb_of_ellipsoids * (nb_of_ellipsoids - 1)) / 2
    radii_array = np.array([[0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [0.5, 0.5, 0.5]])
    ini_centers = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]]) + np.array([0., 10., 0.])
    ini_rotation = np.array([[0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.]])
    connections = np.array([[0, 1], [1, 3], [3, 2], [2, 0]])
    bodies = np.array([0, 0, 0, 0])
    ini_velocities = np.zeros(ini_centers.shape)
    ini_mass = np.array([10., 10., 10., 10.])
    gravity = np.array([0., -9.8, 0.])

    def __init__(self, res=25):
        # create objects in the scene

        self.ellips_field = EllipsoidField(self.radii_array,
                                           self.ini_centers,
                                           self.ini_rotation,
                                           self.connections,
                                           self.bodies,
                                           self.ini_velocities,
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
    def advance(
            self,
            dt: ti.f32,
            t: ti.f32
    ):
        # Corresponds to one iteration of the position based algo page 3
        #Here stifness defined for a one loop solver (cf page 5 position based dynamics)

        self.prologue_velocities()
        self.prologue_positions()
        self.gen_collision_ground()
        # self.generate_collisions_particle()
        # self.solve_collisions_particles()
        self.solve_collisions_ground()
        self.project_distance_constr(1.)
        self.epilogue()

        self.ellips_field.update_new_positions()  # IMPORTANT TO KEEP, NEEDED TO COMPUTE V_NEW !!

    #     self.collisionDetection.compute_collision_detection(
    #         broad_phase_method, narrow_phase_method, self.eps
    #     )

    #     for i in range(self.NUM_CUBES + 5):
    #         self.ellips_field.apply_force_to_COM(ti.Vector(self.gravity), i)

    #     for i in range(self.NUM_CUBES + 5):
    #         # integrate velocities
    #         self.ellips_field.set_linear_momentum(
    #             self.ellips_field.get_linear_momentum(i) + dt * self.ellips_field.get_force(i), i
    #         )
    #         self.ellips_field.set_angular_momentum(
    #             self.ellips_field.get_angular_momentum(i) + dt * self.ellips_field.get_torque(i),
    #             i,
    #         )

    #         # integrate position
    #         self.ellips_field.set_p(
    #             self.ellips_field.get_p(i) + dt * self.ellips_field.get_linear_velocity(i),
    #             i,
    #         )

    #         # integrate rotation (same as in ex2)
    #         w = self.ellips_field.get_angular_velocity(i)

    #         if method == self.METHOD_MATRIX:
    #             W = utils.skew(w)
    #             R = self.ellips_field.get_rotation_matrix(i)
    #             self.ellips_field.set_rotation_matrix(R + dt * (W @ R), i)
    #         elif method == self.METHOD_MATRIX_ORTH:
    #             W = utils.skew(w)
    #             R = self.ellips_field.get_rotation_matrix(i)
    #             U, sigma, V = ti.svd(R + dt * (W @ R), ti.f32)
    #             self.ellips_field.set_rotation_matrix(U @ V.transpose(), i)
    #         elif method == self.METHOD_QUATERNION:
    #             wq = ti.Vector([w.x, w.y, w.z, 0])
    #             q = self.ellips_field.get_rotation(i)
    #             dq = utils.quaternion_multiply(wq, q)
    #             new_q = q + 0.5 * dt * dq
    #             self.ellips_field.set_rotation(new_q.normalized(), i)

    #     self.ellips_field.reset_force()
    #     self.ellips_field.reset_torque()
    #     self.ellips_field.update_new_positions() #IMPORTANT TO KEEP, NEEDED TO COMPUTE V_NEW !!

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
            # Damp velocities
            v = self.damp(v, 1.)

            self.ellips_field.set_velocity(v, i)

    @ti.func
    def damp(self, velocity, stifness):
        '''
        stifness must be a float between 0 and 1
        '''
        return velocity

    @ti.func
    def prologue_positions(self):
        for i in range(self.nb_of_ellipsoids):
            x = self.ellips_field.get_x(i)
            v = self.ellips_field.get_velocity(i)

            p = x + self.dt * v

            self.ellips_field.set_p(p, i)

    @ti.func
    def gen_collision_ground(self):
        k = 0  # M is the number of collisions
        for i in range(self.nb_of_ellipsoids):
            # Looking for collision candidates
            d = self.possible_ground_coll(i)
            if d > 0:
                self.ground_contacts[k][0] = i
                self.ground_contacts[k][1] = d
                k += 1
        self.M_ground[None] = k  # rectification of the number of collisions with the ground after for cycle

    @ti.func
    def possible_ground_coll(self, idx: ti.i32):
        radii = self.ellips_field.get_radii(idx)
        first_radius = radii[0]
        third_radius = radii[1]
        second_radius = radii[2]
        distance_ground = self.ellips_field.get_p(idx)[1]
        return_value = None
        if distance_ground < max(first_radius, third_radius, second_radius):  # approximation of particle with a sphere
            R = self.ellips_field.get_rotation_matrix(idx)
            n = ti.Vector([0, 1, 0])
            elip_matrix = ti.Matrix([[first_radius ** 2, 0, 0], [0, second_radius ** 2, 0], [0, 0, third_radius ** 2]])
            inv_A = R @ elip_matrix @ R.transpose()

            x1 = (1 / (ti.sqrt(n.transpose() @ inv_A @ n))[0]) * (inv_A @ n)
            x2 = - x1
            x = x1[1] if x1[1] < x2[1] else x2[1]

            return_value = abs(x) if x < 0 else x
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
                pos1 = self.ellips_field.get_x(i)

                radii2 = self.ellips_field.get_radii(j)
                max2 = max(radii2[0], radii2[1], radii2[2])
                pos2 = self.ellips_field.get_x(j)

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
        elip_matrix = ti.Matrix([[first_radius ** 2, 0, 0], [0, second_radius ** 2, 0], [0, 0, third_radius ** 2]]) #R is in the same order as radii x,y,z?
        inv_A = R @ elip_matrix @ R.transpose()

        x = (1 / (ti.sqrt(n.transpose() @ inv_A @ n))[0]) * (inv_A @ n)
        center = self.ellips_field.get_p(idx)
        radius = (x - center).norm()
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
