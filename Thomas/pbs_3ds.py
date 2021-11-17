import numpy as np
import itertools
import taichi as ti
import open3d as o3d
from ellipsoid_field import EllipsoidField

ti.init(arch=ti.cpu)

### Parameters
paused = False

@ti.data_oriented
class Simulation(object):
    ''' 
    A simulation is made of ellipsoid and a PBS solver
    '''

    nb_of_ellipsoids = 4
    radii_array = np.array([[0.5, 0.2, 0.5], [0.5, 0.2, 0.5], [0.5, 0.2, 0.5], [0.5, 0.2, 0.5]])
    ini_centers = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]])
    ini_rotation = np.array([[0., 0., 0., 1.],
     [-0.3444844, -0.3444844, -0.8733046, 0], 
     [-0.3444844, -0.3444844, -0.8733046, 0],
     [-0.3444844, -0.3444844, -0.8733046, 0]])

    def __init__(self, res = 120):
        # create objects in the scene

        self.objects = EllipsoidField(self.radii_array, self.ini_centers, self.ini_rotation, res = res, shape = (self.nb_of_ellipsoids,))
        # self.PBS_Solber = PBS_SOLVER ...

        self.dt = 3e-3
        self.t = 0.0
        self.cur_step = 0
        self.paused = True
        # self.mass = 1.0
        # self.angle = 1.047
        # self.force = 10.0
        # self.show_contacts = False
        # self.gravity = np.array([0.0, -9.81, 0.0])
        # self.eps = 1.0

        self.init()

    @ti.kernel
    def set_sim_init(self, dt: ti.f32):
        # set initial condition of simulation
        # momentum = self.force * ti.Vector([ti.cos(self.angle), ti.sin(self.angle), 0])
        # self.objects.set_linear_momentum(momentum, 0)
        pass
    
    def update_meshes(self):
        new_V = self.objects.new_V.to_numpy()
        C = self.objects.C.to_numpy()
        for e in itertools.product(*self.objects.shape_ranges):
            self.objects.meshes[e].vertices = o3d.utility.Vector3dVector(
                new_V[e][: self.objects.nV[e]]
            )
            self.objects.meshes[e].vertex_colors = o3d.utility.Vector3dVector(
                C[e][: self.objects.nV[e]]
            )
    
    def init(self):
        # reset non taichi-scope variables here
        self.t = 0.0
        self.cur_step = 0
        self.objects.reset_members()
        self.set_sim_init(self.dt)
        self.update_meshes()

    # @ti.kernel
    # def advance(
    #     self,
    #     dt: ti.f32,
    #     t: ti.f32,
    #     method: ti.i32,
    #     broad_phase_method: ti.i32,
    #     narrow_phase_method: ti.i32,
    # ):
    #     self.collisionDetection.compute_collision_detection(
    #         broad_phase_method, narrow_phase_method, self.eps
    #     )

    #     for i in range(self.NUM_CUBES + 5):
    #         self.objects.apply_force_to_COM(ti.Vector(self.gravity), i)

    #     for i in range(self.NUM_CUBES + 5):
    #         # integrate velocities
    #         self.objects.set_linear_momentum(
    #             self.objects.get_linear_momentum(i) + dt * self.objects.get_force(i), i
    #         )
    #         self.objects.set_angular_momentum(
    #             self.objects.get_angular_momentum(i) + dt * self.objects.get_torque(i),
    #             i,
    #         )

    #         # integrate position
    #         self.objects.set_position(
    #             self.objects.get_position(i) + dt * self.objects.get_linear_velocity(i),
    #             i,
    #         )

    #         # integrate rotation (same as in ex2)
    #         w = self.objects.get_angular_velocity(i)

    #         if method == self.METHOD_MATRIX:
    #             W = utils.skew(w)
    #             R = self.objects.get_rotation_matrix(i)
    #             self.objects.set_rotation_matrix(R + dt * (W @ R), i)
    #         elif method == self.METHOD_MATRIX_ORTH:
    #             W = utils.skew(w)
    #             R = self.objects.get_rotation_matrix(i)
    #             U, sigma, V = ti.svd(R + dt * (W @ R), ti.f32)
    #             self.objects.set_rotation_matrix(U @ V.transpose(), i)
    #         elif method == self.METHOD_QUATERNION:
    #             wq = ti.Vector([w.x, w.y, w.z, 0])
    #             q = self.objects.get_rotation(i)
    #             dq = utils.quaternion_multiply(wq, q)
    #             new_q = q + 0.5 * dt * dq
    #             self.objects.set_rotation(new_q.normalized(), i)

    #     self.objects.reset_force()
    #     self.objects.reset_torque()
    #     self.objects.update_new_positions()

    def step(self):
        if self.paused:
            return
        self.t += self.dt
        self.cur_step += 1
        # self.advance(
        #     self.dt,
        #     self.t,
        #     self.method,
        #     self.broad_phase_method,
        #     self.narrow_phase_method,
        # )
        self.update_meshes()
        # debug here: e.g. print(self.collisionDetection.sizes)


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
        size = 1, origin=[0, 0, 0]
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

    # simulation geometries for rendering
    for i in range(sim.nb_of_ellipsoids):
        vis.add_geometry(sim.objects.meshes[i])

    # for i in range(5):  # wireframes of the walls
    #     shell = o3d.geometry.LineSet.create_from_triangle_mesh(
    #         sim.objects.meshes[i + sim.NUM_CUBES]
    #     )
    #     vis.add_geometry(shell)

    while True:
        sim.step()

        for mesh in sim.objects.meshes.ravel():
            vis.update_geometry(mesh)

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