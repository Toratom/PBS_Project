import taichi as ti
import open3d as o3d
import numpy as np

###############################################################
## Simulation

ti.init(arch=ti.cpu)


@ti.data_oriented
class ParticleSystem(object):
    def __init__(self, N=50, dt=2e-5, substeps=50):
        self.N = N
        self.dt = dt
        self.substeps = 50
        self.paused = False

        self.x = ti.Vector.field(
            3, float, self.N, needs_grad=True
        )  # particle positions
        self.v = ti.Vector.field(3, float, self.N)  # particle velocities
        self.U = ti.field(float, (), needs_grad=True)  # potential energy

        self.init()

    @ti.kernel
    def init(self):
        for i in self.x:
            self.x[i] = [ti.random(), ti.random(), ti.random()]
            self.v[i] = [0, 0, 0]
        self.U[None] = 0

    @ti.kernel
    def compute_U(self):
        for i, j in ti.ndrange(self.N, self.N):
            r = self.x[i] - self.x[j]
            # r.norm(1e-3) is equivalent to ti.sqrt(r.norm()**2 + 1e-3)
            # This is to prevent 1/0 error which can cause wrong derivative
            self.U[None] += -1 / r.norm(1e-3)  # U += -1 / |r|

    @ti.kernel
    def advance(self):
        for i in self.x:
            self.v[i] += self.dt * -self.x.grad[i]  # dv/dt = -dU/dx
            self.x[i] += self.dt * self.v[i]  # dx/dt = v

    def step(self):
        if self.paused:
            return
        for _ in range(self.substeps):
            with ti.Tape(self.U):
                # Kernel invocations in this scope contribute to partial derivatives of
                # U with respect to input variables such as x.
                self.compute_U()  # The tape will automatically compute dU/dx and save the results in x.grad
            self.advance()


def main():
    p = ParticleSystem()

    ###################################
    # setup gui
    gui = o3d.visualization.VisualizerWithKeyCallback()
    gui.create_window()

    # key callback functions
    def init(vis):
        p.init()
        vis.reset_view_point(True)

    def pause(vis):
        p.paused = not p.paused

    gui.register_key_callback(ord("R"), init)
    gui.register_key_callback(ord(" "), pause)  # space

    # add some default primitives
    aabb = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=np.zeros(3), max_bound=np.ones(3)
    )
    gui.add_geometry(aabb)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.1, origin=[0, 0, 0]
    )
    gui.add_geometry(coordinate_frame)

    # simulation geometries for rendering
    pcd = o3d.geometry.PointCloud()
    pcd.paint_uniform_color([1, 0.706, 0])
    gui.add_geometry(pcd)
    ###################################

    while True:
        p.step()

        # update simulation geometries for rendering
        pcd.points = o3d.utility.Vector3dVector(p.x.to_numpy())
        pcd.normals = o3d.utility.Vector3dVector(p.v.to_numpy())
        gui.update_geometry(pcd)

        gui.poll_events()
        gui.update_renderer()

if __name__ == "__main__":
    main()
