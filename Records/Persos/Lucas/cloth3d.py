import sys
import numpy as np
import taichi as ti
import open3d as o3d
import utils
from functools import partial

ti.init(arch=ti.cpu)

### Parameters

N = 128
W = 2
L = W / N
gravity = 9.8
mass = 0.1
stiffness = 600 # cloth (generic) stiffness 
object_stiffness = 10000
damping = 1.5
steps = 15
dt = 1e-3
paused = False

num_balls = 1
ball_radius = 0.3
ball_centers = ti.Vector.field(3, float, num_balls)

x = ti.Vector.field(3, float, (N, N))
v = ti.Vector.field(3, float, (N, N))
fint = ti.Vector.field(3, float, (N, N))
fext = ti.Vector.field(3, float, (N, N))
fdamp = ti.Vector.field(3, float, (N, N))

num_triangles = (N - 1) * (N - 1) * 2
indices = ti.Vector.field(3, int, num_triangles)
vertices = ti.Vector.field(3, float, N * N)

links = ti.Vector.field(2, int, 8) # relative positions of neighbors connected with a spring
links.from_numpy(np.array([[-1, 0], [1, 0], [0, -1], [0, 1], [-1, -1], [1, 1], [1, -1], [-1, 1]]))
links_end = 8 # take the first 8 elements of links


### EX 5 - Simulation 

@ti.kernel
def init():
    for i, j in ti.ndrange(N, N):
        x[i, j] *= 0
        v[i, j] *= 0
        fint[i, j] *= 0
        fext[i, j] *= 0
        fdamp[i, j] *= 0

    for i, j in ti.ndrange(N, N):
        x[i, j] = ti.Vector(
            [(i + 0.5) * L - 0.5 * W, (j + 0.5) * L / ti.sqrt(2) + 1.0, (N - j) * L / ti.sqrt(2) - 0.4 * W]
        )

        if i < N - 1 and j < N - 1:
            tri_id = ((i * (N - 1)) + j) * 2
            indices[tri_id].x = i * N + j
            indices[tri_id].y = (i + 1) * N + j
            indices[tri_id].z = i * N + (j + 1)

            tri_id += 1
            indices[tri_id].x = (i + 1) * N + j + 1
            indices[tri_id].y = i * N + (j + 1)
            indices[tri_id].z = (i + 1) * N + j
    ball_centers[0] = ti.Vector([0.0, 0.5, 0.0])


@ti.func
def objectBoundPenalty(pos, vel, center, radius, bounce_magnitude=0.1):
    ret = vel * 0
    # TODO: return the penalty-based response wrt to the ball AND the ground (y=0)
    # write your code here
    if (pos[1]<0 and vel[1]<0):
        distance_ground = ti.Vector([0,-pos[1],0])
        ret = object_stiffness * distance_ground
    
    distance_ball_center = (pos-center).norm()

    if (distance_ball_center<radius):
        direction = pos-center
        direction /= direction.norm()
        if (vel.dot(direction)<0):
            ret = object_stiffness * (radius-distance_ball_center) * direction

    return ret


@ti.kernel
def substep(links_end: ti.i32):
    for i in ti.grouped(x):
        # TODO: internal forces

        fint[i] = x[i] * 0
        for k in range(links_end):
            translation = links[k]
            if ((i + translation).x>0 and (i + translation).y>0 and (i + translation).x<N and (i + translation).y<N):
                neighbor = x[i + translation]
                L0 = L
                if (k>3):
                    L0 = L*(2**0.5)
                distance = (neighbor-x[i]).norm()
                direction = (neighbor-x[i])/distance
                fint[i] += stiffness * (distance-L0)*direction / L0

        # TODO: external forces (gravity)
        fext[i] = mass*gravity*ti.Vector([0,-1,0]) # replace this by your code

        # external forces (elastic spring penalty)
        for b in range(num_balls):
            fext[i] += objectBoundPenalty(x[i], v[i], ball_centers[b], ball_radius * 1.01)

        # TODO: damping forces
        fdamp[i] = -v[i] * damping # replace this by your code

    # TODO: semi-implicit Euler update
    for i in ti.grouped(x):
        v[i] += dt*(fint[i] + fext[i] + fdamp[i])/mass
        x[i] += dt*v[i]


@ti.kernel
def update_verts():
    for i, j in ti.ndrange(N, N):
        vertices[i * N + j] = x[i, j]


### GUI

# key callback functions
def reset_sim(vis):
    init()
    update_verts()

def pause_sim(vis):
    global paused
    paused = not paused

def set_mss(vis, type):
    global links_end
    if type == 1:
        links_end = 8
    elif type == 2:
        links_end = 4
    elif type == 3:
        links_end = 6


# init and setup gui
utils.print_pbs()
vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window()
reset_sim(vis)

# set callbacks
vis.register_key_callback(ord("R"), reset_sim)
vis.register_key_callback(ord(" "), pause_sim)  # space
vis.register_key_callback(ord("1"), partial(set_mss, type=1)) # crossed
vis.register_key_callback(ord("2"), partial(set_mss, type=2)) # opposite
vis.register_key_callback(ord("3"), partial(set_mss, type=3)) # regular

# MSS cloth mesh
V = o3d.utility.Vector3dVector(vertices.to_numpy())
F = o3d.utility.Vector3iVector(indices.to_numpy())
mesh = o3d.geometry.TriangleMesh(V, F)
mesh.paint_uniform_color([0.5, 0.5, 0.5])
mesh.compute_vertex_normals()
mesh.compute_triangle_normals()
vis.add_geometry(mesh)

# sphere mesh
sphere = o3d.geometry.TriangleMesh.create_sphere(radius=ball_radius, resolution=120)
sphere.translate(ball_centers[0])
sphere.paint_uniform_color([1.0, 0.4, 0.2])
sphere.compute_vertex_normals()
vis.add_geometry(sphere)

# plane mesh
points = (
        [[i/10, 0, -1] for i in range(-10, 11)] #i/10 : lignes éloignées de i/10 sur x, -1 : ligne éloigné de 1 dans la direction -z
        + [[i/10, 0, 1] for i in range(-10, 11)]
        + [[-1, 0, i/10] for i in range(-10, 11)]
        + [[1, 0, i/10] for i in range(-10, 11)]
    )
lines = [[i, i + 21] for i in range(21)] + [[i + 42, i + 63] for i in range(21)] #on lie d'abord les points entre -z et z puis entre -x et x
colors = [[0.7, 0.7, 0.7] for i in range(len(lines))]
ground_plane = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)
ground_plane.colors = o3d.utility.Vector3dVector(colors)
vis.add_geometry(ground_plane)

# render and view options
rdr = vis.get_render_option()
rdr.mesh_show_back_face = True
#rdr.mesh_show_wireframe = True
ctr = vis.get_view_control()
ctr.set_lookat([0.0, 0.5, 0.0])
ctr.set_up([0.0, 1.0, 0.0])

while True:
    update_verts()

    if not paused:
        for i in range(steps):
            substep(links_end)

    mesh.vertices = o3d.utility.Vector3dVector(vertices.to_numpy())
    mesh.triangles = o3d.utility.Vector3iVector(indices.to_numpy())
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()
    vis.update_geometry(mesh)

    if not vis.poll_events():
        break
    vis.update_renderer()
