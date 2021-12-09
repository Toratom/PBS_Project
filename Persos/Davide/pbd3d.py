import taichi as ti
import numpy as np


# TODO: prima costruisci lo spring system come in pbd_pendule, ma in 3D. Poi estendi a collisioni tra corpi.
#  bisogna estendere SUBITO il codice per poter gestire, di base, più corpi insieme. Usa dei dizionari.

# TODO: bisogna creare una funzione per la generazione dei vincoli di collisione. Per semplificare
#  questa fase (ottimizzazione delle prestazioni) si può introdurre una struttura dati che contiene
#  coppie (id_oggetto, particella) e contiene i neighbors di ogni particella.

# Helper class used to store the particles associated to each object in the simulation
# This class contains the positions and the velocities of the particles of each body.
# It also contains internal constraints, like distance constraints.
# Inter-bodies constraints (like collisions and positions) are handled by the PBD-solver class.
@ti.data_oriented
class Body(object):
    # Set the id of the body (a graph of particles)
    def __init__(self, id, num_particles, d, dt):
        # self.num_particles = structure["num_particles"]
        # self.d = structure["d"]
        self.id = id
        self.d = d
        self.dt = dt
        # self.structure = structure
        self.num_particles = num_particles
        self.p = ti.Vector.field(3, dtype=ti.f32, shape=self.num_particles)
        self.x = ti.Vector.field(3, dtype=ti.f32, shape=self.num_particles)
        self.v = ti.Vector.field(3, dtype=ti.f32, shape=self.num_particles)
        self.ext_forces = ti.Vector.field(3, dtype=ti.f32, shape=self.num_particles)
        self.inv_mass = ti.field(dtype=ti.f32, shape=self.num_particles)
        self.distance_constr = ti.Vector.field(2, dtype=ti.f32, shape=self.num_particles)

    # Set and initialize num_particles, positions, velocities, masses, constraints of the body
    # connections is a list of connected edges (pairs of connected particles), d is the tolerated distance in
    # the distance constraint.
    @ti.kernel
    def set_simulation(self, positions: ti.ext_arr(), velocities: ti.ext_arr(), masses: ti.ext_arr(),
                       connections: ti.ext_arr(), gravity: ti.f32):
        '''num_particles = self.structure["num_particles"]
        positions = self.structure["positions"]
        velocities = self.structure["velocities"]
        masses = self.structure["masses"]
        connections = self.structure["connections"]
        d = self.structure["d"]
        gravity = self.structure["gravity"]'''

        # setting initial position of particles (before the simulation)
        # self.p = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)
        num_particles = self.num_particles

        for i in range(num_particles):
            self.p[i][0] = positions[i, 0]  # particle_positions is a python matrix
            self.p[i][1] = positions[i, 1]
            self.p[i][2] = positions[i, 2]

        # Preparing ti.Vector to store proposed positions
        # self.x = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)

        # setting initial velocity of particles (before the simulation)
        # self.v = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.v[i][0] = velocities[i, 0]
            self.v[i][1] = velocities[i, 1]
            self.v[i][2] = velocities[i, 2]

        # setting external forces
        # self.ext_forces = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.ext_forces[i][1] = masses[i] * gravity  # initialize forces with gravity

        # setting values of masses
        # self.inv_mass = ti.field(ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.inv_mass[i] = 1 / masses[i]

        # Setting distance constraint (springs)
        # self.distance_constr = ti.Vector.field(2, dtype=ti.int31, shape=connections.__len__())
        for i in range(connections.shape[0]):
            self.distance_constr[i][0] = connections[i, 0]
            self.distance_constr[i][1] = connections[i, 1]

    # Generate the distance constraints (spring system of the soft body)
    '''@ti.func
    def generate_distance_constr(self, connections: ti.ext_arr(), d: ti.f32):
        self.d = d
        # self.distance_constr = ti.Vector.field(2, dtype=ti.int31, shape=connections.__len__())
        for i in connections.__len__():
            self.distance_constr[i][0] = connections[i, 0]
            self.distance_constr[i][1] = connections[i, 1]'''

    @ti.kernel
    def prologue_velocities(self):
        for i in range(self.num_particles):
            # Update velocities using forces external
            self.v[i] = self.v[i] + self.dt * self.ext_forces[i] * self.inv_mass[i]

            # Damp velocities
            self.v[i] = self.damp(self.v[i])

    @ti.func
    def damp(self, velocity):
        return velocity

    @ti.kernel
    def prologue_positions(self):
        for i in range(self.num_particles):
            self.x[i] = self.p[i] + self.dt * self.v[i]

    @ti.kernel
    def project_distance_constr(self):
        for i in range(self.num_particles):
            p1_idx = self.distance_constr[i][0]
            p2_idx = self.distance_constr[i][1]
            p1 = self.x[p1_idx]
            p2 = self.x[p2_idx]
            n = (p1 - p2).normalized()
            distance = (p1 - p2).norm() - self.d
            inv_m_total = self.inv_mass[p1_idx] - self.inv_mass[p2_idx]

            delta1 = - (self.inv_mass[p1_idx] / inv_m_total) * distance * n
            delta2 = (self.inv_mass[p2_idx] / inv_m_total) * distance * n

            self.x[p1_idx] += delta1
            self.x[p2_idx] += delta2

    # Note: inside body class, I could add additional constraints (local constraints) to
    # impose certain properties, like a fixed position in space. To do so, I should add a
    # small function called "project additional constraint"

    @ti.kernel
    def epilogue(self):
        for i in range(self.num_particles):
            self.v[i] = (self.p[i] - self.x[i]) / self.dt
            self.p[i] = self.x[i]


@ti.data_oriented
class PBDSolver(object):
    # TODO:
    #  Altrimenti, fa un semplice dizionario di nome constraint, ti.vector in cui metti tutti i constraints.
    #  Nota: nel dizionario metti solo i constraints della classe Solver (quelli globali)
    #  Poi costruisci delle funzioni per costruire e proiettare i ti.vector e le chiami al momento giusto
    # Initialize function is used to set the constants
    def __init__(self):
        self.dt = 3e-3
        self.t = 0.0
        self.paused = True
        self.gravity = -9.8
        self.iterations = 20
        self.n_constraints = 0
        self.bodies = {}

    # Introduce a new object in the scene
    # Note: this function is in python scope because it is used only to call taichi kernels
    def set_simulation(self, num_particles, positions, velocities, masses, connections, d, id):
        if self.bodies.__contains__(id):
            print("ERROR: this body already exists")
        else:
            # Encapsulating all the initialization values inside a dictionary.
            # Taichi kernels can receive only scalars as parameters.
            '''structure = {}
            structure["num_particles"] = num_particles
            structure["positions"] = positions
            structure["velocities"] = velocities
            structure["masses"] = masses
            structure["connections"] = connections
            structure["d"] = d
            structure["gravity"] = self.gravity'''

            body = Body(id, num_particles, d, self.dt)
            body.set_simulation(positions, velocities, masses, connections, self.gravity)
            self.bodies[id] = body

    # Prologue of velocities for each body in the scene
    # Note: this function is in python scope because it is used only to call taichi kernels
    def prologue_velocities(self):
        for j in self.bodies.keys():
            body = self.bodies[j]
            body.prologue_velocities()

    # Prologue of positions for each body in the scene
    # Note: this function is in python scope because it is used only to call taichi kernels
    def prologue_positions(self):
        for j in self.bodies.keys():
            body = self.bodies[j]
            body.prologue_positions()

    # Projection of constraints for each object in the scene
    # Note: this function is in python scope because it is used only to call taichi kernels
    def project_constraints(self):
        # projection of distance constraint (springs)
        for j in self.bodies.keys():
            body = self.bodies[j]
            body.project_distance_constr()

    # Epilogue for each object in the scene
    # Note: this function is in python scope because it is used only to call taichi kernels
    def epilogue(self):
        for j in self.bodies.keys():
            body = self.bodies[j]
            body.epilogue()


def main():
    ti.init(arch=ti.gpu)
    solver = PBDSolver()
    solver.set_simulation(3,  # num_particles
                          np.array([[0, 0, 0], [0, 1, 0], [0, 2, 0]]),  # positions
                          np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]),  # velocities
                          np.array([1, 1, 1]),  # masses
                          np.array([[0, 1], [1, 2]]),  # connections
                          0.2,  # d
                          "test")  # id
    solver.prologue_velocities()
    solver.prologue_positions()
    solver.project_constraints()
    solver.epilogue()

    print("DEBUGGED")


main()
