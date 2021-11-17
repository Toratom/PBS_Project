import taichi as ti


# TODO: prima costruisci lo spring system come in pbd_pendule, ma in 3D. Poi estendi a collisioni tra corpi.
#  bisogna estendere SUBITO il codice per poter gestire, di base, più corpi insieme. Usa dei dizionari.

# Helper class used to store the particles associated to each object in the simulation
# This class contains the positions and the velocities of the particles of each body.
# It also contains internal constraints, like distance constraints.
# Inter-bodies constraints (like collisions and positions) are handled by the PBD-solver class.
@ti.data_oriented
class Body(object):
    # Set the id of the body (a graph of particles)
    def __init__(self, id):
        self.id = id

    # Set and initialize num_particles, positions, velocities, masses, constraints of the body
    # connections is a list of connected edges (pairs of connected particles), d is the tolerated distance in
    # the distance constraint.
    @ti.kernel
    def set_simulation(self, num_particles, positions, velocities, masses, connections, d, gravity):

        self.num_particles = num_particles

        # setting initial position of particles (before the simulation)
        self.p = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.p[i] = positions[i]  # particle_positions is a python matrix

        # Preparing ti.Vector to store proposed positions
        self.x = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)

        # setting initial velocity of particles (before the simulation)
        self.v = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.v[i] = velocities[i]

        # setting external forces
        self.ext_forces = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.ext_forces[i][1] = masses[i] * gravity  # initialize forces with gravity

        # setting values of masses
        self.inv_mass = ti.field(ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.inv_mass[i] = 1 / masses[i]

        # Setting distance constraint (springs)
        self.generate_distance_constr(connections, d)

    # Generate the distance constraints (spring system of the soft body)
    @ti.func
    def generate_distance_constr(self, connections, d):
        self.d = d
        self.distance_constr = ti.Vector.field(2, dtype=ti.int31, shape=connections.__len__())
        for i in connections.__len__():
            self.distance_constr[i] = connections[i]

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

    # Set simulation is used to set the physical values of position, velocity, and force.
    # TODO: Qui non credo serva TI kernel
    @ti.kernel
    def set_simulation(self, num_particles, positions, velocities, masses, connections, d, id):
        if self.bodies.__contains__(id):
            print("ERROR: this body already exists")
        else:
            body = Body(id)
            body.set_simulation(num_particles, positions, velocities, masses, connections, d, self.gravity)
            self.bodies[id] = body


    @ti.kernel
    def prologue_velocities(self):
        for j in self.bodies.keys():
            body = self.bodies[j]
            for i in range(body.num_particles):
                # Update velocities using forces external
                body.v[i] = body.v[i] + body.dt * body.ext_forces[i] * body.inv_mass[i]

                # Damp velocities
                body.v[i] = self.damp(body.v[i])

    @ti.func
    def damp(self, velocity):
        return velocity

    @ti.kernel
    def prologue_positions(self):
        for j in self.bodies.keys():
            body = self.bodies[j]
            for i in range(body.num_particles):
                body.x[i] = body.p[i] + body.dt * body.v[i]

    #@ti.kernel
    #def generate_constraints(self):
        # Setting the position constraints - for collisions between particles and the ground
        '''for i in range(num_particles):
            for j in range(num_particles):
                self.position_constr[c] = [to_be_set]'''
        # The position constraints are useful if we have moving obstacles.
        # Also useful to bound our particles to the borders
        # (like a moving border) in space

    @ti.kernel
    def project_constraints(self):
        # projection of distance (collision) constraint

        # projection of distance constraint (springs)
        for j in self.bodies.keys():
            body = self.bodies[j]
            for i in range(body.num_particles):
                p1_idx = body.distance_constr[i][0]
                p2_idx = body.distance_constr[i][1]
                p1 = body.x[p1_idx]
                p2 = body.x[p2_idx]
                n = (p1 - p2).normalized()
                distance = (p1 - p2).norm() - body.d
                inv_m_total = body.inv_mass[p1_idx] - body.inv_mass[p2_idx]

                delta1 = - (body.inv_mass[p1_idx]/inv_m_total) * distance * n
                delta2 = + (body.inv_mass[p2_idx]/inv_m_total) * distance * n

                body.x[p1_idx] += delta1
                body.x[p2_idx] += delta2

    @ti.kernel
    def epilogue(self):
        for j in self.bodies.keys():
            body = self.bodies[j]
            for i in range(body.num_particles):
                body.v[i] = (body.p[i] - body.x[i]) / body.dt
                body.p[i] = body.x[i]





