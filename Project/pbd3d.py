import taichi as ti


@ti.data_oriented
class PBDSolver(object):
    # Initialize function is used to set the constants
    def __init__(self):
        self.dt = 3e-3
        self.t = 0.0
        self.paused = True
        self.gravity = ti.Vector([0, -9.8, 0])
        self.iterations = 20
        self.n_constraints = 0

    # Set simulation is used to set the physical values of position, velocity, and force.
    @ti.kernel
    def set_simulation(self, num_particles, positions, velocities, masses):

        self.num_particles = num_particles

        # setting initial position of particles (before the simulation)
        self.p = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.p[i] = positions[i]  # particle_positions is a python matrix
        self.x = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)

        # setting initial velocity of particles (before the simulation)
        self.v = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.v[i] = velocities[i]

        self.ext_forces = ti.Vector.field(3, dtype=ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.ext_forces[i][1] = masses[i] * self.gravity[1]  # initialize forces with gravity

        self.inv_mass = ti.field(ti.f32, shape=num_particles)
        for i in range(num_particles):
            self.inv_mass[i] = 1 / masses[i]

        # A distance constraint for each pair of particles
        # A position constraint for each particle
        self.distance_constr = ti.Vector.field(2, dtype=ti.int31, shape=num_particles)
        self.position_constr = ti.Vector.field(2, dtype=ti.int31, shape=num_particles)

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
    def generate_constraints(self):
        c = 0
        # Setting the distance constraints - for collision between particles
        for i in range(self.num_particles):
            for j in range(self.num_particles):
                self.distance_constr[c] = [i,j]

        # Setting the position constraints - for collisions between particles and the ground
        '''for i in range(num_particles):
            for j in range(num_particles):
                self.position_constr[c] = [to_be_set]'''
        # The position constraints are useful if we have moving obstacles
        # (like a moving border) in space

    '''@ti.kernel
    def project_constraints(self):
        for i in range(self.num_particles):'''


