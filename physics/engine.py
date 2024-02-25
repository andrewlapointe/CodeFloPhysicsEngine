import numpy as np
import random, math
from physics import PARTICLE_COUNT, particles

class Engine():

    distance_lambda = lambda p1, p2: math.sqrt(math.pow(p2[0] - p1[0], 2) + math.pow(p2[1] - p1[1], 2))

    def __init__(self) -> None:
        self.particle_list = []
        self.x_positions = np.array([])
        self.y_positions = np.array([])

    def engine_setup(self):
        self.add_particle(300, 400, 1)
        for _ in range(PARTICLE_COUNT):
            x = random.randint(50,750)
            y = random.randint(50, 550)
            self.add_particle(x, y, 1)

    
    def engine_update(self):
        self.navier_stokes()

    def smoothing_influence(radius, distance):
        """
        _summary_

        Arguments:
            radius: float -- smoothing radius of particle
            distance: float -- distance to between particles

        Returns:
            influence: float -- degree of influence between particles
        """
        volume = math.pi * math.pow(radius, 8) / 4  # see volume integration in images
        value = (math.pow(radius, 2) - math.pow(distance, 2))
        return math.pow(value, 3) / volume
    
    def smoothing_derivative(radius: int, distance: float):
        """
        Slope of smoothing function

        Arguments:
            radius: (int) -- smoothing radius
            distance: (float) -- distance between particles

        Returns:
            slope: (float)
        """
        return -(math.pow((24 * distance)*(math.pow(radius, 2) - math.pow(distance, 2)), 2)) / (math.pi * math.pow(radius, 8))

    def calc_density(self):
        # TODO Adjust values (mass, radius, influence function, etc.)
        for particle in self.particle_list:
            density = 0
            for other_particle in self.particle_list:
                if other_particle != particle:
                    distance = Engine.distance_lambda(particle.position, other_particle.position)
                    influence = Engine.smoothing_influence(particle.smoothing_radius, distance)
                    density +=  particle.mass * influence

            particle.density = density


    def calc_pressure_force(self, particle):
        pressure = 0
        for other_particle in self.particle_list:
            if other_particle != particle:
                distance = Engine.distance_lambda(particle.position, other_particle.position)
                force = Engine.smoothing_influence(particle.smoothing_radius, distance)
                pressure += force  # Accumulate forces exerted by other particles
        angle_rad = math.atan(particle.position[1]/particle.position[0])
        return pressure #, angle_rad
    
    def calc_pressure_force_2(self, particle):
        pressure = 0
        for other_particle in self.particle_list:
            if other_particle != particle:
                distance = Engine.distance_lambda(particle.position, other_particle.position)
                force = Engine.smoothing_derivative(particle.smoothing_radius, distance)
                pressure += force  # Accumulate forces exerted by other particles
        angle_rad = math.atan(particle.position[1]/particle.position[0])
        return pressure #, angle_rad


    def calc_pressure_gradient(self):
        gradient_x = []
        gradient_y = []
        h = 1e-5  # Step size for numerical differentiation

        for particle in self.particle_list:
            # Compute pressure at original position
            pressure_original = self.calc_pressure_force_2(particle)

            # Perturb the position of the particle along x-axis
            perturbed_position_x = [particle.position[0] + h, particle.position[1]]
            perturbed_particle_x = particles.Particle(perturbed_position_x[0], perturbed_position_x[1])
            pressure_perturbed_x = self.calc_pressure_force_2(perturbed_particle_x)

            # Compute pressure gradient component in x-direction
            pressure_gradient_component_x = (pressure_perturbed_x - pressure_original) / h
            gradient_x.append(pressure_gradient_component_x)

            # Perturb the position of the particle along y-axis
            perturbed_position_y = [particle.position[0], particle.position[1] + h]
            perturbed_particle_y = particles.Particle(perturbed_position_y[0], perturbed_position_y[1])
            pressure_perturbed_y = self.calc_pressure_force_2(perturbed_particle_y)

            # Compute pressure gradient component in y-direction
            pressure_gradient_component_y = (pressure_perturbed_y - pressure_original) / h
            gradient_y.append(pressure_gradient_component_y)

        return gradient_x, gradient_y
    

    def navier_stokes(self):
        #acceleration * density = -(d pressure_x/dx, d pressure_y/dy) + mew laplace velocity + external force
        #acceleration = (-(d pressure_x/dx, d pressure_y/dy) + mew laplace velocity + external force) / density
    
        pressure_grad_x, pressure_grad_y = self.calc_pressure_gradient()
        viscosity = 0
        force_x = 0
        force_y = 0
        self.calc_density()

        for i, particle in enumerate(self.particle_list):
            particle.accel_x = -(pressure_grad_x[i] + viscosity + force_x) / particle.density

        for i, particle in enumerate(self.particle_list):
            particle.accel_y = -(pressure_grad_y[i] + viscosity + force_y) / particle.density


    def remove_particle(self, particle_obj):
        self.particle_list.remove(particle_obj)


    def add_particle(self, part_x, part_y, mass):
        self.particle_list.append(particles.Particle(part_x, part_y, mass))