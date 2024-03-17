import numpy as np
import random, math, timeit
from physics import PARTICLE_COUNT, particles, quad
from main import SCREEN_WIDTH, SCREEN_HEIGHT

class Engine():

    distance_lambda = lambda p1, p2: math.sqrt(math.pow(p2[0] - p1[0], 2) + math.pow(p2[1] - p1[1], 2))

    def __init__(self) -> None:
        self.particle_list = []
        # self.x_positions = np.array([])
        # self.y_positions = np.array([])
        self.domain = quad.Rect(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, SCREEN_WIDTH, SCREEN_HEIGHT)
        self.quad_tree = quad.QuadTree(self.domain)


    def engine_setup(self):
        self.add_particle(400, 300, 1, 2)
        for _ in range(PARTICLE_COUNT):
            x = random.randint(200, SCREEN_WIDTH-200)
            y = random.randint(100, SCREEN_HEIGHT-100)
            self.add_particle(x, y, 1)

        self.engine_update()

    
    def engine_update(self):
        self.generate_quad_tree()
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
        value = max([0,math.pow(math.pow(radius, 2) - math.pow(distance, 2), 3)])
        return (math.pow(value, 3)/volume)
    
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
            for near_particle in particle.near_particles:
                distance = particle.distance_to(near_particle)
                if distance < particle.smoothing_radius:
                    influence = Engine.smoothing_influence(particle.smoothing_radius, distance)
                    # print(influence)
                    density += near_particle.mass * influence

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
    
    def calc_pressure_force_delta(self, particle, root_particle, x:bool):
        pressure = 0
        for near_particle in root_particle.near_particles:
            distance = particle.distance_to(near_particle)
            force = Engine.smoothing_derivative(particle.smoothing_radius, distance)
            edge_force_x = 0
            edge_force_y = 0
            if x:
                if particle.position[0] <= particle.smoothing_radius:
                    edge_force_x = -(Engine.smoothing_derivative(particle.smoothing_radius, particle.position[0]))
                if particle.position[0] >= SCREEN_WIDTH-particle.smoothing_radius:
                    edge_force_x = (Engine.smoothing_derivative(particle.smoothing_radius, SCREEN_WIDTH-particle.position[0]))
            else:
                if particle.position[1] <= particle.smoothing_radius:
                    edge_force_y = -(Engine.smoothing_derivative(particle.smoothing_radius, particle.position[1]))
                if particle.position[1] >= SCREEN_HEIGHT-particle.smoothing_radius:
                    edge_force_y = (Engine.smoothing_derivative(particle.smoothing_radius, SCREEN_HEIGHT-particle.position[1]))
            pressure += (force + edge_force_x + edge_force_y)  # Accumulate forces exerted by other particles
        # angle_rad = math.atan(particle.position[1]/particle.position[0])
        return pressure #, angle_rad


    def calc_pressure_gradient(self):
        gradient_x = []
        gradient_y = []
        h = 1e-5  # Step size for numerical differentiation

        for particle in self.particle_list:
            perturbed_position_x_forward = [particle.position[0] + h, particle.position[1]]
            perturbed_position_x_backward = [particle.position[0] - h, particle.position[1]]
            perturbed_particle_x_forward = particles.Particle(perturbed_position_x_forward[0], perturbed_position_x_forward[1])
            perturbed_particle_x_backward = particles.Particle(perturbed_position_x_backward[0], perturbed_position_x_backward[1])
            pressure_perturbed_x_forward = self.calc_pressure_force_delta(perturbed_particle_x_forward, particle, True)
            pressure_perturbed_x_backward = self.calc_pressure_force_delta(perturbed_particle_x_backward, particle, True)

            # Compute pressure gradient component in x-direction using central difference
            pressure_gradient_component_x = (pressure_perturbed_x_forward - pressure_perturbed_x_backward) / (2 * h)
            gradient_x.append(pressure_gradient_component_x)

            # Perturb the position of the particle along y-axis positively and negatively
            perturbed_position_y_forward = [particle.position[0], particle.position[1] + h]
            perturbed_position_y_backward = [particle.position[0], particle.position[1] - h]
            perturbed_particle_y_forward = particles.Particle(perturbed_position_y_forward[0], perturbed_position_y_forward[1])
            perturbed_particle_y_backward = particles.Particle(perturbed_position_y_backward[0], perturbed_position_y_backward[1])
            pressure_perturbed_y_forward = self.calc_pressure_force_delta(perturbed_particle_y_forward, particle, False)
            pressure_perturbed_y_backward = self.calc_pressure_force_delta(perturbed_particle_y_backward, particle, False)

            # Compute pressure gradient component in y-direction using central difference
            pressure_gradient_component_y = (pressure_perturbed_y_forward - pressure_perturbed_y_backward) / (2 * h)
            gradient_y.append(pressure_gradient_component_y)

        return gradient_x, gradient_y


    def generate_quad_tree(self):
        self.quad_tree = quad.QuadTree(self.domain)
        for i in self.particle_list:
            self.quad_tree.insert(i)
        for i in self.particle_list:    
            i.near_particles = self.quad_tree.query_radius(i.position, i.smoothing_radius, [])


    def navier_stokes(self):
        #acceleration * density = -(d pressure_x/dx, d pressure_y/dy) + mew laplace velocity + external force
        #acceleration = (-(d pressure_x/dx, d pressure_y/dy) + mew laplace velocity + external force) / density
    
        pressure_grad_x, pressure_grad_y = self.calc_pressure_gradient()
        self.calc_density()
        viscosity = 0
        force_x = 0  # this will normally be 0
        force_y = 0  # replace 0 with acceleration value to add force due to gravity

        for i, particle in enumerate(self.particle_list):
            if particle.density > 0:
                particle.accel_x = 1000 * (-(pressure_grad_x[i] + viscosity + (force_x * particle.density)) / particle.density)
            # print(particle.density)

        for i, particle in enumerate(self.particle_list):
            if particle.density > 0:
                particle.accel_y = 100 * (-(pressure_grad_y[i] + viscosity + (force_y * particle.density)) / particle.density)


    def remove_particle(self, particle_obj):
        self.particle_list.remove(particle_obj)


    def add_particle(self, part_x, part_y, mass, scale=1):
        self.particle_list.append(particles.Particle(part_x, part_y, mass, scale))