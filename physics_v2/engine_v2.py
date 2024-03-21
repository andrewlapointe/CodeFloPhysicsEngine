import numpy as np
import math
from scipy.spatial import cKDTree
from physics_v2 import particle as p
from physics_v2 import spacial_hash as hash
from physics_v2 import PARTICLE_COUNT, SMOOTHING_RADIUS, GRAVITY, DESIRED_DENSITY, particle
class Engine:
    def __init__(self) -> None:
        self.hash = None

        size = np.arange(PARTICLE_COUNT * 3).reshape(PARTICLE_COUNT, 3)
        self.particle_position_array = None
        self.particle_list = []
        self.engine_setup()
        self.time_step = self.set_time_step()

    def engine_setup(self):

        #  Create Bounding Box
        self.bounding_box(20)


        #  Create Particles
        for i in range(PARTICLE_COUNT):
            particle = p.Particle(mass=(math.pow(SMOOTHING_RADIUS, 3)*(DESIRED_DENSITY)))
            self.particle_list.append(particle)
        
        print(len(self.particle_list))
        self.update()


    def update(self):
        self.particle_position_array = np.array([p.position for p in self.particle_list])
        self.hash = cKDTree(self.particle_position_array)
    
        self.set_time_step()
        self.navier_stokes()


    def set_time_step(self):
        max_velocity = 1
        for i in self.particle_list:
            if np.linalg.norm(i.velocity) > max_velocity:
                max_velocity = np.linalg.norm(i.velocity)
        self.time_step = 0.4 * (SMOOTHING_RADIUS / max_velocity)


    def bounding_box(self, edge_points):
        """
        Create points on the surface of a cube with corners at (-1, -1, -1) and (1, 1, 1).

        Parameters:
        edge_points (int): Number of points along each edge of the cube.

        Returns:
        np.array: Array of points on the surface of the cube.
        """
        # Ensure that there are at least two points on each edge (the corners)
        edge_points = max(edge_points, 2)
        
        # Linearly spaced points along each edge
        edge = np.linspace(-1, 1, edge_points)
        
        # Points on each face of the cube
        points = []
        
        # Generate points on the top and bottom faces
        for z in [-1,1]:
            for x in edge:
                for y in edge:
                    points.append([x, y, z])
                    
        # Generate points on the front and back faces
        for x in [-1, 1]:
            for y in edge[1:-1]:  # Exclude the first and last to avoid duplicating corners
                for z in edge[1:-1]:  # Exclude the first and last to avoid duplicating corners
                    points.append([x, y, z])
                    
        # Generate points on the left and right faces
        for y in [-1, 1]:
            for x in edge[1:-1]:  # Exclude the first and last to avoid duplicating corners
                for z in edge[1:-1]:  # Exclude the first and last to avoid duplicating corners
                    points.append([x, y, z])

        for point in points:
            particle = p.Particle(mass=(math.pow(SMOOTHING_RADIUS, 3)*(DESIRED_DENSITY)), x=point[0], y=point[1], z=point[2], draw=False, move=False)
            self.particle_list.append(particle)

    def navier_stokes(self):
        """
        Lagrange form of Navier-Stokes
        """
        # TODO
        # Find all proximal particles
        for i in self.particle_list:
            indices = self.hash.query_ball_point(i.position, SMOOTHING_RADIUS)
            # print(indices)
            i.near_particles = [self.particle_list[j] for j in indices]

        for i in self.particle_list:
            i.density = self.calc_density(i)
            # print(i.density)
            i.pressure = self.calc_pressure(i)
            # print(i.pressure)

        for i in self.particle_list:
            pressure = self.calc_pressure_grad(i)
            viscosity = self.calc_viscosity_grad(i)
            other_force = self.calc_other_force(i)

            i.sum_force = pressure + viscosity + other_force
            # print(f"Force: {i.sum_force}")

        for i in self.particle_list:
            i.velocity = i.velocity + (self.time_step * (i.sum_force / i.mass))
            i.translate_particle(np.multiply(i.velocity, self.time_step))
            # print(i.position)


    def calc_pressure_grad(self, i):
        grad_pressure_x = 0
        grad_pressure_y = 0
        grad_pressure_z = 0

        for j in i.near_particles:

            # Calculate the distance vector and its magnitude
            distance_vector = i.position - j.position
            distance = np.linalg.norm(distance_vector)
            
            # Calculate normalized direction vector
            direction_vector = np.divide(distance_vector, distance) if distance != 0 else np.zeros_like(distance_vector)

            # Calculate the gradient magnitude for this pair
            grad_magnitude = Engine.influence_grad(distance) * j.mass * ((i.pressure / i.density**2) + (j.pressure / j.density**2))
            
            # Accumulate the gradient components
            grad_pressure_x += grad_magnitude * direction_vector[0]
            grad_pressure_y += grad_magnitude * direction_vector[1]
            grad_pressure_z += grad_magnitude * direction_vector[2]

        # Scale by -i.mass/i.density
        scale = -(i.mass / i.density)
        grad_pressure_x *= scale
        grad_pressure_y *= scale
        grad_pressure_z *= scale

        return np.array([grad_pressure_x, grad_pressure_y, grad_pressure_z])


    def calc_viscosity_grad(self, i):
        return np.array([0,0,0])


    def calc_other_force(self, i):
        x_force = 0
        # if i.position[1] < -0.85:
        #     x_force = i.mass * 90
        return np.array([x_force,i.mass * GRAVITY,0])
        # return np.array([0, 0, 0])

        
    def calc_density(self, i):
        density = 0
        for j in i.near_particles:
            density += j.mass * Engine.kernal(np.linalg.norm(i.position - j.position))
        return density + 100

    
    def calc_pressure(self, particle):
        k = 0.001  # STIFFNESS CONSTANT TODO
        pressure = k * (math.pow((particle.density/DESIRED_DENSITY), 7) - 1)
        return pressure
    
    def interpolation(self, i, attribute_name):
        attribute_value = 0

        for j in self.particle_list:
            (j.mass / j.density) * getattr(j, attribute_name) * Engine.kernal(np.linalg.norm(i.position - j.position))

        setattr(i, attribute_name, attribute_value)

    def kernal(distance:float) -> float:
        """
        Refered to as "Wᵢⱼ" by Ihmesen et al. 

        Arguments:
            distance -- Scalar distance between two points

        Returns:
            Influence value based on distance between particles
        """

        if distance < 0:
            return ValueError
        elif distance >= 0 and distance < 1:
            influence = (3/(2*math.pi)) * ((2/3) - (distance ** 2) + (math.pow(distance, 3)/2))
        elif distance >= 1 and distance < 2:
            influence = (3/(2*math.pi)) * ((1/6) * math.pow((2-distance), 3))
        else:
            return 0
        
        return (1 / math.pow(SMOOTHING_RADIUS, 3)) * influence
    
    
    def influence_grad(distance: float) -> float:
        """
        Computes the gradient of the influence value based on the distance between particles.

        Arguments:
            distance -- Scalar distance between two points.
            SMOOTHING_RADIUS -- The smoothing radius used in the computation.

        Returns:
            The gradient of the influence value for the given distance.
        """

        #  TODO Double check this function for mathematical errors
        
        if distance < 0:
            raise ValueError("Distance cannot be negative.")
        elif 0 <= distance < 1:
            grad = (3 * (1.5 * distance**2 - 2 * distance)) / (2 * math.pi * SMOOTHING_RADIUS**3)
        elif 1 <= distance < 2:
            grad = (-0.75 * (2 - distance)**2) / (math.pi * SMOOTHING_RADIUS**3)
        else:  # distance >= 2
            grad = 0

        return grad