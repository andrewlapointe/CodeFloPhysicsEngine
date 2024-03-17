import numpy as np
import math
from physics_v2 import particle as p
from physics_v2 import PARTICLE_COUNT, SMOOTHING_RADIUS, GRAVITY, DESIRED_DENSITY, particle
class Engine:
    def __init__(self) -> None:
        size = np.arange(PARTICLE_COUNT * 3).reshape(PARTICLE_COUNT, 3)
        self.particle_position_array = np.zeros_like(size, dtype=float)
        self.particle_list = []
        self.engine_setup()
        self.time_step = self.set_time_step()

    def engine_setup(self):
        for i in range(PARTICLE_COUNT):
            particle = p.Particle(mass=(math.pow(SMOOTHING_RADIUS, 3)*(DESIRED_DENSITY)))
            self.particle_list.append(particle)
            self.particle_position_array[i, :] = particle.get_pos()
        
        self.update()

    def update(self):
        self.set_time_step()
        self.navier_stokes()

    def set_time_step(self):
        max_velocity = 1
        for i in self.particle_list:
            if np.linalg.norm(i.velocity) > max_velocity:
                max_velocity = np.linalg.norm(i.velocity)
        self.time_step = 0.4 * (SMOOTHING_RADIUS / max_velocity)


    def navier_stokes(self):
        """
        Lagrange form of Navier-Stokes
        """
        # TODO
        # Find all proximal particles
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

        for j in self.particle_list:
            if i != j:
                # Calculate the distance vector and its magnitude
                distance_vector = i.position - j.position
                distance = np.linalg.norm(distance_vector)
                
                # Calculate normalized direction vector
                direction_vector = np.divide(distance_vector, distance) if distance != 0 else np.zeros_like(distance_vector)

                # Calculate the gradient magnitude for this pair
                grad_magnitude = Engine.influence_grad(distance) * j.mass * ((i.pressure / i.density**2) + (j.pressure / j.density**2))
                
                # Accumulate the gradient components
                grad_pressure_x += grad_magnitude * direction_vector[0, 0]
                grad_pressure_y += grad_magnitude * direction_vector[0, 1]
                grad_pressure_z += grad_magnitude * direction_vector[0, 2]

        # Scale by -i.mass/i.density
        scale = -(i.mass / i.density)
        grad_pressure_x *= scale
        grad_pressure_y *= scale
        grad_pressure_z *= scale

        return np.array([grad_pressure_x, grad_pressure_y, grad_pressure_z])


    def calc_viscosity_grad(self, i):
        return np.array([0,0,0])


    def calc_other_force(self, i):
        # return np.array([0,i.mass * GRAVITY,0])
        return np.array([0, 0, 0])

        
    def calc_density(self, i):
        density = 0
        for j in self.particle_list:
            if i == j:
                continue
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