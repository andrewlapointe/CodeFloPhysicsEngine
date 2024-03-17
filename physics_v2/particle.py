import numpy as np

class Particle:
    def __init__(self, mass) -> None:
        """
        position: np.array(x, y, z)
        """
        self.position = np.random.rand(1, 3) * 2 - 1
        self.velocity = np.array([0,0,0])
        self.mass = mass
        self.density = None
        self.pressure = None
        self.sum_force = None

        # print(self.position)

    def translate_particle(self, delta):
        """
        Translate a particle by dx, dy, dz units in the x, y, z directions, respectively.
        Ensure the particle does not move outside the bounding box defined by [-1, 1] in all dimensions.
        
        :param particle: The current position of the particle as a numpy array [x, y, z].
        :param dx: Distance to move in the x direction.
        :param dy: Distance to move in the y direction.
        :param dz: Distance to move in the z direction.
        :return: The new position of the particle as a numpy array [x, y, z].
        """
        
        # Calculate proposed new position
        new_position = np.add(self.position, delta)
        
        # Clamp each coordinate to ensure it stays within the bounding box [-1, 1]
        new_position_clamped = np.clip(new_position, -1, 1)
        
        # Update position
        self.position = new_position_clamped


    def distance_to(self, particle):
        # TODO
        pass

    def get_pos(self):
        return self.position