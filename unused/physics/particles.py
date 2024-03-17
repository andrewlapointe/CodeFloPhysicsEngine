import arcade, random
import numpy as np
from . import TIME_STEP, SMOOTHING_RADIUS

class Particle(arcade.Sprite):
    """
    Particle Class
    """


    def __init__(self, center_x:int, center_y:int, mass:int=1, scale:float=1) -> None:
        super().__init__(center_x=center_x, center_y=center_y, scale=1)

        # x and y coordinates or particle
        qt_node = None
        self.near_particles = []
        self.near_particle_distances = []
        self.position = [self.center_x, self.center_y]
        self.scale = scale
        self.mass = mass
        self.velocity_x = random.randint(-2, 2)
        self.velocity_y = random.randint(-2, 2)
        self.accel_x = 0
        self.accel_y = 0
        self.density = None
        self.pressure_x = None
        self.pressure_y = None
        # self.external_accel_x = 0
        # self.external_accel_y = -9.8
        self.smoothing_radius = SMOOTHING_RADIUS

        self.x = self.get_x()
        self.y = self.get_y()


    def particle_update(self):
        self.calc_displacement()


    def distance_to(self, other):
        try:
            other_x, other_y = other.get_x(), other.get_y()
        except AttributeError:
            other_x, other_y = other
        return np.hypot(self.x - other_x, self.y - other_y)


    def calc_displacement(self):
        """
        Kinematics equation to solve for displacement.
        """
        if self.center_y < 5 or self.center_y > 775:
            self.velocity_y *= -1
        if self.center_x < 0 or self.center_x > 600:
            self.velocity_x *= -1

        self.center_x = self.center_x + ((self.velocity_x * TIME_STEP) + (0.5 * self.accel_x * TIME_STEP))
        self.center_y = self.center_y + ((self.velocity_y * TIME_STEP) + (0.5 * self.accel_y * TIME_STEP))
        self.velocity_x += (self.accel_x * TIME_STEP)
        self.velocity_y += (self.accel_y * TIME_STEP)



    def get_x(self):
        return self.position[0]
    

    def get_y(self):
        return self.position[1]