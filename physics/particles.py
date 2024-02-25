import arcade
import numpy as np
from . import TIME_STEP, SMOOTHING_RADIUS

class Particle(arcade.Sprite):
    """
    Particle Class
    """


    def __init__(self, center_x:int, center_y:int, mass:int = 1) -> None:
        super().__init__("images/blue_particle.png", center_x=center_x, center_y=center_y)

        # x and y coordinates or particle
        self.center_x = center_x
        self.center_y = center_y
        self.position = [self.center_x, self.center_y]
        self.mass = mass
        self.velocity_x = 0
        self.velocity_y = 0
        self.accel_x = 0
        self.accel_y = 0
        self.density = None
        self.pressure_x = None
        self.pressure_y = None
        # self.external_accel_x = 0
        # self.external_accel_y = -9.8
        self.smoothing_radius = SMOOTHING_RADIUS


    def particle_update(self):
        self.calc_displacement()



    def calc_displacement(self):
        """
        Kinematics equation to solve for displacement.
        """
        if self.center_y > 5 and self.center_x > 5 and self.center_x < 795 and self.center_y < 595:
            self.center_x = self.center_x + ((self.velocity_x * TIME_STEP) + (0.5 * self.accel_x * TIME_STEP))
            self.center_y = self.center_y + ((self.velocity_y * TIME_STEP) + (0.5 * self.accel_y * TIME_STEP))
            self.velocity_x += (self.accel_x * TIME_STEP)
            self.velocity_y += (self.accel_y * TIME_STEP)

        else:
            pass



    def get_x_pos(self):
        return self.center_x
    

    def get_y_pos(self):
        return self.center_y