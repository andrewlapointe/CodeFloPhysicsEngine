import math
import numpy as np
from physics_v2 import SMOOTHING_RADIUS

class SpacialHash():
    
    def __init__(self) -> None:
        self.size = 2
        self.spacing = SMOOTHING_RADIUS * 2
        self.num_sections = None
        self.hash_table = None
        
        self.setup()


    def setup(self):
        self.num_sections = int(np.ceil(self.size / self.spacing))
        self.spacing = self.size / self.num_sections

        #  Create a table size num_sections ** dimensions plus 1
        self.hash_table = np.zeros_like((self.num_sections ** 3) + 1)

    
    def insert_particle(self, particle):
        i_x = np.floor(particle.x / self.spacing)
        i_y = np.floor(particle.y / self.spacing)
        i_z = np.floor(particle.z / self.spacing)

        index = (i_x * self.num_sections + i_y) * self.num_sections + i_z


