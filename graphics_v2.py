import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
from physics_v2 import engine_v2


class Graphics():

    def __init__(self) -> None:
        self.engine = engine_v2.Engine()


    def graphics_setup(self):
        glEnable(GL_POINT_SMOOTH)  # Enable smooth points
        glEnable(GL_BLEND)  # Enable blending for transparency
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)  # Set blend function
        glPointSize(5)  # Set point size


    def draw_particles(self):
        """Draw particles in the scene."""
        glBegin(GL_POINTS)
        for particle in self.engine.particle_list:
            if particle.draw:
                # if particle.move:
                # glColor3f(255,0,0)
                glVertex3fv(particle.position)
        glEnd()


    def draw_bounding_box():
        """Draw a wireframe box around the 3D space."""
        glBegin(GL_LINES)
        vertices = np.array([
            [-1, -1, -1],
            [1, -1, -1],
            [1, 1, -1],
            [-1, 1, -1],
            [-1, -1, 1],
            [1, -1, 1],
            [1, 1, 1],
            [-1, 1, 1]
        ])
        
        # Define the 12 lines (edges) of the cube
        edges = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7)
        ]
        
        for edge in edges:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()

    def event_loop(self):
        mouse_down = False
        last_pos = pygame.mouse.get_pos()

        clock = pygame.time.Clock()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
                elif event.type == MOUSEBUTTONDOWN:
                    if event.button == 1:  # Left mouse button
                        mouse_down = True
                elif event.type == MOUSEBUTTONUP:
                    if event.button == 1:  # Left mouse button
                        mouse_down = False

            if mouse_down:
                current_pos = pygame.mouse.get_pos()
                dx, dy = current_pos[0] - last_pos[0], current_pos[1] - last_pos[1]
                glRotatef(np.sqrt(dx**2 + dy**2) * 0.1, dy, dx, 0)
                last_pos = current_pos

            self.engine.update()
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            self.draw_particles()
            Graphics.draw_bounding_box()  # Draw the bounding box around the particles
            pygame.display.flip()
            clock.tick(60)  # Cap the frame rate to 60 FPS
