import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np

def setup():
    glEnable(GL_POINT_SMOOTH)  # Enable smooth points
    glEnable(GL_BLEND)  # Enable blending for transparency
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)  # Set blend function
    glPointSize(5)  # Set point size

def create_particles(n):
    """Generate an array of 'n' particles represented as points in 3D space."""
    return np.random.rand(n, 3) * 2 - 1  # Random positions in range [-1, 1]

def draw_particles(particles):
    """Draw particles in the scene."""
    glBegin(GL_POINTS)
    for particle in particles:
        glVertex3fv(particle)
    glEnd()

def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    setup()

    particles = create_particles(100)  # Create 100 particles

    clock = pygame.time.Clock()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        draw_particles(particles)
        pygame.display.flip()
        clock.tick(60)  # Cap the frame rate to 60 FPS

if __name__ == "__main__":
    main()
