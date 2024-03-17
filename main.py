from graphics_v2 import *
from physics_v2 import *


def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    graphics_handler = Graphics()
    graphics_handler.graphics_setup()
    graphics_handler.event_loop()

  
if __name__ == "__main__":
    main()
