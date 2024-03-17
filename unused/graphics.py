import arcade
from physics import engine
import time

class MyGame(arcade.Window):
    def __init__(self, width, height, title):
        super().__init__(width, height, title)

        arcade.set_background_color(arcade.color.GRAY_BLUE)

        self.sprite_list = None
        self.engine = engine.Engine()

    def setup(self):
        """ Set up the game variables. Call to re-start the simulation. """
        # Create your sprites and sprite lists here
        self.engine.engine_setup()
        self.sprite_list = self.engine.particle_list

    def on_draw(self):
        """
        Render the screen.
        """

        # This command should happen before we start drawing. It will clear
        # the screen to the background color, and erase what we drew last frame
        self.clear()

        # Call draw() on all your sprite lists below
        for sprite in self.sprite_list:
            arcade.draw_circle_outline(sprite.get_x(), sprite.get_y(), sprite.smoothing_radius, [0,0,0], 1)
            arcade.draw_point(sprite.get_x(), sprite.get_y(), [0,0,100], 5 * sprite.scale)
        # self.draw_quad()
        

        # arcade.draw_text(f"y = {self.sprite_list[0].get_y()}", 10, 10, arcade.color.WHITE, 12, 20, "left")
        # arcade.draw_text(f"x = {self.sprite_list[0].get_x()}", 10, 25, arcade.color.WHITE, 12, 20, "left")
        arcade.draw_text(f"Near Particles = {len(self.sprite_list[0].near_particles)}", 10, 40, arcade.color.WHITE, 12, 20, "left")

    def draw_quad(self):
        for i in self.engine.quad_tree.grid_list:
            arcade.draw_lines(point_list=i, color=[0,0,0], line_width=1)


    def on_update(self, time_step):
        """
        All the logic to move, and the game logic goes here.
        Normally, you'll call update() on the sprite lists that
        need it.
        """
        for particle in self.sprite_list:
            self.engine.engine_update()
            particle.particle_update()
