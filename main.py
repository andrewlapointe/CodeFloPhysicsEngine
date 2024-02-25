import arcade
import graphics

# CONSTANTS
SCREEN_TITLE = "CodeFlo"
SCREEN_HEIGHT = 600
SCREEN_WIDTH = 800

if __name__ == "__main__":
    game = graphics.MyGame(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
    game.setup()
    arcade.run()