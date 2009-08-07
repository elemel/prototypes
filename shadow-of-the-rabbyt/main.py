import pyglet
from pyglet.gl import *
import rabbyt
import sys

def create_shadow(sprite, texture, x=0, y=0, rot=0, red=0, green=0, blue=0,
                  alpha=1):
    shadow = rabbyt.Sprite(texture, scale=sprite.scale)
    shadow.rgba = red, green, blue, alpha
    shadow.x = sprite.attrgetter('x') + x
    shadow.y = sprite.attrgetter('y') + y
    shadow.rot = sprite.attrgetter('rot') + rot
    return shadow

def load_tileable_texture(name):
    image = pyglet.resource.texture(name)
    return pyglet.image.TileableTexture.create_for_image(image)

class MyWindow(pyglet.window.Window):
    def __init__(self, **kwargs):
        super(MyWindow, self).__init__(**kwargs)
        rabbyt.set_default_attribs()
        glClearColor(1, 1, 1, 1)
        self.background = load_tileable_texture('background.png')
        self.ship_texture = pyglet.resource.texture('ship.png')
        self.ship = rabbyt.Sprite(self.ship_texture,
                                  rot=rabbyt.lerp(end=60, dt=1,
                                                  extend='extrapolate'))
        self.shadow = create_shadow(self.ship, self.ship_texture, x=20, y=-30,
                                    alpha=0.5)
        self.time = 0.

    def step(self, dt):
        self.time += dt
        rabbyt.set_time(self.time)

    def on_draw(self):
        self.clear()
        self.background.blit_tiled(0, 0, 0, self.width, self.height)
        glPushMatrix()
        glTranslatef(self.width // 2, self.height // 2, 0)
        self.shadow.render()
        self.ship.render()
        glPopMatrix()

def main():
    window = MyWindow(fullscreen=('--fullscreen' in sys.argv))
    pyglet.clock.schedule_interval(window.step, 1. / 60.)
    pyglet.app.run()

if __name__ == '__main__':
    main()
