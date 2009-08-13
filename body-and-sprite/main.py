from __future__ import division

from Box2D import *
import math
import pyglet
from pyglet.gl import *
import rabbyt
import sys

def create_world(lower_bound=(-100, -100), upper_bound=(100, 100),
                 gravity=(0, -10), do_sleep=True):
    aabb = b2AABB()
    aabb.lowerBound = lower_bound
    aabb.upperBound = upper_bound
    return b2World(aabb, gravity, do_sleep)

def create_circle_body(world, position=(0, 0), linear_velocity=(0, 0),
                       angular_velocity=0, radius=1, density=None,
                       friction=None, restitution=None, user_data=None):
    body_def = b2BodyDef()
    body_def.position = position
    body = world.CreateBody(body_def)
    body.linearVelocity = linear_velocity
    body.angularVelocity = angular_velocity
    body.userData = user_data
    shape_def = b2CircleDef()
    shape_def.radius = radius
    if density is not None:
        shape_def.density = density
    if friction is not None:
        shape_def.friction = friction
    if restitution is not None:
        shape_def.restitution = restitution
    body.CreateShape(shape_def)
    if density is not None:
        body.SetMassFromShapes()
    return body

def create_box_body(world, position=(0, 0), linear_velocity=(0, 0),
                    angular_velocity=0, half_width=1, half_height=1, angle=0,
                    density=None, friction=None, restitution=None,
                    user_data=None):
    body_def = b2BodyDef()
    body_def.position = position
    body = world.CreateBody(body_def)
    body.linearVelocity = linear_velocity
    body.angularVelocity = angular_velocity
    body.userData = user_data
    shape_def = b2PolygonDef()
    shape_def.SetAsBox(half_width, half_height, position, angle)
    if density is not None:
        shape_def.density = density
    if friction is not None:
        shape_def.friction = friction
    if restitution is not None:
        shape_def.restitution = restitution
    body.CreateShape(shape_def)
    if density is not None:
        body.SetMassFromShapes()
    return body

def rad_to_deg(angle):
    return angle * 180. / math.pi
 
def connect_sprite_to_body(sprite, body):
    sprite.x = lambda: body.position.x
    sprite.y = lambda: body.position.y
    sprite.rot = lambda: rad_to_deg(body.angle)

def save_screenshot(name='screenshot.png', format='RGB'):
    image = pyglet.image.get_buffer_manager().get_color_buffer().image_data
    image.format = format
    image.save(name)

def disconnect_sprite_from_body(sprite, body):
    end_x = body.position.x + body.linearVelocity.x
    end_y = body.position.y + body.linearVelocity.y
    end_rot = rad_to_deg(body.angle + body.angularVelocity)
    sprite.x = rabbyt.lerp(end=end_x, dt=1., extend='extrapolate')
    sprite.y = rabbyt.lerp(end=end_y, dt=1., extend='extrapolate')
    sprite.rot = rabbyt.lerp(end=end_rot, dt=1., extend='extrapolate')

class MyWindow(pyglet.window.Window):
    def __init__(self, **kwargs):
        super(MyWindow, self).__init__(**kwargs)
        self.set_exclusive_mouse(self.fullscreen)
        self.set_exclusive_keyboard(self.fullscreen)

        glClearColor(0, 0.5, 0, 0)
        rabbyt.set_default_attribs()
        self.scale = 10

        self.time = 0
        self.world = create_world()
        self.ground_body = create_box_body(self.world, position=(0, -2),
                                           half_width=4)

        eightball_texture = pyglet.resource.texture('eightball.png')
        eightball_scale = 2 / (eightball_texture.width +
                               eightball_texture.height)
        eightball_sprite = rabbyt.Sprite(eightball_texture,
                                         scale=eightball_scale)
        eightball_body = create_circle_body(self.world, position=(-3, 2),
                                            angular_velocity=-math.pi,
                                            density=1, restitution=0.9)
        connect_sprite_to_body(eightball_sprite, eightball_body)
        self.sprites = [eightball_sprite]

    def on_draw(self):
        rabbyt.set_time(self.time)
        self.clear()
        glPushMatrix()
        glTranslatef(self.width // 2, self.height // 2, 0)
        scale = min(self.width, self.height) / self.scale
        glScalef(scale, scale, scale)
        rabbyt.render_unsorted(self.sprites)
        glPopMatrix()

    def step(self, dt):
        self.time += dt
        self.world.Step(dt, 10, 10)

    def on_key_press(self, symbol, modifiers):
        if symbol == pyglet.window.key.ESCAPE:
            self.on_close()
        if symbol == pyglet.window.key.F12:
            save_screenshot()

def main():
    fullscreen = '--fullscreen' in sys.argv
    window = MyWindow(fullscreen=fullscreen)
    pyglet.clock.schedule_interval(window.step, 1 / 60)
    pyglet.app.run()

if __name__ == '__main__':
    main()
