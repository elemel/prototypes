from Box2D import *
import pyglet
from pyglet.gl import *

from itertools import chain
from math import *
import sys

def equals(x, y, epsilon=1e-9):
    return abs(x - y) < epsilon

def normalize_angle(angle):
    if angle < 0.:
        while angle < 0.:
            angle += 2 * pi
    else:
        while angle >= 2 * pi:
            angle -= 2 * pi
    return angle

def interpolate_angle(angle_1, angle_2, weight_2=0.5):
    angle_1 = normalize_angle(angle_1)
    angle_2 = normalize_angle(angle_2)
    if angle_1 - angle_2 < -pi:
        angle_2 -= 2 * pi
    elif angle_1 - angle_2 >= pi:
        angle_2 += 2 * pi
    angle = angle_1 * (1. - weight_2) + angle_2 * weight_2
    return normalize_angle(angle)

def sign(x):
    return x / abs(x) if x else 0.

def init_shape_def(shape_def, **kwargs):
    if 'density' in kwargs:
        shape_def.density = kwargs.pop('density')
    if 'friction' in kwargs:
        shape_def.friction = kwargs.pop('friction')
    if 'group_index' in kwargs:
        shape_def.filter.groupIndex = kwargs.pop('group_index')
    assert not kwargs

def create_circle_def(radius, center, **kwargs):
    circle_def = b2CircleDef()
    init_shape_def(circle_def, **kwargs)
    circle_def.localPosition = center
    circle_def.radius = radius
    return circle_def

def create_box_def(half_width, half_height, center, angle, **kwargs):
    polygon_def = b2PolygonDef()
    init_shape_def(polygon_def, **kwargs)
    polygon_def.SetAsBox(half_width, half_height, center, angle)
    return polygon_def

def create_body(world, shape_defs):
    body_def = b2BodyDef()
    body = world.CreateBody(body_def)
    for shape_def in shape_defs:
        body.CreateShape(shape_def)
    body.SetMassFromShapes()
    return body

def create_circle_vertex_list(x=0., y=0., radius=1., vertex_count=100):
    coords = []
    for i in xrange(vertex_count):
        angle = 2. * pi * float(i) / float(vertex_count)
        coords.append(x + radius * cos(angle))
        coords.append(y + radius * sin(angle))
        if i:
            coords.extend(coords[-2:])
    coords.extend(coords[:2])
    return pyglet.graphics.vertex_list(len(coords) // 2, ('v2f', coords))

class Ragdoll(object):
    def __init__(self, world):
        self.init_bodies(world)
        self.init_joints(world)

    def init_bodies(self, world):
        self.bodies = {}
        self.init_capsule_body(world, 'torso', 0.2, 0.4, (0., 0.))
        self.init_circle_body(world, 'head', 0.4, (1., 0.))
        self.init_box_body(world, 'left-upper-arm', 0.2, 0.1, (0.5, 0.3))
        self.init_box_body(world, 'right-upper-arm', 0.2, 0.1, (0.5, -0.3))
        self.init_box_body(world, 'left-lower-arm', 0.2, 0.1, (0.9, 0.3))
        self.init_box_body(world, 'right-lower-arm', 0.2, 0.1, (0.9, -0.3))
        self.init_box_body(world, 'left-upper-leg', 0.2, 0.1, (-0.2, 0.2))
        self.init_box_body(world, 'right-upper-leg', 0.2, 0.1,
                               (-0.2, -0.2))
        self.init_box_body(world, 'left-lower-leg', 0.2, 0.1, (0.2, 0.2))
        self.init_box_body(world, 'right-lower-leg', 0.2, 0.1, (0.2, -0.2))

    def init_joints(self, world):
        self.joints = {}
        self.init_joint(world, 'neck', 'torso', 'head', (0.6, 0.))
        self.init_joint(world, 'left-shoulder', 'torso', 'left-upper-arm',
                        (0.3, 0.3))
        self.init_joint(world, 'right-shoulder', 'torso', 'right-upper-arm',
                        (0.3, -0.3))
        self.init_joint(world, 'left-elbow', 'left-upper-arm',
                        'left-lower-arm', (0.7, 0.3))
        self.init_joint(world, 'right-elbow', 'right-upper-arm',
                        'right-lower-arm', (0.7, -0.3))
        self.init_joint(world, 'left-hip', 'torso', 'left-upper-leg',
                        (-0.4, 0.2))
        self.init_joint(world, 'right-hip', 'torso', 'right-upper-leg',
                        (-0.4, -0.2))
        self.init_joint(world, 'left-knee', 'left-upper-leg', 'left-lower-leg',
                        (0., 0.2))
        self.init_joint(world, 'right-knee', 'right-upper-leg',
                        'right-lower-leg', (0., -0.2))

        for joint_name in ('left-knee', 'right-knee'):
            joint = self.joints[joint_name]
            joint.EnableLimit(True)
            joint.SetLimits(-pi, 0.)

    def init_circle_body(self, world, name, radius, center):
        shape_def = create_circle_def(radius, center, group_index=-1,
                                      density=1., friction=0.)
        self.bodies[name] = create_body(world, [shape_def])

    def init_box_body(self, world, name, half_width, half_height, center,
                      angle=0.):
        shape_def = create_box_def(half_width, half_height, center, angle,
                                   group_index=-1, density=1., friction=0.)
        self.bodies[name] = create_body(world, [shape_def])

    def init_capsule_body(self, world, name, half_width, half_height, center,
                      angle=0.):
        shape_defs = []
        x, y = center
        dx = cos(angle) * half_width
        dy = sin(angle) * half_width
        kwargs = dict(group_index=-1, density=1., friction=0.)
        shape_defs.append(create_circle_def(half_height, (x + dx, y + dy),
                                            **kwargs))
        shape_defs.append(create_box_def(half_width, half_height, center,
                                         angle, **kwargs))
        shape_defs.append(create_circle_def(half_height, (x - dx, y - dy),
                                            **kwargs))
        self.bodies[name] = create_body(world, shape_defs)

    def init_joint(self, world, joint_name, body_name_1, body_name_2,
                    position):
        joint_def = b2RevoluteJointDef()
        joint_def.Initialize(self.bodies[body_name_1],
                             self.bodies[body_name_2], position)
        joint = world.CreateJoint(joint_def).asRevoluteJoint()
        joint.EnableMotor(joint_name == 'neck')
        joint.SetMaxMotorTorque(50.)
        self.joints[joint_name] = joint

    def step(self, dt):
        torso = self.bodies['torso']
        angle_diff = pi / 2. - torso.angle
        while angle_diff < -pi:
            angle_diff += 2. * pi
        while angle_diff >= pi:
            angle_diff -= 2. * pi
        torque = 500. * angle_diff - 20. * torso.angularVelocity
        torso.ApplyTorque(torque)

class Pose(object):
    def __init__(self, joint_angles):
        self.joint_angles = joint_angles

    def mirror(self):
        self.joint_angles = dict((n, -a)
                                 for n, a in self.joint_angles.iteritems())

class KeyFrameAnimation(object):
    def __init__(self, duration, poses):
        self.duration = duration
        self.poses = poses

    def mirror(self):
        for pose in self.poses:
            pose.mirror()

    def loop(self, ragdoll):
        player = KeyFrameAnimationPlayer(self, ragdoll)
        player.on_end = player.start
        player.start()
        return player

class KeyFrameAnimationPlayer(object):
    def __init__(self, animation, ragdoll, on_end=None):
        self.animation = animation
        self.ragdoll = ragdoll
        self.on_end = on_end
        self.index = 0

    def start(self):
        if self.animation.poses:
            self.index = 0
            self.set_motor_speeds()
            self.index = 1
            interval = self.animation.duration / len(self.animation.poses)
            pyglet.clock.schedule_interval(self.step, interval)

    def set_motor_speeds(self):
        for joint in self.ragdoll.joints.itervalues():
            joint.EnableMotor(False)
        pose = self.animation.poses[self.index]
        for joint_name, angle in pose.joint_angles.iteritems():
            joint = self.ragdoll.joints[joint_name]
            angle_diff = angle - joint.GetJointAngle()
            if angle_diff < -pi:
                angle_diff += 2. * pi
            elif angle_diff >= pi:
                angle_diff -= 2. * pi
            joint.EnableMotor(True)
            interval = self.animation.duration / len(self.animation.poses)
            joint.SetMotorSpeed(angle_diff / interval)
            
    def stop(self):
        pyglet.clock.unschedule(self.step)

    def step(self, dt):
        if self.index < len(self.animation.poses):
            self.set_motor_speeds()
            self.index += 1
        else:
            self.stop()
            if self.on_end is not None:
                self.on_end()

def create_walk_animation():
    poses = []
    def add_pose(**kwargs):
        poses.append(Pose(dict((key.replace('_', '-'), value)
                               for key, value in kwargs.iteritems())))
    add_pose(neck=0.,
             left_shoulder=(-0.75 * pi), right_shoulder=(0.75 * pi),
             left_elbow=(0.5 * pi), right_elbow=(0.5 * pi),
             left_hip=(0.75 * pi), right_hip=(-0.75 * pi),
             left_knee=(-0.25 * pi), right_knee=(-0.25 * pi))
    add_pose(neck=0.,
             left_shoulder=pi, right_shoulder=pi,
             left_elbow=(0.5 * pi), right_elbow=(0.5 * pi),
             left_hip=(-0.5 * pi), right_hip=pi,
             left_knee=(-0.5 * pi), right_knee=0.)
    add_pose(neck=0.,
             left_shoulder=(0.75 * pi), right_shoulder=(-0.75 * pi),
             left_elbow=(0.5 * pi), right_elbow=(0.5 * pi),
             left_hip=(-0.75 * pi), right_hip=(0.75 * pi),
             left_knee=(-0.25 * pi), right_knee=(-0.25 * pi))
    add_pose(neck=0.,
             left_shoulder=pi, right_shoulder=pi,
             left_elbow=(0.5 * pi), right_elbow=(0.5 * pi),
             left_hip=pi, right_hip=(-0.5 * pi),
             left_knee=0., right_knee=(-0.5 * pi))
    return KeyFrameAnimation(1., poses)

class MyWindow(pyglet.window.Window):
    def __init__(self, **kwargs):
        super(MyWindow, self).__init__(**kwargs)
        if self.fullscreen:
            self.set_exclusive_keyboard(True)
            self.set_exclusive_mouse(True)
        self.init_world()
        self.init_platform()
        self.ragdoll = Ragdoll(self.world)
        self.screen_time = 0.
        self.world_time = 0.
        self.time_step = 1. / 60.
        self.circle_vertex_list = create_circle_vertex_list()
        pyglet.clock.schedule_interval(self.step, self.time_step)
        animation = create_walk_animation()
        animation.loop(self.ragdoll)

    def init_world(self):
        aabb = b2AABB()
        aabb.lowerBound = -10., -10.
        aabb.upperBound = 10., 10.
        self.world = b2World(aabb, (0., -10.), True)

    def init_platform(self):
        shape_def = create_box_def(5., 0.1, (0., -1.5), 0.)
        create_body(self.world, [shape_def])

    def step(self, dt):
        self.screen_time += dt
        while self.world_time + self.time_step < self.screen_time:
            self.world_time += self.time_step
            self.ragdoll.step(self.time_step)
            self.world.Step(self.time_step, 10, 10)

    def on_draw(self):
        glClearColor(0., 0., 0., 1.)
        self.clear()
        self.debug_draw()

    def on_close(self):
        pyglet.clock.unschedule(self.step)
        super(MyWindow, self).on_close()

    def debug_draw(self):
        glPushMatrix()
        glTranslatef(float(self.width // 2), float(self.height // 2), 0.)
        scale = 100.
        glScalef(scale, scale, scale)
        for body in self.world.bodyList:
            self.debug_draw_body(body)
        for joint in self.world.jointList:
            self.debug_draw_joint(joint)
        glPopMatrix()

    def debug_draw_body(self, body):
        glPushMatrix()
        glTranslatef(body.position.x, body.position.y, 0.)
        glRotatef(body.angle * 180. / pi, 0., 0., 1.)
        for shape in body.shapeList:
            if isinstance(shape, b2PolygonShape):
                self.debug_draw_polygon(shape.vertices)
            elif isinstance(shape, b2CircleShape):
                x, y = shape.localPosition.tuple()
                self.debug_draw_circle(x, y, shape.radius)
        glPopMatrix()

    def debug_draw_polygon(self, vertices):
        coords = []
        for i in xrange(len(vertices)):
            coords.extend(vertices[i])
            coords.extend(vertices[(i + 1) % len(vertices)])
        pyglet.graphics.draw(len(coords) // 2, GL_LINES,
                             ('v2f', coords))

    def debug_draw_circle(self, x, y, radius):
        glPushMatrix()
        glTranslatef(x, y, 0.)
        glScalef(radius, radius, radius)
        self.circle_vertex_list.draw(GL_LINES)
        glPopMatrix()

    def debug_draw_joint(self, joint):
        x, y = joint.GetAnchor1().tuple()
        self.debug_draw_circle(x, y, 0.1)
        self.debug_draw_circle(x, y, 0.05)

        x, y = joint.GetAnchor2().tuple()
        self.debug_draw_circle(x, y, 0.1)
        self.debug_draw_circle(x, y, 0.05)

    def on_key_press(self, symbol, modifiers):
        if symbol == pyglet.window.key.ESCAPE:
            self.on_close()
        if symbol == pyglet.window.key.F12:
            color_buffer = pyglet.image.get_buffer_manager().get_color_buffer()
            color_buffer.save('screenshot.png')

def main():
    fullscreen = '--fullscreen' in sys.argv
    window = MyWindow(fullscreen=fullscreen)
    pyglet.app.run()

if __name__ == '__main__':
    main()
