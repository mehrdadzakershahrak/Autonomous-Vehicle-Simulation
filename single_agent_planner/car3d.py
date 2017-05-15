# car3d.py {{{
# Description:
#   Car rigid body for 2d environment.
#   This is a rectangle shape, and it keeps its orientation.
#   Position and its orientation can be changed.
# }}}
import pygame
from pygame.locals import *
from pylygon import Polygon
import sys
import time
import copy
from numpy import ndarray, array, dot
from math import fabs, tan, sin, cos, pi, sqrt, atan2

class Environment():
    pass

class Vehicle3D(Polygon):
    def __init__(self, (x, y), theta):
        #self.BASE = 8.0
        #self.BASE = 6.0
        self.BASE = 4.0
        self.x1m = self.BASE * 0.8
        self.x2m = self.BASE * 3.0
        self.x3m = self.BASE * 1.0
        self.y1m = self.BASE * 1.0

        self.L = self.x2m
        
        
#        self.xm = self.BASE * 1.0
#        self.y1m = self.BASE * 0.8
#        self.y2m = self.BASE * 3.0
#        self.y3m = self.BASE * 1.0
#        self.L = self.BASE * 3.0
        self.ackermann_c = array((x, y))
#        p1 = (x - self.xm, y + self.y1m)
#        p2 = (x + self.xm, y + self.y1m)
#        p3 = (x - self.xm, y - (self.y2m + self.y3m))
#        p4 = (x + self.xm, y - (self.y2m + self.y3m))
        p1 = (x - self.x1m, y + self.y1m)
        p2 = (x - self.x1m, y - self.y1m)
        p3 = (x + (self.x2m + self.x3m), y - self.y1m)
        p4 = (x + (self.x2m + self.x3m), y + self.y1m)
        super(Vehicle3D, self).__init__([p1, p2, p3, p4])
        self.rotate_ackermann(theta)

    def rotate_ackermann(self, theta):
        other = Polygon(self.rotopoints_ackermann(theta))
        self.P[:] = other.P
        self.angle = theta

    def rotopoints_ackermann(self, theta):
        P = self.P
        rotate = self._rotate
        return array([rotate(p, theta) for p in P])

    def _rotate(self, x0, theta):
        origin = self.ackermann_c.reshape(2, 1)
        x0 = x0.reshape(2, 1)
        x0 = x0 - origin # assingment operator (-=) would modify original x0
        A = array([[cos(theta), -sin(theta)], # rotation matrix
                    [sin(theta), cos(theta)]])
        return (dot(A, x0) + origin).ravel()

    def drive_ip(self, (x, y, theta)):
        (x1, y1) = self.ackermann_c
        (x2, y2) = (x - x1, y - y1)
        self.move_ip(x2, y2)
        ac = self.ackermann_c
        self.ackermann_c = array((ac[0] + x2, ac[1] + y2))
        if theta < -pi:
            theta += 2.0 * pi
        elif theta > pi:
            theta -= 2.0 * pi
        angle_diff = theta - self.angle
        if angle_diff < -pi:
            angle_diff += 2.0 * pi
        elif angle_diff > pi:
            angle_diff -= 2.0 * pi
        self.rotate_ackermann(angle_diff)

    def draw(self, screen):
        p_int = [(int(p_x), int(p_y)) for (p_x, p_y) in self.P]
        ac_int = (int(self.ackermann_c[0]), int(self.ackermann_c[1]))
        pygame.draw.polygon(screen, (0, 255, 0), p_int, 1)
        pygame.draw.circle(screen, (255, 0, 0), ac_int, 1)

def set_direction(v1, accel):
    if v1 < 0:
        DIR = -1
    elif v1 > 0:
        DIR = 1
    else: # when v1 == 0:
        if accel < 0:
            DIR = -1
        else: # when accel >= 0:
            DIR = 1
    return DIR

def proceed(state, dt, L, (accel, steering)):
    (x1, y1, th1, v1) = state
    DIR = set_direction(v1, accel)
    x2 = x1 + v1 * cos(th1) * dt
    y2 = y1 + v1 * sin(th1) * dt
    th2 = th1 + (v1 / L) * tan(steering) * dt
    (v2, terminate) = update_and_enforce_velocity(v1, DIR, accel, dt)
    return (x2, y2, th2, v2)

def turn_left_90(state, ctrl):
    global ENV
    (x1, y1, th1, v1), (accel, steering) = state, ctrl
    DIR = set_direction(v1, accel)
    th1_orig = th1
    close_th_diff = float("Inf")
    itr = 0
    while True:
        x2 = x1 + v1 * cos(th1) * ENV.dt
        y2 = y1 + v1 * sin(th1) * ENV.dt
        th2 = th1 + (v1 / ENV.L) * tan(steering) * ENV.dt
        (v2, terminate) = update_and_enforce_velocity(v1, DIR, accel, ENV.dt)
        th_diff = fabs(th1_orig - th2)
        left_90 = fabs(th_diff - pi/2)
        if left_90 < close_th_diff:
            close_th_diff = left_90
            (x1, y1, th1, v1) = (x2, y2, th2, v2)
        else:
            return (x1, y1, th1, v1, itr)
        itr += 1
    return (x2, y2, th2, v2, itr)

def turn_right_90(state, ctrl):
    global ENV
    (x1, y1, th1, v1), (accel, steering) = state, ctrl
    DIR = set_direction(v1, accel)
    th1_orig = th1
    close_th_diff = float("Inf")
    itr = 0
    while True:
        x2 = x1 + v1 * cos(th1) * ENV.dt
        y2 = y1 + v1 * sin(th1) * ENV.dt
        th2 = th1 + (v1 / ENV.L) * tan(steering) * ENV.dt
        (v2, terminate) = update_and_enforce_velocity(v1, DIR, accel, ENV.dt)
        th_diff = fabs(th1_orig - th2)
        right_90 = fabs(th_diff - pi/2)
        if right_90 < close_th_diff:
            close_th_diff = right_90
            (x1, y1, th1, v1) = (x2, y2, th2, v2)
        else:
            return (x1, y1, th1, v1, itr)
        itr += 1
    return (x2, y2, th2, v2, itr)

def turn_left_45(state, ctrl):
    global ENV
    (x1, y1, th1, v1), (accel, steering) = state, ctrl
    DIR = set_direction(v1, accel)
    th1_orig = th1
    close_th_diff = float("Inf")
    itr = 0
    while True:
        x2 = x1 + v1 * cos(th1) * ENV.dt
        y2 = y1 + v1 * sin(th1) * ENV.dt
        th2 = th1 + (v1 / ENV.L) * tan(steering) * ENV.dt
        (v2, terminate) = update_and_enforce_velocity(v1, DIR, accel, ENV.dt)
        th_diff = fabs(th1_orig - th2)
        left_45 = fabs(th_diff - pi/4)
        if left_45 < close_th_diff:
            close_th_diff = left_45
            (x1, y1, th1, v1) = (x2, y2, th2, v2)
        else:
            return (x1, y1, th1, v1, itr)
        itr += 1
    return (x2, y2, th2, v2, itr)

def turn_right_45(state, ctrl):
    global ENV
    (x1, y1, th1, v1), (accel, steering) = state, ctrl
    DIR = set_direction(v1, accel)
    th1_orig = th1
    close_th_diff = float("Inf")
    itr = 0
    while True:
        x2 = x1 + v1 * cos(th1) * ENV.dt
        y2 = y1 + v1 * sin(th1) * ENV.dt
        th2 = th1 + (v1 / ENV.L) * tan(steering) * ENV.dt
        (v2, terminate) = update_and_enforce_velocity(v1, DIR, accel, ENV.dt)
        th_diff = fabs(th1_orig - th2)
        right_45 = fabs(th_diff - pi/4)
        if right_45 < close_th_diff:
            close_th_diff = right_45
            (x1, y1, th1, v1) = (x2, y2, th2, v2)
        else:
            return (x1, y1, th1, v1, itr)
        itr += 1
    return (x2, y2, th2, v2, itr)

def update_and_enforce_velocity(v1, DIR, acc, dt):
    global ENV
    terminate = False
    v2 = v1 + DIR * acc * dt
    if v2 < -ENV.V_MAX:
        v2 = -ENV.V_MAX
    elif v2 > ENV.V_MAX:
        v2 = ENV.V_MAX
    elif v1 < 0 and -ENV.V_THRESHOLD <= v2:
        (v2, terminate) = (0.0, True)
    elif v1 >0 and v2 <= ENV.V_THRESHOLD:
        (v2, terminate) = (0.0, True)
    elif v1 == 0 and -ENV.V_THRESHOLD <= v2 and v2 <= ENV.V_THRESHOLD:
        v2 = 0.0
    return (v2, terminate)

def draw_temp_car(screen, p1):
    if not screen:
        raise BaseException
    temp_car = Vehicle3D((p1[0], p1[1]), p1[2])
    tc_int = [(int(px), int(py)) for (px, py) in temp_car.P]
    tcac_int = (int(temp_car.ackermann_c[0]), int(temp_car.ackermann_c[1]))
    pygame.draw.polygon(screen, (0, 255, 0), tc_int, 1)
    pygame.draw.circle(screen, (255, 0, 0), tcac_int, 1)

def draw_temp_car2(screen, p1, color):
    temp_car = Vehicle3D((p1[0], p1[1]), p1[2])
    tc_int = [(int(px), int(py)) for (px, py) in temp_car.P]
    tcac_int = (int(temp_car.ackermann_c[0]), int(temp_car.ackermann_c[1]))
    pygame.draw.polygon(screen, color, tc_int, 1)
    #pygame.draw.polygon(screen, (50, 255, 50), tc_int, 1)
    pygame.draw.circle(screen, (255, 50, 50), tcac_int, 1)

def draw_temp_car_c(screen, p1):
    temp_car = Vehicle3D((p1[0], 800 - p1[1]), p1[2])
    tcac_int = (int(temp_car.ackermann_c[0]), int(temp_car.ackermann_c[1]))
    pygame.draw.circle(screen, (255, 0, 0), tcac_int, 0)

def draw_temp_car3(screen, p1, ENV):
    temp_car = Vehicle3D((p1[0], ENV.MAX_Y - p1[1]), p1[2])
    tc_int = [(int(px), int(py)) for (px, py) in temp_car.P]
    tcac_int = (int(temp_car.ackermann_c[0]), int(temp_car.ackermann_c[1]))
    pygame.draw.polygon(screen, (0, 255, 0), tc_int, 1)
    pygame.draw.circle(screen, (255, 0, 0), tcac_int, 1)

def draw_temp_car4(screen, p1, ENV):
    temp_car = Vehicle3D((p1[0], ENV.MAX_Y - p1[1]), p1[2])
    tc_int = [(int(px), int(py)) for (px, py) in temp_car.P]
    tcac_int = (int(temp_car.ackermann_c[0]), int(temp_car.ackermann_c[1]))
    pygame.draw.polygon(screen, (50, 255, 50), tc_int, 1)
    pygame.draw.circle(screen, (255, 0, 0), tcac_int, 1)

def draw_temp_car_allow(screen, p1, color):
    temp_car = Vehicle3D((p1[0], p1[1]), p1[2])
    tc_int = [(int(px), int(py)) for (px, py) in temp_car.P]
    tcac_int = (int(temp_car.ackermann_c[0]), int(temp_car.ackermann_c[1]))
    pygame.draw.polygon(screen, color, tc_int, 1)
    #pygame.draw.polygon(screen, (50, 255, 50), tc_int, 1)
    pygame.draw.circle(screen, (255, 50, 50), tcac_int, 1)
    ax, ay = temp_car.ackermann_c[0], temp_car.ackermann_c[1]
    #ax2 = ax + 10.0 * cos(p1[2])
    #ay2 = ay + 10.0 * sin(p1[2])
    dist_points = []
    for p1 in tc_int:
        dist_points.append((dist((ax, ay), p1), p1))
    dist_points.sort()
    _, p1 = dist_points.pop(-1)
    _, p2 = dist_points.pop(-1)

    #pygame.draw.line(screen, color, tc_int[2], tc_int[3], 5)
    pygame.draw.line(screen, color, p1, p2, 5)

def dist((x1, y1), (x2, y2)):
    return (x1 - x2) ** 2 + (y1 - y2) ** 2

ENV = Environment()
ENV.V_THRESHOLD = 0.005
ENV.V_MAX = 40
ENV.dt = 0.01
ENV.L = 8.0 * 3.0

if __name__ == '__main__':
    pygame.init()
    SCREEN_SIZE = (800, 600)               # initialize screen size
    SCREEN = pygame.display.set_mode(SCREEN_SIZE) # load screen

    car1 = Vehicle3D((200, 200), 0.0)
    car1.draw(SCREEN)
    pygame.display.update()

    state1 = (200, 200, 0, 20.0)
    (accel, steering) = (0.0, -pi/6)
    temp = state1

    state2 = turn_left_90(state1, (accel, steering))
    (x2, y2, th2, v2, itr) = state2
    print "itr:%d, x2:%05.3f, y2:%05.3f, th2:%05.3f, v2:%05.3f" % (itr, x2, y2, th2, v2)
    #car1.drive_ip((x2, y2, th2))
    #car1.draw(SCREEN)
    draw_temp_car(SCREEN, (x2, y2, th2))
    pygame.display.update()

    state1 = (200, 200, 0, 20.0)
    (accel, steering) = (0.0, pi/6)
    state2 = turn_right_90(state1, (accel, steering))
    (x2, y2, th2, v2, itr) = state2
    print "itr:%d, x2:%05.3f, y2:%05.3f, th2:%05.3f, v2:%05.3f" % (itr, x2, y2, th2, v2)
    draw_temp_car(SCREEN, (x2, y2, th2))
    pygame.display.update()

    #draw_temp_car_allow(SCREEN, (150, 150, pi/5))

#    state1 = (200, 200, 0, 20.0)
#    (accel, steering) = (0.0, -pi/4)
#    state2 = turn_left_45(state1, (accel, steering))
#    (x2, y2, th2, v2, itr) = state2
#    print "itr:%d, x2:%05.3f, y2:%05.3f, th2:%05.3f, v2:%05.3f" % (itr, x2, y2, th2, v2)
#    draw_temp_car(SCREEN, (x2, y2, th2))
#    pygame.display.update()
#
#    state1 = (200, 200, 0, 20.0)
#    (accel, steering) = (0.0, pi/4)
#    state2 = turn_right_45(state1, (accel, steering))
#    (x2, y2, th2, v2, itr) = state2
#    print "itr:%d, x2:%05.3f, y2:%05.3f, th2:%05.3f, v2:%05.3f" % (itr, x2, y2, th2, v2)
#    draw_temp_car(SCREEN, (x2, y2, th2))
#    pygame.display.update()

    #for i in range(20):
    #    state2 = proceed(temp, ENV.dt, ENV.L, (accel, steering))
    #    (x2, y2, th2, v2) = state2
    #    temp = state2
    #    print "i:%d, x2:%05.3f, y2:%05.3f, th2:%05.3f, v2:%05.3f" % (i, x2, y2, th2, v2)
    #car1.drive_ip((x2, y2, th2))
    #car1.draw(SCREEN)
    #pygame.display.update()

    while True:
        for ev in pygame.event.get():
            if ev.type == KEYDOWN:
                if ev.key == K_q: exit()
            if ev.type == QUIT:
                pygame.quit()
                sys.exit(0)
        pygame.display.update()
