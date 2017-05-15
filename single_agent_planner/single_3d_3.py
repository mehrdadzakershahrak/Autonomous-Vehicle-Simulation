# single_3d.py {{{
# Description:
#   Single Agent RRT for 2D environment.
#   We here use a car-like model.
#   It should be dual tree RRT.
#   Searching space is (x, y, th, v).
#   The control input is (accel, steering).
# }}}

#---Package imports-------------------------------------{{{1
from math import fabs, tan, sin, cos, pi, sqrt, atan2
import car3d as vehicle
import pygame
from pygame.locals import *
from pygame import gfxdraw
import time
import sys
from pylygon import Polygon
from numpy import ndarray, array, dot, isclose
import copy
import treetemp2d as tr
import treetemp3d as tr3
#from map_loader4 import load_map, get_near_borders_lpoly, draw_triangles
#from map_loader4_1 import load_map, get_near_borders_lpoly, draw_triangles, get_shortest_path
#from map_loader4_6 import load_map, get_near_borders_lpoly, draw_triangles, get_shortest_path
#from map_loader4_6 import load_map
from random import uniform, random, randint, sample

#---TODO------------------------------------------------{{{1o
#---@2016-11-29-----------------------------------------{{{2
#   1. simulate_2d_rigid() only checks 2d rigid body, not considering
#     its orientation. We need to look into the raycast() function
#     in order to use its theta argument.
#
#---@2016-11-28-----------------------------------------{{{2
#   1. implement single agent rrt for 2d with orientation
#   2. add rigid body
#   3. try in exenv2.svg for parallel parking

#---Classes---------------------------------------------{{{1
class Environment():
    pass

#---Distance--------------------------------------------{{{1
def distance_2d(p1, p2):
    (x1, y1) = (list(p1)[0], list(p1)[1])
    (x2, y2) = (list(p2)[0], list(p2)[1])
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def step_from_to_2d(p1, p2):
    global ENV
    (x1, y1) = (list(p1)[0], list(p1)[1])
    (x2, y2) = (list(p2)[0], list(p2)[1])
    if distance_2d(p1, p2) >= ENV.EPSILON:
        theta = atan2(y2-y1, x2-x1)
        return (x1 + ENV.EPSILON * cos(theta), y1 + ENV.EPSILON * sin(theta))
    else:
        return p2

def step_from_to_2d_2(p1, p2, epsilon):
    (x1, y1) = (list(p1)[0], list(p1)[1])
    (x2, y2) = (list(p2)[0], list(p2)[1])
    dist_2d = distance_2d(p1, p2)
    if dist_2d >= epsilon:
        theta = atan2(y2-y1, x2-x1)
        x3, y3 = x1 + epsilon * cos(theta), y1 + epsilon * sin(theta)
        return (x3, y3), epsilon
    else:
        return (x2, y2), dist_2d

def distance(p1, p2):
    (p1, p2) = (list(p1), list(p2))
    assert(len(p1) >= 3)
    assert(len(p2) >= 3)
    val = fabs(p1[2] - p2[2])
    if val > pi:
        val = 2.0 * pi - val
    return sqrt(val**2 + (p1[1] - p2[1])**2 + (p1[0] - p2[0])**2)

#---Validation------------------------------------------{{{1
def enforce_bounds_2d(p1, ENV):
#    global ENV
    passed = False
    if p1[0] <= ENV.MIN_X:
        p1[0] = ENV.MIN_X
        passed = True
    elif p1[0] >= ENV.MAX_X:
        p1[0] = ENV.MAX_X
        passed = True
    if p1[1] <= ENV.MIN_Y:
        p1[1] = ENV.MIN_Y
        passed = True
    elif p1[1] >= ENV.MAX_Y:
        p1[1] = ENV.MAX_Y
        passed = True
    assert(passed == False)
    return (p1, passed)

def valid_state_2d(p1, ENV):
    #global ENV
    BELOW_MIN = p1[0] <= ENV.MIN_X or p1[1] <= ENV.MIN_Y
    ABOVE_MAX = p1[0] >= ENV.MAX_X or p1[1] >= ENV.MAX_Y
    return not (BELOW_MIN or ABOVE_MAX)

def enforce_velocity(v1, DIR, acc, ENV):
  #global ENV
  VTHRES = ENV.V_THRESHOLD
  terminate = False
  v2 = v1 + DIR * acc * ENV.dt
  if v2 < -ENV.V_MAX:
    v2 = -ENV.V_MAX
  elif v2 > ENV.V_MAX:
    v2 = ENV.V_MAX
  elif v1 < 0 and -VTHRES <= v2:
    (v2, terminate) = (0.0, True)
  elif v1 > 0 and v2 <= VTHRES:
    (v2, terminate) = (0.0, True)
  elif v1 == 0 and -VTHRES <= v2 and v2 <= VTHRES:
    v2 = 0.0
  return (v2, terminate)

def enforce_theta(th1, th2):
    th2 = th2 + th1
    if th2 < -pi:
        th2 += 2.0 * pi
    elif th2 > pi:
        th2 -= 2.0 * pi
    return th2

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

#---Finding---------------------------------------------{{{1
def find_closest(tree, p1):
    (nearest, n_dist) = tree.search_nn(p1[:2])
    return (nearest, sqrt(n_dist))

def find_near_nodes(tx, rpos, ENV):
    #global ENV
    near_nodes = set()
    ncoords = tx.search_nn_dist(rpos, ENV.EPSILON * 3)
    if len(ncoords) == 0:
        nnode, _ = tx.search_nn(rpos[:2])
        return tx.nodes[nnode.nvalue[:2]]
    for coord in ncoords:
        near_nodes = near_nodes.union(tx.nodes[coord])
    return near_nodes

def find_goal_near(tx, gpos):
    global ENV
    near_nodes = set()
    ncoords = tx.search_nn_dist(gpos[:2], ENV.EPSILON * 5)
    if len(ncoords) == 0:
        nnode, _ = tx.search_nn(gpos[:2])
        return tx.nodes[nnode.nvalue[:2]]
    for coord in ncoords:
        near_nodes = near_nodes.union(tx.nodes[coord])
    return near_nodes

def find_goal_near2(tx, gpos, ENV):
    near_nodes = set()
    ncoords = tx.search_nn_dist(gpos[:2], ENV.EPSILON * 5)
    if len(ncoords) == 0:
        nnode, _ = tx.search_nn(gpos[:2])
        return tx.nodes[nnode.nvalue[:2]]
    for coord in ncoords:
        near_nodes = near_nodes.union(tx.nodes[coord])
    return near_nodes

#---Sampling--------------------------------------------{{{1
def random_state_2d(goal_state):
    global ENV
    if random() <= ENV.goal_chance:
        return goal_state
    else:
        return (randint(ENV.MIN_X,ENV.MAX_X), randint(ENV.MIN_Y,ENV.MAX_Y))

def random_local_state_2d(src, dest, r, tcond, ENV):
    #global ENV
    if random() <= ENV.goal_chance:
        return dest
    else:
        #alpha = 2 * (pi / 3.0) * random() - (pi / 3.0)
        #alpha = 2 * (pi / 6.0) * random() - (pi / 6.0)
        #alpha = 2 * (pi / 4.0) * random() - (pi / 4.0)
        alpha = 2 * tcond[3] * random() - tcond[3]
        r2 = r * random() * tcond[2]
        # this is only for 2D (which doesn't keep its orientation)
        th0 = atan2(dest[1] - src[1], dest[0] - src[0])
        th = th0 + alpha
        if th < -pi:
            th += 2.0 * pi
        elif th > pi:
            th -= 2.0 * pi
        x = r2 * cos(th) + src[0]
        y = r2 * sin(th) + src[1]
        #x = r2 * cos(th - alpha) + src[0]
        #y = r2 * sin(th - alpha) + src[1]
        #x = r2 * cos(src[2] - alpha) + src[0]
        #y = r2 * sin(src[2] - alpha) + src[1]
        return (x, y)

def random_control(ENV):
  #global ENV
  accel = uniform(-ENV.MAX_ACCEL, ENV.MAX_ACCEL)
  steering = uniform(-ENV.MAX_STEER, ENV.MAX_STEER)
  return (accel, steering)

#---Collision-------------------------------------------{{{1
def simulate_2d_rigid((x1, y1), (x2, y2)):
    global ENV
    ### get near borders' lpoly
    radius = distance_2d((x1, y1), (x2, y2))
    lpolies = get_near_borders_lpoly((x1, y1), ENV, radius * 2)

    ### get the rigid body
    car = vehicle.Vehicle2D((x1, y1))
    r = array((x1 - x2, y1 - y2))

    ### raycast the rigid body for vector r with each lpoly
    for lpoly in lpolies:
        results = car.raycast(lpoly, r)
        if results:
            return False

    return True

def simulate_2d_rigid_theta(s1, s2, th, near_lpolys):
    global ENV

    (x1, y1, th1, _) = s1
    (x2, y2, th2, _) = s2

    origin = vehicle.Vehicle3D((x1, y1), th1)
    r = array((x1 - x2, y1 - y2))

    for lpoly in near_lpolys:
        results = origin.raycast(lpoly, r, self_theta = th)
        if results:
            return False
    return True

def conflict(s1, s2, th, near_npolys):
    global ENV

    (x1, y1, th1), (x2, y2) = s1, s2

    #origin = vehicle.Vehicle3D((x1, y1), th1)
    origin = vehicle.Vehicle3D((x1, ENV.MAX_Y - y1), th1)
    r = array((x1 - x2, y1 - y2))

    for lpoly in near_npolys:
        results = origin.raycast(lpoly, r, self_theta = th)
        if results:
            return True
    return False

def collide((x1, y1, th1), near_lpolys, ENV):
    #global ENV
    #car = vehicle.Vehicle3D((x1, y1), th1)
    car = vehicle.Vehicle3D((x1, ENV.MAX_Y - y1), th1)
    for lpoly in near_lpolys:
        proj = car.collidepoly(lpoly)
        if type(proj) is bool and proj:
            return True
        if type(proj) is ndarray and proj.size > 0:
            return True
        if type(proj) is array and proj.size > 0:
            return True
    return False

#---Simulate--------------------------------------------{{{1
def simulate(tx, nears, zn, goal_state, ENV):
    #global ENV
    (nearest_coords, n_dist) = find_closest(tx, zn)
    if zn == goal_state[:2] and n_dist <= ENV.EPSILON:
        goal_nears = find_goal_near2(tx, goal_state, ENV)
        return simulate_goal(tx, goal_nears, goal_state, ENV)
    else:
        return simulate_many(tx, nears, zn, ENV)

def simulate_many(tx, nears, zn, ENV):
    #global ENV
    nosamples = 15 if len(nears) > 15 else len(nears)
    sampled_states = sample(nears, nosamples)

    # distance, ctrl, near_node, state, # of steps
    Mstate = (float("Inf"), None, None, None, None)

    for nearone in sampled_states:
        #zn = add_theta(nearone.nvalue, zn)
        (xi, yi), (xj, yj) = nearone.nvalue[:2], zn[:2]
        zn = (xj, yj, atan2(yj - yi, xj - xi))
        near_lpolys = ENV.tg.get_near_borders_lpoly(nearone.nvalue[:2], 30)
        (x0, y0, th0, v0) = copy.deepcopy(nearone.nvalue)
        for j in range(ENV.NOSAMPLES):
            (x, y, th, v) = (x0, y0, th0, v0)
            (accel, steering) = random_control(ENV)
            DIR = set_direction(v, accel)
            valid = True
            for i in range(ENV.MAX_NSTEP):
                x2 = x + v * cos(th) * ENV.dt
                y2 = y + v * sin(th) * ENV.dt
                self_th = (v/ENV.L) * tan(steering) * ENV.dt
                th2 = enforce_theta(th, self_th)
                (v2, terminate) = enforce_velocity(v, DIR, accel, ENV)
                valid = valid and valid_state_2d((x, y), ENV)
                if not valid:
                    break
                cur_dist = distance((x, y, th), zn)
                if isclose(((i+1) * ENV.dt) % 2, 0):
                    if collide((x2, y2, th2), near_lpolys, ENV):
                        break
                # if collide((x2, y2, th2), near_lpolys, ENV):
                #     break

                #if isclose(((i+1) * ENV.dt) % 3, 0):
                #    if collide((x2, y2, th2), near_lpolys):
                #        break
                #if conflict((x, y, th), (x2, y2), self_th, near_lpolys):
                #    break
                (x, y, th, v), (ctrl) = (x2, y2, th2, v2), (accel, steering)
                if cur_dist < Mstate[0]:    # distance
                    Mstate = (cur_dist, ctrl, nearone, (x, y, th, v), i+1)
                if terminate:
                    break

    if Mstate[3]:  # state (x, y, th, v)
        return (True, Mstate[2], Mstate[3], Mstate[1], Mstate[4])
    else:
        return (False, None, None, None, None)

def simulate_goal(tx, goal_nears, zn, ENV):
    #global ENV

    # close_diff, ctrl, near, state, # of steps
    #Cstate = (ENV.delta_orient, None, None, None, None)
    Cstate = (pi / 12.0, None, None, None, None)
    # distance, ctrl, near, state, # of steps
    Mstate = (float("Inf"), None, None, None, None)

    for nearone in goal_nears:
        near_lpolys = ENV.tg.get_near_borders_lpoly(nearone.nvalue[:2], 70)
        (x0, y0, th0, v0) = copy.deepcopy(nearone.nvalue)
        for j in range(ENV.NOSAMPLES):
            (x, y, th, v) = (x0, y0, th0, v0)
            (accel, steering) = random_control(ENV)
            DIR = set_direction(v, accel)
            valid = True
            for i in range(ENV.MAX_NSTEP):
                x2 = x + v * cos(th) * ENV.dt
                y2 = y + v * sin(th) * ENV.dt
                self_th = (v/ENV.L) * tan(steering) * ENV.dt
                th2 = enforce_theta(th, self_th)
                (v2, terminate) = enforce_velocity(v, DIR, accel, ENV)
                valid = valid and valid_state_2d((x, y), ENV)
                if not valid:
                    break
                if isclose(((i+1) * ENV.dt) % 3, 0):
                    if collide((x2, y2, th2), near_lpolys, ENV):
                        break

                # if collide((x2, y2, th2), near_lpolys, ENV):
                #     break

                #if conflict((x, y, th), (x2, y2), self_th, near_lpolys):
                #    break
                (x, y, th, v), (ctrl) = (x2, y2, th2, v2), (accel, steering)
                cur_dist = distance((x, y, th), zn)
                orient_diff = fabs(th - zn[2])
                if orient_diff < Cstate[0]: # close_diff
                    Cstate = (orient_diff, ctrl, nearone, (x, y, th, v), i+1)
                if cur_dist < Mstate[0]:
                    Mstate = (cur_dist, ctrl, nearone, (x, y, th, v), i+1)
                if terminate:
                    break

    if Cstate[3]:
        return (True, Cstate[2], Cstate[3], Cstate[1], Cstate[4])
    elif Mstate[3]:
        return (True, Mstate[2], Mstate[3], Mstate[1], Mstate[4])
    else:
        return (False, None, None, None, None)

def simulate_many2(tx, nears, zn, ENV):
    nosamples = 15 if len(nears) > 15 else len(nears)
    sampled_states = sample(nears, nosamples)

    Mstate = (float("Inf"), None, None, None, None)
    chains = {}
    for nearone in sampled_states:
        (xi, yi), (xj, yj) = nearone.nvalue[:2], zn[:2]
        zn = (xj, yj, atan2(yj - yi, xj - xi))
        near_lpolys = ENV.tg.get_near_borders_lpoly(nearone.nvalue[:2], 30)
        (x0, y0, th0, v0) = copy.deepcopy(nearone.nvalue)
        for j in range(ENV.NOSAMPLES):
            (x, y, th, v) = (x0, y0, th0, v0)
            (accel, steering) = random_control(ENV)
            DIR = set_direction(v, accel)
            valid = True
            ctrl = (accel, steering)
            chains[ctrl] = []
            for i in range(ENV.MAX_NSTEP):
                x2 = x + v * cos(th) * ENV.dt
                y2 = y + v * sin(th) * ENV.dt
                self_th = (v/ENV.L) * tan(steering) * ENV.dt
                th2 = enforce_theta(th, self_th)
                (v2, terminate) = enforce_velocity(v, DIR, accel, ENV)
                valid = valid and valid_state_2d((x, y), ENV)
                if not valid:
                    break
                chains[ctrl].append(((x, y, th, v2),(x2, y2, th2, v2)))
                (x, y, th, v), (ctrl) = (x2, y2, th2, v2), (accel, steering)
                if terminate:
                    break
            if len(chains) == 0:
                continue
            index = len(chains) - 1
            while True:
                _, (x2, y2, th2, v2) = chains[ctrl][index]
                collided = collide((x2, y2, th2), near_lpolys, ENV)
                if not collided:
                    chains[ctrl] = chains[ctrl][:index+1]
                    break
                index = int(round(index / 2.0))


def add_theta(p1, p2):
    (x1, y1, th1), (x2, y2) = list(p1)[:3], list(p2)[:2]
    th2 = th1 + atan2(y2 - y1, x2 - x1) 
    if th2 < -pi:
        th2 += 2.0 * pi
    elif th2 > pi:
        th2 -= 2.0 * pi
    #th2 = th2 > pi ? 2.0 * pi - th2 : th2
    return (x2, y2, th2)

#---Planning--------------------------------------------{{{1
def plan_step_single_3d(tw, tx, car):
    global ENV, SCREEN

    ### Sampling & find closest
    Goal = car.dest
    rp = random_state_2d(Goal)
    closest, _ = find_closest(tw, rp)
    Znew = step_from_to_2d(closest.nvalue, rp)

    #### Collision check
    #if not simulate_2d_rigid(closest.nvalue[:2], rp[:2]):
    #    return (tw, tx)

    ### Add it to a tree tw
    try:
        Znode = tr.Node(Znew)
        tw.add_tree(closest, Znode)
    except BaseException:
        return (tw, tx)

    ### Simulate it for 4D state space
    nears = find_near_nodes(tx, Znode.nvalue)
    (passed, near, xnew, xctrl, xsteps) = simulate(tx, nears, Znew, Goal)
    if not passed:
        return (tw, tx)

    ### Add it to a tree tx
    try:
        Xnode = tr3.Node(xnew)
        tx.add_tree(near, Xnode, xctrl, xsteps * ENV.dt)
        #tx.add_tree(nearone, xnew, xnew_ctrl, xnew_steps * ENV.dt)
    except BaseException:
        return (tw, tx)

    ### Check if goal reached
    xn_goal_dist = distance_2d(Xnode.nvalue[:2], Goal)
    if xn_goal_dist < tx.close_dist:
        (tx.close_node, tx.close_dist) = (Xnode, xn_goal_dist)
    if tx.close_dist <= ENV.goal_near:
        if tx.close_node.cweight < tx.path_cost:
            tx.path_cost = tx.close_node.cweight
            tx.path = tx.compute_path(tx.close_node)

    ### Return
    return (tw, tx)

def plan_step_single_rrt_3d(tw, tx, src, dest, tcond, ENV):

    rp = random_local_state_2d(src, dest, distance_2d(src, dest), tcond, ENV)
    closest, _ = find_closest(tw, rp)
    #Znew = step_from_to_2d(closest.nvalue, rp)
    dist = ENV.EPSILON * 2.0
    collided = True
    near_lpolys = ENV.tg.get_near_borders_lpoly(closest.nvalue, 30)
    if closest != tw.root:
        (x0, y0), (x1, y1) = closest.parent.nvalue, closest.nvalue
        th1 = atan2(y1 - y0, x1 - x0)
    else:
        (x1, y1, th1) = tx.root.nvalue[:3]
    while collided:
        dist *= 0.5
        #Znew, dist = step_from_to_2d_2(closest.nvalue, rp, dist)
        (x2, y2), dist = step_from_to_2d_2(closest.nvalue, rp, dist)
        #_, __, th2 = add_theta((x1, y1, th1), Znew)
        #th2 = atan2(Znew[1] = y1, Znew[0] - x1)
        #collided = collide(tuple(list(Znew[:2]) + [th2]), near_lpolys, ENV)
        th2 = atan2(y2 - y1, x2 - x1)
        collided = collide((x2, y2, th2), near_lpolys, ENV)
        if dist < 0.5:
            return (tw, tx)
    try:
        #Znode = tr.Node(Znew)
        Znode = tr.Node((x2, y2))
        tw.add_tree(closest, Znode)
    except BaseException:
        return (tw, tx)

    nears = find_near_nodes(tx, Znode.nvalue, ENV)
    #(passed, near, xnew, xctrl, xsteps) = simulate(tx, nears, Znew, dest, ENV)
    (passed, near, xnew, xctrl, xsteps) = simulate(tx, nears, (x2, y2), dest, ENV)
    if not passed:
        return (tw, tx)

    try:
        Xnode = tr3.Node(xnew)
        tx.add_tree(near, Xnode, xctrl, xsteps * ENV.dt)
    except BaseException:
        return (tw, tx)

    xn_goal_dist = distance_2d(Xnode.nvalue[:2], dest)
    #theta_close = fabs(Xnode.nvalue[2] - dest[2]) <= (pi / 8.0) # 48: 7.5'
    theta_close = fabs(Xnode.nvalue[2] - dest[2]) <= tcond[1]
    if xn_goal_dist < tx.close_dist and theta_close:  
        (tx.close_node, tx.close_dist) = (Xnode, xn_goal_dist)
    if tx.close_dist <= tcond[0] and theta_close:  
        # goal_near can be variable, depending on |th1 - th2|
        if tx.close_node.cweight < tx.path_cost:
            tx.path_cost = tx.close_node.cweight
            tx.path = tx.compute_path(tx.close_node)

    return (tw, tx)

#---Drawing---------------------------------------------{{{1
def draw_path_2d(screen, tree):
    #screen.fill((0,0,0))
    for (u, v) in tree.path:
        draw_line_2d_red(screen, u.nvalue, v.nvalue)
    pygame.display.update()

def draw_line_2d_red(screen, (x1, y1), (x2, y2), update=False):
    global ENV
    DIR_COLOR = (255, 0, 0)    # red
    pygame.draw.line(screen, DIR_COLOR, (int(x1), int(y1)), (int(x2), int(y2)), 3)
    if update:
        pygame.display.update()

def draw_line_2d_blue(screen, (x1, y1), (x2, y2), update=False):
    global ENV
    DIR_COLOR = (0, 0, 255)    # blue
    pygame.draw.line(screen, DIR_COLOR, (int(x1), int(y1)), (int(x2), int(y2)), 3)
    if update:
        pygame.display.update()

def draw_line_3d(ENV, screen, car, state, ctrl, lsteps, update = False):
#    global ENV
    (x1, y1, th1, v1), (accel, steering) = state, ctrl
    DIR = set_direction(v1, accel)
    if DIR < 0:
        #DIR_COLOR = (135, 206, 250) # light sky blue
        DIR_COLOR = (0, 191, 255) # deep sky blue
    else:
        DIR_COLOR = (255, 255, 20)  # yellow
    #print_str = "=>((%05.3f,%05.3f)," % (x1, y1)
    for i in range(lsteps):
        x2 = x1 + v1 * cos(th1) * ENV.dt
        y2 = y1 + v1 * sin(th1) * ENV.dt
        #th2 = th1 + (v1/car.L) * tan(steering) * ENV.dt
        #self_th = (v1/car.L) * tan(steering) * ENV.dt
        #th2 = enforce_theta(th1, self_th)
        th2 = enforce_theta(th1, (v1/car.L) * tan(steering) * ENV.dt)
        (v2, terminate) = enforce_velocity(v1, DIR, accel, ENV)
        (x2, y2, th2), passed = enforce_bounds_2d((x2, y2, th2), ENV)
#        p1, p2 = (int(x1), ENV.MAX_Y - int(y1)), (int(x2), ENV.MAX_Y - int(y2))
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        pygame.draw.line(screen, DIR_COLOR, p1, p2)
        #print "=>%i:((%04.2f,%04.2f),(%04.2f,%04.2f))"%(i,x1,y1,x2,y2)
        (x1, y1, th1, v1) = (x2, y2, th2, v2)
        if terminate:
            break
    if update:
        pygame.display.update()
    #print_str += "(%05.3f,%05.3f))" % (x1, y1)
#    print print_str

def draw_path_3d(ENV, screen, car, tree, update=False):
#    global ENV
    v_cursor, _ = tree.path[0]
    #for (u, v) in tree.path:
    for i, (u, v) in enumerate(tree.path):
        assert (v_cursor == u)
        v_cursor = v
        #lsteps = int(v.tweight / ENV.dt)
        lsteps = int(round(v.tweight / ENV.dt))
        assert (isclose(lsteps, v.tweight / ENV.dt))
        #draw_line_3d(screen, car, u.nvalue, v.ctrl, v.tweight/ENV.dt, update)
        (x1, y1), (x2, y2) = u.nvalue[:2], v.nvalue[:2]
        (th1, v1), (th2, v2) = u.nvalue[2:4], v.nvalue[2:4]
        print "path:%d:%d,(%03.2f,%03.2f),((%05.3f,%05.3f,%03.2f,%03.2f),(%05.3f,%05.3f,%03.2f,%03.2f))" % (i,lsteps,v.ctrl[0],v.ctrl[1],x1,y1,th1,v1,x2,y2,th2,v2)
        draw_line_3d(ENV, screen, car, u.nvalue, v.ctrl, lsteps, update)
    if update:
        pygame.display.update()

def draw_traj(ENV, screen, traj, color):
    for _, tx in traj:
        u, _ = tx.path[0]
        vehicle.draw_temp_car_allow(screen, u.nvalue[:3], color)
        draw_path_3d(ENV, screen, ENV.car, tx)
    _, v = tx.path[-1]
    vehicle.draw_temp_car_allow(screen, v.nvalue[:3], color)
    pygame.display.update()

def draw_arc(ENV, screen, p1, p2):
    (x1, y1), (x2, y2) = p1[:2], p2[:2]
    radius = distance_2d((x1, y1), (x2, y2)) * 1.4
    th1 = atan2(y2 - y1, x2 - x1)
    th1_s, th1_e = th1 - pi / 4.0, th1 + pi / 4.0
    sx, sy = x1 + cos(th1_s) * radius, y1 + sin(th1_s) * radius
    ex, ey = x1 + cos(th1_e) * radius, y1 + sin(th1_e) * radius
    rect = ((x1 - radius, y1 - radius), (radius * 2, radius * 2))
    pygame.draw.line(screen, (0, 191, 255), (int(x1), int(y1)), (int(sx), int(sy)), 1)
    pygame.draw.line(screen, (0, 191, 255), (int(x1), int(y1)), (int(ex), int(ey)), 1)
    dx, dy = x2 + 4.0 * cos(p2[2]), y2 + 4.0 * sin(p2[2])
    pygame.draw.line(screen, (255, 255, 255), (int(x2), int(y2)), (int(dx), int(dy)), 2)

    pygame.draw.circle(screen, (255, 50, 50), (int(x1), int(y1)), 1)
    pygame.draw.circle(screen, (255, 50, 50), (int(x2), int(y2)), 1)
    # if th1_s < 0:
    #     th2_s = -1.0 * th1_s 
    #     if th1_e < 0:
    #         th2_e = -1.0 * th1_e
    #     else:
    #         th2_e = 2.0 * pi - th1_e
    # else:
    #     th2_s = 2.0 * pi - th1_s
    #     if th1_e < 0:
    #         th2_e = -1.0 * th1_e
    #     else:
    #         th2_e = 2.0 * pi - th1_e
    # th_s, th_e = min(th2_s, th2_e), max(th2_s, th2_e)
    # pygame.draw.arc(screen, (0, 191, 255), rect, th_s, th_e, 1)

    if th1_s < 0 and th1_e > 0:
        th2_s = -1.0 * th1_s
        th2_s0 = 0.0
        th2_e0 = 2.0 * pi - th1_e
        th2_e = 1.99999 * pi
        pygame.draw.arc(screen, (0, 191, 255), rect, th2_s0, th2_s, 1)
        pygame.draw.arc(screen, (0, 191, 255), rect, th2_e0, th2_e, 1)
    elif th1_s < 0 and th1_e == 0:
        th2_s = -1.0 * th1_s
        pygame.draw.arc(screen, (0, 191, 255), rect, th1_e, th2_s, 1)
    elif th1_s == 0 and th1_e > 0:
        th1_s = 1.99999 * pi
        th2_e = 2 * pi - th1_e
        pygame.draw.arc(screen, (0, 191, 255), rect, th1_e, th2_s, 1)
    elif th1_s > 0 and th1_e > 0:
        th2_s = 2.0 * pi - th1_s
        th2_e = 2.0 * pi - th1_e
        th_s, th_e = min(th2_s, th2_e), max(th2_s, th2_e)
        pygame.draw.arc(screen, (0, 191, 255), rect, th_s, th_e, 1)
    elif th1_s < 0 and th1_e < 0:
        th2_s = -1.0 * th1_s
        th2_e = -1.0 * th1_e
        th_s, th_e = min(th2_s, th2_e), max(th2_s, th2_e)
        pygame.draw.src(screen, (0, 191, 255), rect, th_s, th_e, 1)
    else:
        pygame.draw.arc(screen, (0, 191, 255), rect, 0.0, 2.0 * pi, 3)
    pygame.display.update()

#---Main Codes------------------------------------------{{{1
def run_single_rrt_3d():
    global ENV, SCREEN

    tick = time.time()
    root_node_tw = tr.Node(ENV.car.src[:2])
    root_node_tx = tr3.Node(ENV.car.src)

    for itr1 in range(ENV.max_iter):
        tw = tr.Tree(root_node_tw)
        tx = tr3.Tree(root_node_tx)
        min_pcost = float("Inf")
        for itr2 in range(ENV.rrt_cutoff):
            (tw, tx) = plan_step_single_3d(tw, tx, ENV.car)
            if len(tx.path) > 0 and tx.path_cost < min_pcost:
                tock = time.time()
                min_pcost = tx.path_cost
                ENV.itr2_csr = itr2
                draw_path_3d(SCREEN, ENV.car, tx)
                #vehicle.draw_temp_car3(SCREEN, tx.close_node.nvalue, ENV)
                #vehicle.draw_temp_car2(SCREEN, ENV.car.dest, ENV)
                #vehicle.draw_temp_car2(SCREEN, tx.close_node.nvalue)
                #vehicle.draw_temp_car2(SCREEN, ENV.car.dest)
                pygame.display.update()
                print "|path|:%d, %05.3f sec" % (len(tx.path), tock - tick)
                break
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    pygame.quit()
                    sys.exit(0)
        if min_pcost < float("Inf"):
            ENV.itr1_csr = itr1
            return True
        else:
            print "itr2:%d, |tw|:%d, |tx|:%d" % (itr2, len(tw), len(tx))
            if len(tx) < 5:
                print "tx:",tx
                print tx.nodes
    return False

def run_rrt_3d_p(ENV, screen, ls, lg):
    root_node_tw = tr.Node(ls[:2])
    root_node_tx = tr3.Node(ls[:4])
    if fabs(ls[2] - lg[2]) >= pi / 2.0:
    #if fabs(ls[2] - lg[2]) > pi / 5.0:
        #tcond = (ENV.goal_near * 1.5, pi/3.0, 2.0, pi/3.0)
        tcond = (ENV.goal_near * 1.5, pi*(2.0/3.0), 2.0, pi/2.0)
    else:
        tcond = (ENV.goal_near, pi / 8.0, 1.4, pi/4.0)
    for itr1 in range(ENV.max_iter):
        tw, tx = tr.Tree(root_node_tw), tr3.Tree(root_node_tx)
        min_pcost = float("Inf")
        for itr2 in range(ENV.rrt_cutoff):
            (tw, tx) = plan_step_single_rrt_3d(tw, tx, ls, lg, tcond, ENV)
            if len(tx.path) > 0 and tx.path_cost < min_pcost:
                min_pcost = tx.path_cost
                draw_path_3d(ENV, screen, ENV.car, tx)
                #vehicle.draw_temp_car2(screen, tx.close_node.nvalue)
                pygame.display.update()
                break
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    pygame.quit()
                    sys.exit(0)
        if min_pcost < float("Inf"):
            return True, tx.close_node.nvalue, tx
        else:
            print "itr2:%d, |tw|:%d, |tx|:%d" % (itr2, len(tw), len(tx))
            if len(tx) < 5:
                print "tx:",tx
                print tx.nodes
    return False, (-1, -1, 0, 0), None

def run_rrt_3d_full(ENV, screen, path, color):
    tw, tx, succeed, (cursor, _) = None, None, False, path[0]
    #while cursor != path[-1][1]:
    # TODO: for path[-1][1], we have to add the velocity as 0.0.
    #   We may also have to specify the driving direction whether forward or backward.
    #vehicle.draw_temp_car2(screen, path[0][0])
    #pygame.display.update()
    (x1, y1, th) = path[0][0][:3]
    print "run_rrt_3d_full, path:"
    print "(%04.1f, %04.1f, %04.1f)" % (x1, y1, th)
    #for (u, v) in path:
    #    draw_arc(ENV, screen, u, v)
    #    #vehicle.draw_temp_car2(screen, v)
    #    print "(%04.1f, %04.1f, %04.1f)" % (v[0], v[1], v[2])
   
    traj = []
    for (u, v) in path:
        #vehicle.draw_temp_car2(screen, cursor, color)
        vehicle.draw_temp_car_allow(screen, cursor, color)
        pygame.display.update()
        dist = distance_2d(cursor, v)
        diff = fabs(cursor[2] - v[2])
        #(x1, y1, th1, v1), (x2, y2, th2, v2) = cursor, v
        #print "(%04.1f, %04.1f, %04.1f), diff:%03.2f" % (v[0], v[1], v[2], fabs(cursor[2] - v[2]))
        #print "[1] (%04.1f, %04.1f, %03.2f, %03.2f), (%04.1f, %04.1f, %03.2f, %03.2f), dist:%03.2f, diff:%03.2f" % (x1, y1, th1, v1, x2, y2, th2, v2, dist, diff)
        (x1, y1, th1) = cursor[:3]
        if len(cursor) == 4:
            v1 = cursor[3]
        elif len(cursor) == 3:
            v1 = 100.0
        else:
            raise BaseException
        ptr = "[1] (%04.1f, %04.1f, %03.2f, %03.2f)," % (x1, y1, th1, v1)
        (x2, y2, th2) = v[:3]
        if len(v) == 4:
            v2 = v[3]
        elif len(v) == 3:
            v2 = 100.0
        else:
            raise BaseException
        ptr += "(%04.1f, %04.1f, %03.2f, %03.2f)," % (x2, y2, th2, v2)
        ptr += "dist:%03.2f, diff:%03.2f" % (dist, diff)
        print ptr
        succeed, cnode, tx = run_rrt_3d_p(ENV, screen, cursor, v)
        if succeed:
            (x3, y3, th3, v3) = cnode
            print "[2] (%04.1f, %04.1f, %03.2f, %03.2f), (%04.1f, %04.1f, %03.2f, %03.2f), |path|:%d" % (x1, y1, th1, v1, x3, y3, th3, v3, len(tx.path))
            print "---- ---- ---- ---- ---- ---- ----"
            cursor = cnode
        else:
            raise BaseException, "failed"
        traj.append(((u, v), tx))

    print "==== ==== ==== ==== ==== ==== ===="

    #vehicle.draw_temp_car2(screen, cursor, color)
    vehicle.draw_temp_car_allow(screen, cursor, color)
    pygame.display.update()
    print "succeed!"
    return traj

# #---Global Codes----------------------------------------{{{1
# #ENV = load_map("exenv2.svg")
# #ENV = load_map("exenv3.svg")
# ENV = Environment()
# ENV.MIN_X, ENV.MIN_Y = 0, 0
# ENV.MAX_X, ENV.MAX_Y = ENV.map_size
# ENV.MAX_TH = pi
# ENV.dt = 0.2
# ENV.L = 6
# ENV.EPSILON = 10
# ENV.max_iter = 10
# ENV.rrt_cutoff = 200
# ENV.goal_chance = 0.05
# ENV.goal_near = 10.0
# ENV.MAX_ACCEL = 1.0
# ENV.MAX_STEER = pi/8
# ENV.MIN_NSTEP = 1
# ENV.MAX_NSTEP = 40
# ENV.NOSAMPLES = 5
# ENV.V_THRESHOLD = 0.005
# ENV.V_MAX = 4
# ENV.goal_dist = 5
# ENV.goal_orient = pi/4
# ENV.goal_chance = 0.05
# 
# ENV.car = Environment()
# #(s1, _) = ENV.kdt_cg.search_nn((167.083, 65.0))
# #(s1, _) = ENV.kdt_cg.search_nn((37.5, 51.6))
# #(s1, _) = ENV.kdt_cg.search_nn((218.75, 65.0))
# #(s1, _) = ENV.kdt_cg.search_nn((284.16, 41.6))
# (s1, _) = ENV.kdt_cg.search_nn((125.3, 235.83))
# #(d1, _) = ENV.kdt_cg.search_nn((362.5, 51.6))
# 
# #(d1, _) = ENV.kdt_cg.search_nn((170.5, 235.3))
# (d1, _) = ENV.kdt_cg.search_nn((317.16, 236.6))
# 
# 
# #(d1, _) = ENV.kdt_cg.search_nn((332.5, 51.6))
# #ENV.car.src = (s1.data[0], s1.data[1], -pi / 2, 0.0)
# #ENV.car.dest = (d1.data[0], d1.data[1], -pi / 2, 0.0)
# ENV.car.src = (s1.data[0], s1.data[1], 0.0, 0.0)
# #ENV.car.dest = (d1.data[0], d1.data[1], pi / 16, 0.0)
# ENV.car.dest = (d1.data[0], d1.data[1], 0.0, 0.0)
# ENV.car.L = 18.0
# ENV.itr1_csr = -1
# ENV.itr2_csr = -1

if __name__ == '__main__':
    pygame.init()
    SCREEN = pygame.display.set_mode(ENV.map_size) # load screen

    colors = [(255, 238, 170), (233, 198, 175), (136, 170, 0), (0, 0, 0)]
    for index, nodes in enumerate(ENV.obstacles):
        pygame.draw.polygon(SCREEN, colors[index], nodes, 0)
    
    draw_triangles(SCREEN, ENV)
    #car = vehicle.Vehicle3D(ENV.car.src[:3])
    #car.draw(SCREEN)
    #vehicle.draw_temp_car2(SCREEN, ENV.car.src[:3], ENV)
    vehicle.draw_temp_car2(SCREEN, ENV.car.src[:3])

    pygame.display.update()

    result = run_single_rrt_3d()
    (itr1, itr2) = (ENV.itr1_csr, ENV.itr2_csr)
    if result:
        print "plan has been found at (%d, %d)!" % (itr1, itr2)
    else:
        print "plan hasn't been found at (%d, %d)!" % (itr1, itr2)
    while True:
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                pygame.quit()
                sys.exit(1)

# modelines {{{1
# vim:fdm=marker:fdl=0:
# vim:foldtext=getline(v\:foldstart).'...'.(v\:foldend-v\:foldstart):
# #vim:nofen
