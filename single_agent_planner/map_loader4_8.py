# == outline for map loader ==
# 1. load a svg map image
# 2. extract the map's width and height
# 3. extract paths for obstacles and free_space
# 4. triangulate the free_space
# 5. compute the centroid of the triangles
# 6. generate centroid graph and ctmap (centroid triangle map)
#   TODO: It should be networkx.Graph() so that we can compute
#   the shortest path easily.
# 7. this loader can be called through load_map(SVG_MAP_IMAGE)

# == TODO ==
# 1. import kdtree
# 2. add a property for obstacle border for each line seg. of each triangle.
# 3. when generating the centroid graph, wrap it up with kdtree so that when we
#   query one of centroid, we can find the near triangles easily.
# 4. create an api for returning near borders from the near triangles.

# == NOTE ==
# 1. refer tree_2d.py for importing kdtree and using the apis of kdtree such as
#   kdtree.create(), kdtree.add(), kdtree.search_nn(), kdtree.search_nn_dist().
#
# == prerequired packages ==
# svg.path, triangle, pylygon, pygame, kdtree, bounded_priority_queue

import itertools
from bs4 import BeautifulSoup
from svg.path import parse_path
#from svg.path import Line, Path
import triangle
import copy
from numpy import *
#import kdtree
import re
import pygame
from pygame.locals import *
import sys
import heapq
#from collections import defaultdict, deque
#import itertools
import os
import pickle
#import networkx as nx
#import tgraph2 as tgraph
import tgraph3 as tgraph    # KJ: Just for testing (@2017-02-11)
import time

import single_3d_3 as s3d
import car3d as vehicle

from numpy import ones, vstack, isclose
from numpy.linalg import lstsq

from pylygon import Polygon

from lineequ5 import is_intersected, on_borders
from math import fabs, tan, sin, cos, pi, sqrt, atan2

class Environment():
    pass

def get_edge(l): # l: line segment
    n_from = (int(round(l.start.real)), int(round(l.start.imag)))
    n_to = (int(round(l.end.real)), int(round(l.end.imag)))
    return (n_from, n_to)

def get_edge2(l, (cx, cy)): # l: line segment
    n_from = (int(round(l.start.real + cx)), int(round(l.start.imag + cy)))
    n_to = (int(round(l.end.real + cx)), int(round(l.end.imag + cy)))
    return (n_from, n_to)

  
def get_nodes(line_seg_list):
    nodes = []
    (u, v) = get_edge(line_seg_list[0])
    nodes = [u, v]
    for line_seg in line_seg_list[1:]:
        (_, v) = get_edge(line_seg)
        if v != u:
            nodes.append(v)
    return nodes

def get_nodes2(line_seg_list, translate):
    nodes = []
    (u, v) = get_edge2(line_seg_list[0], translate)
    nodes = [u, v]
    for line_seg in line_seg_list[1:]:
        (_, v) = get_edge2(line_seg, translate)
        if v != u:
            nodes.append(v)
    return nodes

# It removes duplicated nodes.
def get_nodes3(line_seg_list, translate):
    nodes, visited = [], set()
    (u, v) = get_edge2(line_seg_list[0], translate)
    nodes.append(u)
    visited.add(u)
    if v not in visited:
        visited.add(v)
        nodes.append(v)
    for line_seg in line_seg_list[1:]:
        (_, v) = get_edge2(line_seg, translate)
        if v != u and v not in visited:
            visited.add(v)
            nodes.append(v)
    return nodes

def draw_path(screen, path):
    #screen.fill((0,0,0))
    for (u, v) in path:
        draw_line_red(screen, u, v)
    #pygame.display.update()

def draw_path2(screen, path):
    #screen.fill((0,0,0))
    for (u, v) in path:
        draw_line_blue(screen, u, v)
    #pygame.display.update()

def draw_path3(screen, path, color):
    for (u, v) in path:
        draw_line(screen, u, v, color)

def draw_line(screen, (x1, y1), (x2, y2), color):
    p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
    pygame.draw.line(screen, color, p1, p2, 2)

def draw_line_red(screen, (x1, y1), (x2, y2), update=False):
    global ENV
    DIR_COLOR = (255, 0, 0)    # red
    p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
    pygame.draw.line(screen, DIR_COLOR, p1, p2, 3)
    if update:
        pygame.display.update()

def draw_line_blue(screen, (x1, y1), (x2, y2), update=False):
    global ENV
#    DIR_COLOR = (0, 0, 255)    # blue
#    DIR_COLOR = (0, 191, 255)    # deep sky blue
    DIR_COLOR = (30, 144, 255)    # dodger blue
    p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
    pygame.draw.line(screen, DIR_COLOR, p1, p2, 3)
    if update:
        pygame.display.update()

def draw_path_3d(ENV, screen, car, tree, update=False):
    #global ENV
    v1, _ = tree.path[0]
    #for (u, v) in tree.path:
    for i, (u, v) in enumerate(tree.path):
        assert (v1 == u)
        v1 = v
        lsteps = int(v.tweight / ENV.dt)
        assert (isclose(lsteps, v.tweight / ENV.dt))
        #draw_line_3d(screen, car, u.nvalue, v.ctrl, v.tweight/ENV.dt, update)
        (x1, y1), (x2, y2) = u.nvalue[:2], v.nvalue[:2]
        print "path:%d:%d((%05.3f,%05.3f),(%05.3f,%05.3f))" % (i,lsteps,x1,y1,x2,y2)
        draw_line_3d(screen, car, u.nvalue, v.ctrl, lsteps, update)
    if update:
        pygame.display.update()

def draw_line_3d(ENV, screen, car, state, ctrl, lsteps, update = False):
    #global ENV
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
        (v2, terminate) = enforce_velocity(v1, DIR, accel, ENV.dt)
        (x2, y2, th2), passed = enforce_bounds_2d((x2, y2, th2))
#        p1, p2 = (int(x1), ENV.MAX_Y - int(y1)), (int(x2), ENV.MAX_Y - int(y2))
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        pygame.draw.line(screen, DIR_COLOR, p1, p2)
        print "=>%i:((%04.2f,%04.2f),(%04.2f,%04.2f))"%(i,x1,y1,x2,y2)
        (x1, y1, th1, v1) = (x2, y2, th2, v2)
        if terminate:
            break
    if update:
        pygame.display.update()
    #print_str += "(%05.3f,%05.3f))" % (x1, y1)
#    print print_str


def get_centroids(B):
    c_list = []
    for (iv1, iv2, iv3) in list(B['triangles']):
        (x1, y1) = B['vertices'][iv1]
        (x2, y2) = B['vertices'][iv2]
        (x3, y3) = B['vertices'][iv3]
        (cx, cy) = ((x1 + x2 + x3) / 3, (y1 + y2 + y3) / 3)
        c_list.append((cx, cy))
    return c_list  

def get_centroid(t1, t2, t3):
    ((x1, y1), (x2, y2), (x3, y3)) = (t1, t2, t3)
    return ((x1 + x2 + x3) / 3, (y1 + y2 + y3) / 3)

def generate_centroid_graph_weighted(vtriangles):
    tg = tgraph.Graph()
#   .. print "|vtriangles|:%d" % len(vtriangles)
    for ((ti, ci, bi), (tj, cj, bj)) in itertools.combinations(vtriangles, 2):
        try:
            tg.add_node(ci, ti)
        except KeyError:
            pass
        tg.add_borders(ci, bi)

        try:
            tg.add_node(cj, tj)
        except KeyError:
            pass
        tg.add_borders(cj, bj)
    return tg

def distance_2d(p1, p2):
    (x1, y1) = (list(p1)[0], list(p1)[1])
    (x2, y2) = (list(p2)[0], list(p2)[1])
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

def get_m_c(points):
    x_coords, y_coords = zip(*points)
    A = vstack( [x_coords, ones(len(x_coords))] ).T
    m, c = lstsq( A, y_coords )[0]
    return (m, c)

def get_midpoint(points):
    (x1, y1), (x2, y2) = points
    return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)

def on_border(points1, points2):
    m, c = get_m_c(points1)
    (mx, my) = get_midpoint(points2)
    if isclose(my, m * mx + c):
        return True
    else:
        return False

def clear_and_draw_base(screen, ENV, colors):
    screen.fill((0, 0, 0))
    for index, nodes in enumerate(ENV.obstacles):
        pygame.draw.polygon(screen, colors[index % (len(colors) - 1)], nodes, 0)

    #ENV.tg.draw_triangles(screen)
    #ENV.tg.draw_triangles2(screen)

    ENV.tg.draw_triangles3(screen)

def mouse_clicked():
    if pygame.mouse.get_pressed()[0]:
        pressed = True
    else:
        pressed = False
    while pressed:
        for e in pygame.event.get():
            if e.type == MOUSEBUTTONUP and e.button == 1: #LEFT
                (cx, cy) = pygame.mouse.get_pos()
                return (True, (cx, cy))
    return (False, (None, None))

def mouse_clicked_and_pressed():
    return pygame.mouse.get_pressed()[0]

def key_pressed(e, key_value):
    return e.type == KEYDOWN and e.key == key_value

def handle_quit_event(timeout = 0.5):
    tick = time.time()
    while time.time() - tick < timeout:
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                pygame.quit()
                sys.exit(0)
        pygame.display.update()
        time.sleep(0.001)

def check_conflict(path1, path2):
    #p1 = copy.deepcopy(path1)
    #p2 = copy.deepcopy(path2)
    p1, p2 = [], []
    for (x1, y1), (x2, y2) in path1:
        p1.append(((x1, y1), (x2, y2)))
    for (x1, y1), (x2, y2) in path2:
        p2.append(((x1, y1), (x2, y2)))
    #p1 = path1[:]
    #p2 = path2[:]
    longer = max(len(p1), len(p2))
    diff = max(len(p1), len(p2)) - min(len(p1), len(p2))
    for i in range(diff):
        if len(p1) < len(p2):
            p1.append((p1[-1][1],p1[-1][1]))
        elif len(p1) > len(p2):
            p2.append((p2[-1][1],p2[-1][1]))

#    while len(p1) != len(p2):
#        if len(p1) < len(p2):
#            p1.apppend((p1[-1][1],p1[-1][1]))
#        elif len(p1) < len(p2):
#            p2.apppend((p2[-1][1],p2[-1][1]))
    for i in range(len(p1)):
        (ca1, ca2), (cb1, cb2) = p1[i], p2[i]
        if (ca1 == cb2 and ca2 == cb1) or ca2 == cb2:
            return True, i
    return False, None

def check_conflict_id(path1, path2):
    p1, p2 = [], []
    for (u1, v1) in path1:
        p1.append((u1, v1))
    for (u2, v2) in path2:
        p2.append((u2, v2))
    longer = max(len(p1), len(p2))
    diff = max(len(p1), len(p2)) - min(len(p1), len(p2))
    for i in range(diff):
        if len(p1) < len(p2):
            p1.append((p1[-1][1],p1[-1][1]))
        elif len(p1) > len(p2):
            p2.append((p2[-1][1],p2[-1][1]))
    for i in range(len(p1)):
        (ca1, ca2), (cb1, cb2) = p1[i], p2[i]
        if (ca1 == cb2 and ca2 == cb1) or ca2 == cb2:
            return True
    return False

def get_local_goal(step, p1, p2):
    ls1, ls2 = p1[step][0], p2[step][0]
#    print "glg, #1:%s, %s" % (repr(p1[step-1]), repr(p2[step-1]))
#    print "glg, #2:%s, %s" % (repr(p1[step]), repr(p2[step]))
#    print "glg, #3:%s, %s" % (repr(p1[step+1:step+3]), repr(p2[step+1:step+3]))
#    raise BaseException
    if p1[step][1] == p2[step][1]:
        for i in range(step, len(max(len(p1[:]), len(p2[:])))):
            if p1[i][1] != p2[i][1]:
                return p1[i][1], p2[i][1]
        raise BaseException
    elif p1[step][0] == p2[step][1] and p1[step][1] == p2[step][0]:
        if p1[step][1] != p2[step][1]:
            return p1[step][1], p2[step][1]
    raise BaseException

def compute_coop_astar(ENV, (ls1, ls2), (t, t2)):
    return res, p1, p2

def compute_bfs(ENV, (ls1, ls2), (lg1, lg2)):
    print "compute_bfs((%s, %s), (%s, %s))" % (repr(ls1), repr(ls2), repr(lg1), repr(lg2))
    #h, came_from = deque(), {}
    #h.apend((ls1, ls2))
    h, came_from, found, path = [], {}, False, []
    heapq.heappush(h, (0, (ls1, ls2)))
    came_from[(ls1, ls2)] = None
    while len(h) > 0:
        depth, (p1, p2) = heapq.heappop(h)
        if p1 == lg1 and p2 == lg2:
            found = True
            break
        neighbors1 = ENV.tg.neighbors(p1, True)
        neighbors2 = ENV.tg.neighbors(p2, True)

        #neighbors1 = ENV.cg[p1].keys() + [p1]
        #neighbors2 = ENV.cg[p2].keys() + [p2]
        for (p3, p4) in itertools.product(neighbors1, neighbors2):
            if (p3, p4) not in came_from:
                heapq.heappush(h, (depth + 1, (p3, p4)))
                came_from[(p3, p4)] = (p1, p2)
    if found:
        print "found:%s, depth:%d, p:%s" % (repr(found), depth, repr(came_from))
        cursor = (lg1, lg2)
        # for safety, we can check if the iteration exceeds |came_from|.
        itr = 0
        while cursor != (ls1, ls2):
            path = [(came_from[cursor], cursor)] + path
            cursor = came_from[cursor]
            if itr < len(came_from):
                itr += 1
            else:
                raise BaseException, path
        print "path:",path
    else:
        print "found:%s, |came_from|:%d" % (repr(found), len(came_from))
    return found, path

def compute_bfs2(G, (ls1, ls2), (t1, t2), (pa1, pa2), (dista1, dista2)):
    dist_s = dista1[G.cidmap[ls1]] + dista2[G.cidmap[ls2]]
    #print "compute_bfs2 #1, dist_s:%05.1f" % dist_s
    h, came_from, found, path, lg1, lg2, = [], {}, False, [], None, None
    ls1_id, ls2_id = G.cidmap[ls1], G.cidmap[ls2]
    tg1_id, tg2_id = G.cidmap[t1], G.cidmap[t2]
    heapq.heappush(h, (0, (ls1_id, ls2_id)))
    came_from[(0, ls1_id, ls2_id)] = (-1, -1, -1)
    ldepth = 0
    #print "compute_bfs2,dist_s:%04.1f, (%d,%d), (%d,%d), |h|:%d" % (dist_s, ls1_id, ls2_id, tg1_id, tg2_id, len(h))
    itr = 0
    while len(h) > 0:
        depth, (p1, p2) = heapq.heappop(h)
        dist = dista1[p1] + dista2[p2]
        #if itr % 100 == 0:
        #    print "itr:%d, |h|:%d, depth:%d, |came_from|:%d" % (itr, len(h), depth, len(came_from))
        if dist < dist_s:
            path1 = G.get_path(pa1, p1, tg1_id)
            path2 = G.get_path(pa2, p2, tg2_id)
            if not check_conflict_id(path1, path2):
                found = True
                lg1, lg2, ldepth = p1, p2, depth
                break
            #print "=> itr:%d, dist:%04.1f < dist_s:%04.1f, |h|:%d, |came_from|:%d" % (itr, dist, dist_s, len(h), len(came_from))
        else:
            neighbors1 = G.neighbors_id(p1, True)
            neighbors2 = G.neighbors_id(p2, True)
            for (p3, p4) in itertools.product(neighbors1, neighbors2):
                if (depth + 1, p3, p4) not in came_from:
                    heapq.heappush(h, (depth + 1, (p3, p4)))
                    came_from[(depth + 1, p3, p4)] = (depth, p1, p2)
        itr += 1

    #print "found:%s, ldepth:%d, |came_from|:%d, itr:%d, lg1:%d, lg2:%d" % (repr(found), ldepth, len(came_from), itr, lg1, lg2)
    if found:
        #print "came_from:"
        #for (i, u1, u2), (j, v1, v2) in came_from.iteritems():
        #    print "(ld0:%d, %02d, %02d), (ld1:%d, %02d, %02d)" % (i, u1, u2, j, v1, v2)
        #print "---- ----"
        cursor = (ldepth, lg1, lg2)
        #print "cursor:(%d, %02d, %02d) != (0, %02d, %02d)" % (ldepth, lg1, lg2, ls1_id, ls2_id)
        while cursor != (0, ls1_id, ls2_id):
            (_, u1, u2), (__, v1, v2) = came_from[cursor], cursor
            #print "came_from[cursor]:(%d, %02d, %02d), (%d, %02d, %02d)" % (_, u1, u2, __, v1, v2)
            #path = [(came_from[cursor], cursor)] + path
            path = [((u1, u2), (v1, v2))] + path
            cursor = came_from[cursor]
        for i in range(max(len(path1), len(path2))):
            if i >= len(path1):
                u1, v1 = path1[-1][1], path1[-1][1]
            else:
                u1, v1 = path1[i]
            if i >= len(path2):
                u2, v2 = path2[-1][1], path2[-1][1]
            else:
                u2, v2 = path2[i]
            path = path + [((u1, u2), (v1, v2))]
    return path

def compute_bfs3(G, (ls1, ls2), (t1, t2), (pa1, pa2), (dista1, dista2)):
    dist_s = dista1[G.cidmap[ls1]] + dista2[G.cidmap[ls2]]
    h, came_from, found, path, lg1, lg2, = [], {}, False, [], None, None
    ls1_id, ls2_id = G.cidmap[ls1], G.cidmap[ls2]
    tg1_id, tg2_id = G.cidmap[t1], G.cidmap[t2]
    heapq.heappush(h, (0, (ls1_id, ls2_id)))
    came_from[(ls1_id, ls2_id)] = (-1, -1)
    ldepth = 0
    itr = 0
    while len(h) > 0:
        depth, (p1, p2) = heapq.heappop(h)
        dist = dista1[p1] + dista2[p2]
        if dist < dist_s:
            path1 = G.get_path(pa1, p1, tg1_id)
            path2 = G.get_path(pa2, p2, tg2_id)
            if not check_conflict_id(path1, path2):
                found = True
                lg1, lg2, ldepth = p1, p2, depth
                break
        else:
            neighbors1 = G.neighbors_id(p1, True)
            neighbors2 = G.neighbors_id(p2, True)
            for (p3, p4) in itertools.product(neighbors1, neighbors2):
                if (p3, p4) not in came_from:
                    heapq.heappush(h, (depth + 1, (p3, p4)))
                    came_from[(p3, p4)] = (p1, p2)
        itr += 1
    #print "found:%s, ldepth:%d, |came_from|:%d, itr:%d, lg1:%d, lg2:%d" % (repr(found), ldepth, len(came_from), itr, lg1, lg2)
    if found:
        #print "came_from:"
        #for (u1, u2), (v1, v2) in came_from.iteritems():
        #    print "(%02d, %02d), (%02d, %02d)" % (u1, u2, v1, v2)
        #print "---- ----"
        cursor = (lg1, lg2)
        #print "ldepth:%d, cursor:(%02d, %02d) != (%02d, %02d)" % (ldepth, lg1, lg2, ls1_id, ls2_id)
        itr = 0
        while cursor != (ls1_id, ls2_id):
            (u1, u2), (v1, v2) = came_from[cursor], cursor
            #print "came_from[cursor]:(%02d, %02d), (%02d, %02d)" % (u1, u2, v1, v2)
            path = [((u1, u2), (v1, v2))] + path
            cursor = came_from[cursor]
            if itr < len(came_from):
                itr += 1
            else:
                raise BaseException, "itr:%d exceeds |came_from|:%d" % (itr, len(came_from))
                break
        for i in range(max(len(path1), len(path2))):
            if i >= len(path1):
                u1, v1 = path1[-1][1], path1[-1][1]
            else:
                u1, v1 = path1[i]
            if i >= len(path2):
                u2, v2 = path2[-1][1], path2[-1][1]
            else:
                u2, v2 = path2[i]
            path = path + [((u1, u2), (v1, v2))]
    return path

def convert_to_midpath3d(mpath):
    mpath3d = []
    for i, ((x1, y1), (x2, y2)) in enumerate(mpath):
        if i == 0:
            th1 = atan2(y2 - y1, x2 - x1)
            _, (x3, y3) = mpath[i+1]
            th2 = atan2(y3 - y2, x3 - x2)
            mpath3d.append(((x1, y1, th1), (x2, y2, th2)))
        elif i == len(mpath) - 1:
            _, (__, ___, th1) = mpath3d[-1]
            mpath3d.append(((x1, y1, th1), (x2, y2, th1)))
        else:
            _, (__, ___, th1) = mpath3d[-1]
            _, (x3, y3) = mpath[i+1]
            th2 = atan2(y3 - y2, x3 - x2)
            mpath3d.append(((x1, y1, th1), (x2, y2, th2)))
    #mpath3d = path_normalize(ENV, mpath3d)
    #mpath3d = normalize2(mpath3d)
    (x1, y1, th1), p1 = mpath3d[0]
    p2, (x2, y2, th2) = mpath3d[-1]
    mpath3d = [((x1, y1, th1, 0.0), p1)] + mpath3d[1:]
    mpath3d = mpath3d[:-1] + [(p2, (x2, y2, th2, 0.0))]
    return mpath3d

def convert_to_midpath3d2(mpath, th0):
    mpath3d = [], Forward, Stop = True, False
    for i, ((x1, y1), (x2, y2)) in enumerate(mpath):
        if i == 0:
            if (x1, y1) == (x2, y2):
                mpath3d.append(((x1, y1, th0),(x2, y2, th0)))
                Stop = True
                continue
            th1 = atan2(y2 - y1, x2 - x1)
            _, (x3, y3) = mpath[i+1]
            d_th0 = fabs(th0 - th1)
            #if (7.0/8) * pi <= d_th0 and d_th0 <= (9.0/8) * pi:
            if (7.0/8) * pi <= d_th0:
                Forward = False
                th1 = th0
                th2 = atan2(y2 - y3, x2 - x3)
                if (x2, y2) == (x3, y3):
                    th2 = th0
            else:
                th2 = atan2(y3 - y2, x3 - x2)
            mpath3d.append(((x1, y1, th1), (x2, y2, th2)))

def convert_to_midpath3d_reverse(mpath):
    mpath3d = []
    for i, ((x1, y1), (x2, y2)) in enumerate(mpath):
        if i == len(mpath) - 1:
            _, (__, ___, th1) = mpath3d[-1]
            mpath3d.append(((x1, y1, th1), (x2, y2, th1)))
        else:
            th1 = atan2(y1 - y2, x1 - x2)
            _, (x3, y3) = mpath[i+1]
            th2 = atan2(y2 - y3, x2 - x3)
            mpath3d.append(((x1, y1, th1),(x2, y2, th2)))
    (x1, y1, th1), p1 = mpath3d[0]
    p2, (x2, y2, th2) = mpath3d[-1]
    mpath3d = [((x1, y1, th1, 0.0), p1)] + mpath3d[1:]
    mpath3d = mpath3d[:-1] + [(p2, (x2, y2, th2, 0.0))]
    return mpath3d

def path_normalize(ENV, mpath):
    mpath_full = []
    for (x1, y1, th1), (x2, y2, th2) in mpath:
        dist = distance_2d((x1, y1), (x2, y2))
        th = atan2(y2 - y1, x2 - x1)
        print "(%04.1f, %04.1f, %03.2f), (%04.1f, %04.1f, %03.2f), dist:%03.2f, diff:%03.2f" % (x1, y1, th1, x2, y2, th2, dist, fabs(th1 - th2))
        mpath3d = []
        cx, cy, cth = x1, y1, th1
        #dist_statck = []
        #while dist >= ENV.max_dist:
        #    dist_stack.append(ENV.max_dist)
        #    dist -= ENV.max_dist
        #while len(dist_stack) > 0:
        #    dist_e = dist_stack.pop(-1)
        #    x3 = cx + dist_e * c
        while dist >= ENV.max_dist:
            x3 = cx + ENV.max_dist * cos(th)
            y3 = cy + ENV.max_dist * sin(th)
            mpath3d.append(((cx, cy, cth), (x3, y3, th)))
            cx, cy, cth = x3, y3, th
            dist -= ENV.max_dist
        if dist > 0:
            mpath3d.append(((cx, cy, cth), (x2, y2, th2)))
        else:
            p, (x2, y2, _) = mpath3d[-1]
            mpath3d = mpath3d[:-1] + [(p, (x2, y2, th2))]
        for (xi, yi, thi), (xj, yj, thj) in mpath3d:
            disti = distance_2d((xi, yi), (xj, yj))
            print "=>(%04.1f, %04.1f, %03.2f), (%04.1f, %04.1f, %03.2f), dist:%03.2f, diff:%03.2f" % (xi, yi, thi, xj, yj, thj, disti, fabs(thi - thj))
        mpath_full = mpath_full + mpath3d
    return mpath_full

def get_centroid_2d(plist):
#    nop = float(len(plist))
    cx = float(sum([p[0] for p in plist])) / float(len(plist))
    cy = float(sum([p[1] for p in plist])) / float(len(plist))
    return (cx, cy)

def normalize2(mpath):
    print "==== normalize2:, before normalize, |mpath|:%d ====" % len(mpath)
    for p1, p2 in mpath:
        diff = p1[2] - p2[2]
        pstr = "th_diff:%03.2f," % diff
        pstr += "(%04.1f,%04.1f,%03.2f)," % (p1[0], p1[1], p1[2])
        pstr += "(%04.1f,%04.1f,%03.2f)" % (p2[0], p2[1], p2[2])
        print pstr
    mpath_full = []
    #mpath_temp = copy.deepcopy(mpath)
    mpath_temp = mpath[:]
    while len(mpath_temp) > 0:
        (p1, p2) = mpath_temp.pop(0)
        diff = p1[2] - p2[2]
        if fabs(diff) >= pi / 2.0:
            if len(mpath_temp) == 0:
                raise BaseException, "This is not possible since p1[2] == p2[2] if it was the last tuple of mpath."
            (_, p3) = mpath_temp.pop(0)
            (cx, cy) = get_centroid_2d([p1, p2, p3])
            cth = atan2(p3[1] - cy, p3[0] - cx)
            th0 = atan2(cy - p1[1], cx - p1[0])
            if len(p1) == 3:
                mpath_full.append(((p1[0], p1[1], th0),(cx, cy, cth)))
                mpath_full.append(((cx, cy, cth),p3))
            elif len(p1) == 4:
                mpath_full.append(((p1[0], p1[1], th0, p1[3]),(cx, cy, cth)))
                mpath_full.append(((cx, cy, cth),p3))
            else:
                raise BaseException
        else:
            mpath_full.append((p1, p2))

    print "==== after normalize, |mpath|:%d ====" % (len(mpath_full))
    for p1, p2 in mpath_full:
        diff = p1[2] - p2[2]
        pstr = "th_diff:%03.2f," % diff
        pstr += "(%04.1f,%04.1f,%03.2f)," % (p1[0], p1[1], p1[2])
        pstr += "(%04.1f,%04.1f,%03.2f)" % (p2[0], p2[1], p2[2])
        print pstr

    return mpath_full

def load_map(svg_map_file):
    # 0. instantiate the environment class
    # 1. load SVG_MAP
    # 2. import display related modules like pygame, etc
    # 3. initialize pygame with its Surface screen
    # 4. find all group in SVG_MAP
    # 5. find all obstacles from group
    # 6. find free_space from group
    # 7. draw obstacle
    # 8. (cg, ctmap) = load_map(SVG_MAP_IMAGE)
    # 9. draw traiangles from cg and ctmap
    # 10. draw centroids and centroids path

    # 0. instantiate the environment class
    ENV = Environment()

    # 1. load SVG_MAP
    #soup = BeautifulSoup(open("drawing3.svg", "r"), "lxml-xml")
    soup = BeautifulSoup(open(svg_map_file, "r"), "lxml-xml")
    # circles = soup.g.find_all('circle')
    # str(circles[0]['style'])
    # import re
    # color = re.findall('fill:(.*?);', style)

    ENV.map_size = (int(soup.svg['width']), int(soup.svg['height']))

    # 4. find all group in SVG_MAP
    paths = soup.g.find_all('path')

#    transform = soup.g['transform']

    ptr = r"([-+]?\d*\.*\d+)"
    try:
        res = re.findall(ptr, soup.g['transform'])
    except KeyError:
        res = []
    if len(res) == 2:
        cx, cy = float(res[0]), float(res[1])
    elif len(res) == 1:
        cx, cy = float(res[0]), 0.0
    else:
        cx, cy = 0.0, 0.0
    
    # 5. find all obstacles from group
    # 6. find free_space from group
    (ENV.obstacles, ENV.free_space) = ([], [])
    for index, path in enumerate(paths):
        data = parse_path(path['d'])
        if path.title == None:  # then it is an obstacle
            #nodes = get_nodes(data)
            #ENV.obstacles.append(get_nodes(data))
            #nodes = get_nodes2(data, (cx, cy))
            #ENV.obstacles.append(get_nodes2(data, (cx, cy)))
            nodes = get_nodes3(data, (cx, cy))
            ENV.obstacles.append(get_nodes3(data, (cx, cy)))
        else: # then it is free_space
            #ENV.fs_nodes = get_nodes(data)
            #free_line_seg_list = [get_edge(line_seg) for line_seg in data]
            #ENV.fs_nodes = get_nodes2(data, (cx, cy))
            ENV.fs_nodes = get_nodes3(data, (cx, cy))
            free_line_seg_list = [get_edge2(line_seg, (cx, cy)) for line_seg in data]
            ENV.fsdict_vkeys = {}
            for i, node in enumerate(ENV.fs_nodes):
                ENV.fsdict_vkeys[node] = i
            ENV.fs_segments = []
            for (u, v) in free_line_seg_list:
                ENV.fs_segments.append((ENV.fsdict_vkeys[u], ENV.fsdict_vkeys[v]))

    # 8. (cg, ctmap) = load_map(SVG_MAP_IMAGE)
    # 8.1. Create a graph G which is formatted in numpy.array.
    #    It should have G['vertices'] = array(())
    #    It also should have G['segments'] = array(())
    A = {}
    A['vertices'] = array(ENV.fs_nodes)
    A['segments'] = array(ENV.fs_segments)

    # 8.2. TA = triangulate(G, 'pD')
    B = triangle.triangulate(A, 'pDa1000')
    dict_keyvs = {}
    vtriangles = []
    for i, node in enumerate(B['vertices']):
        dict_keyvs[i] = node
        
    # 8.3. compute centroids from TA
    fseg_set = {((float(a), float(b)), (float(c), float(d))) for ((a, b), (c, d)) in free_line_seg_list}
    vtriangles = []
    for (u, v, w) in B['triangles']:
        (t1, t2, t3) = (dict_keyvs[u], dict_keyvs[v], dict_keyvs[w])
        (cx, cy) = get_centroid(t1, t2, t3)
        fsegs1 = set([((t1x, t1y), (t2x, t2y)) for (t1x, t1y), (t2x, t2y) in itertools.combinations([t1, t2, t3], 2)])
        fcom1 = fseg_set.intersection(fsegs1)
        fsegs2 = set([((t2x, t2y), (t1x, t1y)) for (t1x, t1y), (t2x, t2y) in itertools.combinations([t1, t2, t3], 2)])
        fcom2 = fseg_set.intersection(fsegs2)
        fsegs_reduced = fcom1.union(fcom2)
        fseg_set = fseg_set - fsegs_reduced
        vtriangles.append([(tuple(t1), tuple(t2), tuple(t3)), get_centroid(t1, t2, t3), list(fsegs_reduced)])
        #vtriangles.append([(tuple(t1), tuple(t2), tuple(t3)), get_centroid(t1, t2, t3)])

    # 8.5. get neighbors of each triangle in TA; generate centroid graph
    ENV.tg = generate_centroid_graph_weighted(vtriangles)
    ENV.tg.kdt_cg.rebalance()

    for vid, ((cx, cy), p_set) in ENV.tg.V.iteritems():
        for (p1, p2) in itertools.combinations(p_set, 2):
            for (p3, p4) in fseg_set:
                (ps1, ps2) = ([p3, p4], [p1, p2])
                if on_borders(ps1, ps2) or on_border(ps1, ps2):
                    ENV.tg.add_border((cx, cy), (p1, p2))
                    break

    ENV.fseg_set = fseg_set
    return ENV

if __name__ == '__main__':
    conf_file = "./_map_loader4_8.cfg"
    if os.path.isfile(conf_file):
        conf_file_exist = True
        input_ds = open(conf_file, 'rb')
        conf_data = pickle.load(input_ds)
        input_ds.close()
        ENV = load_map(conf_data['map_file'])

        # for car 1
        s, t, path = conf_data['s1'], conf_data['t1'], conf_data['path1']
        tg1 = path[-1][0]
        patha, dista = ENV.tg.compute_shortest_reverse(tg1)
        mpath = ENV.tg.get_shortest_midpath(s, t, None)

        # for car 2
        s2, t2, path2 = conf_data['s2'], conf_data['t2'], conf_data['path2']
        tg2 = path2[-1][0]
        patha2, dista2 = ENV.tg.compute_shortest_reverse(tg2)
        mpath2 = ENV.tg.get_shortest_midpath(s2, t2, None)

        # for car 3
        s3, t3, path3 = conf_data['s3'], conf_data['t3'], conf_data['path3']
        tg3 = path3[-1][0]
        patha3, dista3 = ENV.tg.compute_shortest_reverse(tg3)
        mpath3 = ENV.tg.get_shortest_midpath(s3, t3, None)
        mpath3_3d = convert_to_midpath3d(mpath3)

        # for car 4
        s4, t4, path4 = conf_data['s4'], conf_data['t4'], conf_data['path4']
        tg4 = path4[-1][0]
        patha4, dista4 = ENV.tg.compute_shortest_reverse(tg4)
        mpath4 = ENV.tg.get_shortest_midpath(s4, t4, None)
        mpath4_3d = convert_to_midpath3d(mpath4)

        # for car 5
        s5, t5, path5 = conf_data['s5'], conf_data['t5'], conf_data['path5']
        tg5 = path5[-1][0]
        patha5, dista5 = ENV.tg.compute_shortest_reverse(tg5)
        mpath5 = ENV.tg.get_shortest_midpath(s5, t5, None)
        mpath5_3d = convert_to_midpath3d(mpath5)

        # for car 6
        s6, t6, path6 = conf_data['s6'], conf_data['t6'], conf_data['path6']
        tg6 = path6[-1][0]
        patha6, dista6 = ENV.tg.compute_shortest_reverse(tg6)
        mpath6 = ENV.tg.get_shortest_midpath(s6, t6, None)
        mpath6_3d = convert_to_midpath3d(mpath6)
        normal = False
    else:
        conf_file_exist = False
        map_file = "exenv9.svg"
        conf_data = {}
        conf_data['map_file'] = map_file
        s, t, path, mpath = None, None, [], []
        s2, t2, path2, mpath2 = None, None, [], []
        s3, t3, path3, mpath3 = None, None, [], []
        s4, t4, path4, mpath4 = None, None, [], []
        s5, t5, path5, mpath5 = None, None, [], []
        s6, t6, path6, mpath6 = None, None, [], []
        traj1, traj2, traja, trajb = [], [], [], []
        traj3, traj4, traj5, traj6 = [], [], [], []
        normal = True
        ENV = load_map(conf_data['map_file'])

    ENV.MIN_X, ENV.MIN_Y = 0, 0
    ENV.MAX_X, ENV.MAX_Y = ENV.map_size
    ENV.MAX_TH = pi
    ENV.dt = 0.05
    #ENV.dt = 0.1
    #ENV.dt = 0.2
    #ENV.L = 6
    #ENV.L = 18.0
    ENV.L = 12.0
    #ENV.EPSILON = 10
    ENV.EPSILON = 4
    ENV.max_iter = 20
    #ENV.rrt_cutoff = 200
    #ENV.rrt_cutoff = 200
    ENV.rrt_cutoff = 500
    ENV.goal_chance = 0.05
    ENV.goal_near = 10.0
    #ENV.goal_near = 7.0
    #ENV.MAX_ACCEL = 1.0
    ENV.MAX_ACCEL = 0.3
    #ENV.MAX_STEER = pi/6
    #ENV.MAX_STEER = pi/5
    ENV.MAX_STEER = pi/4
    ENV.MIN_NSTEP = 1
    ENV.MAX_NSTEP = 40 * 4
    ENV.NOSAMPLES = 5
    ENV.V_THRESHOLD = 0.001
    #ENV.V_THRESHOLD = 0.005
    #ENV.V_MAX = 4
    #ENV.V_MAX = 2
    ENV.V_MAX = 0.5
    ENV.goal_dist = 5
    ENV.goal_orient = pi/4
    ENV.goal_chance = 0.05
    ENV.max_dist = 10.0

    ENV.DODGER_BLUE = (30, 144, 255)
    ENV.ORANGE = (255, 165, 0)
    ENV.PLUM = (221, 160, 221)
    ENV.GREEN = (50, 255, 50)

    ENV.car = Environment()
    #(x1, y1), (x2, y2) = mpath[0]
    #(_, __, th1) = s3d.add_theta((x1, y1, 0.0), (x2, y2))
    #ENV.car.src = (x1, y1, th1, 0.0)
    
    #(x3, y3), (x4, y4) = mpath[-1]
    #(_, __, th4) = s3d.add_theta((x3, y3, 0.0), (x4, y4))
    #ENV.car.dest = (x4, y4, th4, 0.0)
    ENV.car.L = 12.0
    ENV.itr1_csr = -1
    ENV.itr2_csr = -1

    # 3. initialize pygame with its Surface screen
    pygame.init()
    SCREEN = pygame.display.set_mode(ENV.map_size)
    colors = [(255, 238, 170), (233, 198, 175), (136, 170, 0), (0, 0, 0)]
    clear_and_draw_base(SCREEN, ENV, colors)
    #vehicle.draw_temp_car2(SCREEN, ENV.car.src[:3])
    pygame.display.update()
#    tpoint = (362.5, 51.6)
#    print get_near_triangle(tpoint, ENV)
#    get_triangle_and_border(tpoint, ENV)

    cpath, cpath2 = [], []
    traj1, traj2, traja, trajb = [], [], [], []
    traj3, traj4, traj5, traj6 = [], [], [], []
    while True:
        # K_c: clean all path, then normal display for mout press
        # K_s: ready for input point, after two mouse pressed, draw the path
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                pygame.quit()
                sys.exit(0)

            if normal:
                if key_pressed(e, K_s):
                    s, t, s2, t2 = None, None, None, None
                    s3, t3, s4, t4, normal  = None, None, None, None, False
                    s5, t5, s6, t6 = None, None, None, None
                    print "path drawing mode. click start postion."
                    clear_and_draw_base(SCREEN, ENV, colors)
                elif key_pressed(e, K_l):
                    if conf_file_exist:
                        input_ds = open(conf_file, 'rb')
                        conf_data = pickle.load(input_ds)
                        input_ds.close()
                        # for car 1
                        s, t, path = conf_data['s1'], conf_data['t1'], conf_data['path1']
                        mpath = ENV.tg.get_shortest_midpath(s, t, SCREEN)
                        # for car 2
                        s2, t2, path2 = conf_data['s2'], conf_data['t2'], conf_data['path2']
                        mpath2 = ENV.tg.get_shortest_midpath(s2, t2, SCREEN)
                        # for car 3
                        s3, t3, path3 = conf_data['s3'], conf_data['t3'], conf_data['path3']
                        mpath3 = ENV.tg.get_shortest_midpath(s3, t3, SCREEN)
                        # for car 4
                        s4, t4, path4 = conf_data['s4'], conf_data['t4'], conf_data['path4']
                        mpath4 = ENV.tg.get_shortest_midpath(s4, t4, SCREEN)
                        # for car 5
                        s5, t5, path5 = conf_data['s5'], conf_data['t5'], conf_data['path5']
                        mpath5 = ENV.tg.get_shortest_midpath(s5, t5, SCREEN)
                        # for car 6
                        s6, t6, path6 = conf_data['s6'], conf_data['t6'], conf_data['path6']
                        mpath6 = ENV.tg.get_shortest_midpath(s6, t6, SCREEN)
                        normal = False
                        print "configuration is loaded. Press 1, 2, 3, 4 or 0 (t and d later)"
                    else:
                        print "There is no configuration file."
                        s, t, s2, t2 = None, None, None, None
                        s3, t3, s4, t4, normal  = None, None, None, None, False
                        s5, t5, s6, t6 = None, None, None, None
                        print "path drawing mode. click start postion."
                        clear_and_draw_base(SCREEN, ENV, colors)
                elif mouse_clicked_and_pressed():
                    (cx, cy) = pygame.mouse.get_pos()
                    clear_and_draw_base(SCREEN, ENV, colors)
                    ENV.tg.draw_borders(SCREEN, (cx, cy), 70)
                    ptr_str = "(%05.3f, %05.3f)" % (cx, cy)
                    print ptr_str + str(ENV.tg.get_near_triangle((cx, cy)))
            else:
                if key_pressed(e, K_c):
                    clear_and_draw_base(SCREEN, ENV, colors)
                    normal, path, path2, cpath = True, [], [], []
                    path3, path4, cpath2 = [], [], []
                    path5, path6 = [], []
                    print "normal mode"
                elif key_pressed(e, K_1):
                    clear_and_draw_base(SCREEN, ENV, colors)
                    if cpath:
                        for (pa1, pb1), (pa2, pb2) in cpath:
                            draw_line_red(SCREEN, ENV.tg.V[pa1][0], ENV.tg.V[pa2][0])
                            #draw_line_blue(SCREEN, ENV.tg.V[pb1][0], ENV.tg.V[pb2][0])
                    else:
                        #draw_path(SCREEN, path)
                        draw_path(SCREEN, mpath)
                    print "draw path 1"
                elif key_pressed(e, K_2):
                    clear_and_draw_base(SCREEN, ENV, colors)
                    if cpath:
                        for (pa1, pb1), (pa2, pb2) in cpath:
                            #draw_line_red(SCREEN, ENV.tg.V[pa1][0], ENV.tg.V[pa2][0])
                            draw_line_blue(SCREEN, ENV.tg.V[pb1][0], ENV.tg.V[pb2][0])
                    else:
                        #draw_path2(SCREEN, path2)
                        draw_path(SCREEN, mpath2)
                    print "draw path 2"
                elif key_pressed(e, K_3):
                    clear_and_draw_base(SCREEN, ENV, colors)
                    if cpath2:
                        for (pa1, pb1), (pa2, pb2) in cpath2:
                            draw_line_blue(SCREEN, ENV.tg.V[pb1][0], ENV.tg.V[pb2][0])
                    else:
                        draw_path(SCREEN, mpath3)
                    print "draw path 3"
                elif key_pressed(e, K_4):
                    clear_and_draw_base(SCREEN, ENV, colors)
                    if cpath2:
                        for (pa1, pb1), (pa2, pb2) in cpath2:
                            draw_line_blue(SCREEN, ENV.tg.V[pb1][0], ENV.tg.V[pb2][0])
                    else:
                        draw_path(SCREEN, mpath4)
                    print "draw path 4"

                elif key_pressed(e, K_5):
                    clear_and_draw_base(SCREEN, ENV, colors)
                    draw_path(SCREEN, mpath5)
                    print "draw path 5"

                elif key_pressed(e, K_6):
                    clear_and_draw_base(SCREEN, ENV, colors)
                    draw_path(SCREEN, mpath6)
                    print "draw path 6"

                elif key_pressed(e, K_0):
                    clear_and_draw_base(SCREEN, ENV, colors)
                    if cpath:
                        for (pa1, pb1), (pa2, pb2) in cpath:
                            draw_line_red(SCREEN, ENV.tg.V[pa1][0], ENV.tg.V[pa2][0])
                            draw_line_blue(SCREEN, ENV.tg.V[pb1][0], ENV.tg.V[pb2][0])
                    else:
                        draw_path(SCREEN, mpath)
                        draw_path2(SCREEN, mpath2)
                    print "draw path 1 and 2"

                    if cpath2:
                        for (pa1, pb1), (pa2, pb2) in cpath2:
                            draw_line_red(SCREEN, ENV.tg.V[pa1][0], ENV.tg.V[pa2][0])
                            draw_line_blue(SCREEN, ENV.tg.V[pb1][0], ENV.tg.V[pb2][0])
                    else:
                        draw_path(SCREEN, mpath3)
                        draw_path2(SCREEN, mpath4)
                    print "draw path 3 and 4"

                    draw_path(SCREEN, mpath5)
                    print "draw path 5"

                    draw_path(SCREEN, mpath6)
                    print "draw path 6"

                elif key_pressed(e, K_d):
                    print "s, t, s2, t2, map data will be dumped."
                    print "s3, t3, s4, t4, paths will be dumped, too."
                    print "s5, t5, s6, t6, paths will be dumped, too."
                    conf_data['s1'] = s
                    conf_data['t1'] = t
                    conf_data['s2'] = s2
                    conf_data['t2'] = t2
                    conf_data['path1'] = path
                    conf_data['path2'] = path2
                    conf_data['s3'] = s3
                    conf_data['t3'] = t3
                    conf_data['s4'] = s4
                    conf_data['t4'] = t4
                    conf_data['path3'] = path3
                    conf_data['path4'] = path4
                    conf_data['s5'] = s5
                    conf_data['t5'] = t5
                    conf_data['path5'] = path5
                    conf_data['s6'] = s6
                    conf_data['t6'] = t6
                    conf_data['path6'] = path6
                    # if len(traj1) > 0:
                    #     conf_data['traj1'] = traj1
                    # if len(traj2) > 0:
                    #     conf_data['traj2'] = traj2

                    # if len(traja) > 0:
                    #     conf_data['traja'] = traja
                    # if len(trajb) > 0:
                    #     conf_data['trajb'] = trajb

                    # if len(traj3) > 0:
                    #     conf_data['traj3'] = traj3
                    # if len(traj4) > 0:
                    #     conf_data['traj4'] = traj4
                    # if len(traj5) > 0:
                    #     conf_data['traj5'] = traj5
                    # if len(traj6) > 0:
                    #     conf_data['traj6'] = traj6
                    output_ds = open(conf_file, 'wb')
                    pickle.dump(conf_data, output_ds)
                    output_ds.close()
                elif key_pressed(e, K_t):
                    assert path and path2

                    conflict, step = check_conflict(path, path2)
                    if conflict:
                        for i in range(step + 1):
                            (x1, y1), (x2, y2) = path[i]
                            #ptr += "((%04.1f,%04.1f),(%04.1f,%04.1f)) " % (x1, y1, x2, y2)
                        #ptr += "\n"
                        for i in range(step + 1):
                            (x1, y1), (x2, y2) = path2[i]
                            #ptr += "((%04.1f,%04.1f),(%04.1f,%04.1f)) " % (x1, y1, x2, y2)
                        #ptr += "\n"
                        #clear_and_draw_base(SCREEN, ENV, colors)
                        #draw_path(SCREEN, path[:step])
                        #draw_path2(SCREEN, path2[:step])

                        #mpath_1 = ENV.tg.get_shortest_midpath(path[0][0], path[step-1][0], SCREEN)
                        mpath_1 = ENV.tg.get_shortest_midpath(path[0][0], path[step-2][0], SCREEN)
                        #draw_path3(SCREEN, mpath_1, (255, 0, 0))
                        mpath1_3d = convert_to_midpath3d(mpath_1)
                        traj1 = s3d.run_rrt_3d_full(ENV, SCREEN, mpath1_3d, ENV.GREEN)

                        #mpath_2 = ENV.tg.get_shortest_midpath(path2[0][0], path2[step-1][0], SCREEN)
                        mpath_2 = ENV.tg.get_shortest_midpath(path2[0][0], path2[step-2][0], SCREEN)
                        #draw_path3(SCREEN, mpath_2, ENV.DODGER_BLUE) 
                        mpath2_3d = convert_to_midpath3d(mpath_2)
                        traj2 = s3d.run_rrt_3d_full(ENV, SCREEN, mpath2_3d, ENV.DODGER_BLUE)

                        pygame.display.update()
                        ls1, ls2 = path[step-1][0], path2[step-1][0]
                        #lg1, lg2 = get_local_goal(step, path, path2)
                        #(res, p1, p2) = compute_coop_astar(ENV, (ls1, ls2), (lg1, lg2))
                        #tick = time.time()
                        #cpath = compute_bfs2(ENV.tg, (ls1, ls2), (tg1, tg2), (patha, patha2), (dista, dista2))
                        #tock = time.time()
                        #print "compute_bfs2 took %f sec" % (tock - tick)
                        #tick = time.time()
                        cpath = compute_bfs3(ENV.tg, (ls1, ls2), (tg1, tg2), (patha, patha2), (dista, dista2))
                        #tock = time.time()
                        #print "compute_bfs3 took %f sec" % (tock - tick)
                        #assert (cpath == cpath2)
                        #print "cpath:",cpath
                        #print "cpath2:",cpath2

                        #res, cpath = compute_bfs(ENV, (ls1, ls2), (lg1, lg2))
                        #if res:
                        #    for (pa1, pb1), (pa2, pb2) in cpath:
                        #        draw_line_red(SCREEN, pa1, pa2)
                        #        draw_line_blue(SCREEN, pb1, pb2)
                        if len(cpath) > 0:
                            #cpatha = [(ENV.tg.V[pa1][0], ENV.tg.V[pa2][0]) for (pa1, _), (pa2, __) in cpath2]
                            #cpathb = [(ENV.tg.V[pb1][0], ENV.tg.V[pb2][0]) for (_, pb1), (__, pb2) in cpath2]
                            cpatha = [(pa1, pa2) for (pa1, _), (pa2, __) in cpath]
                            cpathb = [(pb1, pb2) for (_, pb1), (__, pb2) in cpath]
                            #mpatha = ENV.tg.get_shortest_midpath(cpatha[0][0], cpatha[-1][1])
                            #mpathb = ENV.tg.get_shortest_midpath(cpathb[0][0], cpathb[-1][1])
                            mpatha = ENV.tg.get_coop_midpath(mpath_1[-1][0], tg1, cpatha, SCREEN)
                            mpathb = ENV.tg.get_coop_midpath(mpath_2[-1][0], tg2, cpathb, SCREEN)
                            mpatha3d = convert_to_midpath3d(mpatha)
                            mpathb3d = convert_to_midpath3d(mpathb)

                            #for (u, v) in mpatha3:
                            #    vehicle.draw_temp_car2(SCREEN, u, (255, 0, 0))
                            #vehicle.draw_temp_car2(SCREEN, v, (255, 0, 0))
                            #pygame.display.update()

                            #for (u, v) in mpathb3:
                            #    vehicle.draw_temp_car2(SCREEN, u, ENV.DODGER_BLUE)
                            #vehicle.draw_temp_car2(SCREEN, v, ENV.DODGER_BLUE)
                            #pygame.display.update()

                            traja = s3d.run_rrt_3d_full(ENV, SCREEN, mpatha3d, ENV.GREEN)
                            trajb = s3d.run_rrt_3d_full(ENV, SCREEN, mpathb3d, ENV.DODGER_BLUE)

                            #for i in range(min(len(mpatha), len(mpathb))):
                            #    (p1, p2), (p3, p4) = mpatha[i], mpathb[i]
                            #    draw_line_red(SCREEN, p1, p2)
                            #    handle_quit_event()
                            #    draw_line_blue(SCREEN, p3, p4)
                            #    handle_quit_event()


                            #for (pa1, pb1), (pa2, pb2) in cpath2:
                            #    draw_line_red(SCREEN, ENV.tg.V[pa1][0], ENV.tg.V[pa2][0])
                            #    handle_quit_event()
                            #    draw_line_blue(SCREEN, ENV.tg.V[pb1][0], ENV.tg.V[pb2][0])
                            #    handle_quit_event()
                elif key_pressed(e, K_f):
                    assert path3 and path4
                    conflict, step = check_conflict(path3, path4)
                    if conflict:
                        for i in range(step + 1):
                            (x1, y1), (x2, y2) = path3[i]
                        for i in range(step + 1):
                            (x1, y1), (x2, y2) = path4[i]
                        #clear_and_draw_base(SCREEN, ENV, colors)
                        mpath_3 = ENV.tg.get_shortest_midpath(path3[0][0], path3[step-1][0], SCREEN)
                        #draw_path3(SCREEN, mpath_3, (255, 0, 0))
                        mpath3_3d = convert_to_midpath3d(mpath_3)
                        #traj3 = s3d.run_rrt_3d_full(ENV, SCREEN, mpath3_3d, (255, 0, 0))

                        mpath_4 = ENV.tg.get_shortest_midpath(path4[0][0], path4[step-1][0], SCREEN)
                        #draw_path3(SCREEN, mpath_4, ENV.DODGER_BLUE)
                        mpath4_3d = convert_to_midpath3d(mpath_4)
                        #traj4 = s3d.run_rrt_3d_full(ENV, SCREEN, mpath4_3d, ENV.DODGER_BLUE)

                        pygame.display.update()
                        ls3, ls4 = path3[step-1][0], path4[step-1][0]
                        cpath2 = compute_bfs3(ENV.tg, (ls3, ls4), (tg3, tg4), (patha3, patha4), (dista3, dista4))
                        if len(cpath2) > 0:
                            cpatha2 = [(pa1, pa2) for (pa1, _), (pa2, __) in cpath2]
                            cpathb2 = [(pb1, pb2) for (_, pb1), (__, pb2) in cpath2]
                            mpatha2 = ENV.tg.get_coop_midpath(mpath_3[-1][0], tg3, cpatha2, SCREEN)
                            mpathb2 = ENV.tg.get_coop_midpath(mpath_4[-1][0], tg4, cpathb2, SCREEN)
#                            th3 = atan2(mpath_4[-1][1][1] - mpath_4[-1][0][1], mpath_4[-1][1][0], mpath_4[-1][0][0])
#                            th4 = atan2(mpath_4[-1][1][1] - mpath_4[-1][0][1], mpath_4[-1][1][0], mpath_4[-1][0][0])
                            mpatha3d2 = convert_to_midpath3d(mpatha2)
                            mpathb3d2 = convert_to_midpath3d(mpathb2)

                            #for (u, v) in mpatha3d2:
                            #    vehicle.draw_temp_car2(SCREEN, u, (255, 0, 0))
                            #vehicle.draw_temp_car2(SCREEN, v, (255, 0, 0))
                            #pygame.display.update()

                            #for (u, v) in mpathb3d2:
                            #    vehicle.draw_temp_car2(SCREEN, u, ENV.DODGER_BLUE)
                            #vehicle.draw_temp_car2(SCREEN, v, ENV.DODGER_BLUE)
                            #pygame.display.update()

                            #for i in range(min(len(mpatha2), len(mpathb2))):
                            #    (p1, p2), (p3, p4) = mpatha2[i], mpathb2[i]
                            #    draw_line_red(SCREEN, p1, p2)
                            #    handle_quit_event()
                            #    draw_line_blue(SCREEN, p3, p4)
                            #    handle_quit_event()

                            trajd = s3d.run_rrt_3d_full(ENV, SCREEN, mpathb3d2, ENV.DODGER_BLUE)
                            trajc = s3d.run_rrt_3d_full(ENV, SCREEN, mpatha3d2, ENV.GREEN)
                elif key_pressed(e, K_r):
                    assert path3 and path4 and path5 and path6
                    #clear_and_draw_base(SCREEN, ENV, colors)
                    traj4 = s3d.run_rrt_3d_full(ENV, SCREEN, mpath4_3d, ENV.GREEN)
                    traj5 = s3d.run_rrt_3d_full(ENV, SCREEN, mpath5_3d, ENV.GREEN)
                    traj3 = s3d.run_rrt_3d_full(ENV, SCREEN, mpath3_3d, ENV.DODGER_BLUE)
                    traj6 = s3d.run_rrt_3d_full(ENV, SCREEN, mpath6_3d, ENV.GREEN)

                elif key_pressed(e, K_p):
                    if len(traj4) > 0:
                        s3d.draw_traj(ENV, SCREEN, traj4, ENV.GREEN)
                    if len(traj5) > 0:
                        s3d.draw_traj(ENV, SCREEN, traj5, ENV.GREEN)
                    if len(traj3) > 0:
                        s3d.draw_traj(ENV, SCREEN, traj3, ENV.DODGER_BLUE)
                    if len(traj6) > 0:
                        s3d.draw_traj(ENV, SCREEN, traj6, ENV.GREEN)
                    if len(traj1) > 0:
                        s3d.draw_traj(ENV, SCREEN, traj1, ENV.GREEN)
                    if len(traj2) > 0:
                        s3d.draw_traj(ENV, SCREEN, traj2, ENV.DODGER_BLUE)
                    if len(traja) > 0:
                        s3d.draw_traj(ENV, SCREEN, traja, ENV.GREEN)
                    if len(trajb) > 0:
                        s3d.draw_traj(ENV, SCREEN, trajb, ENV.DODGER_BLUE)

                clicked, (cx, cy) = mouse_clicked()
                car1_done = s is not None and t is not None
                car2_done = s2 is not None and t2 is not None
                car3_done = s3 is not None and t3 is not None
                car4_done = s4 is not None and t4 is not None
                car5_done = s5 is not None and t5 is not None
                car6_done = s6 is not None and t6 is not None
                group1_done = car1_done and car2_done
                group2_done = car3_done
                group3_done = car4_done and car5_done and car6_done
                if not clicked:
                    pass
                elif s is None and t is None and not car2_done and not group2_done and not group3_done:
                    s = (cx, cy)
                    ptr = "car 1 start position is chosen (%d, %d)! " % (cx, cy)
                    ptr += "click end position."
                    print ptr
                elif s is not None and t is None and not car2_done and not group2_done and not group3_done:
                    t = (cx, cy)
                    ptr = "car 1 end position is chosen (%d, %d)! " % (cx, cy)
                    ptr += "Here is a new path!"
                    print ptr
                    path = ENV.tg.get_shortest_path(s, t)
                    tg1 = path[-1][0]
                    patha, dista = ENV.tg.compute_shortest_reverse(tg1)
                    mpath = ENV.tg.get_shortest_midpath(s, t, SCREEN)
                elif s2 is None and t2 is None and car1_done and not group2_done and not group3_done:
                    s2 = (cx, cy)
                    ptr = "car 2 start position is chosen (%d, %d)! " % (cx, cy)
                    ptr += "click end position."
                    print ptr
                elif s2 is not None and t2 is None and car1_done and not group2_done and not group3_done:
                    t2 = (cx, cy)
                    ptr = "car 2 end position is chosen (%d, %d)! " % (cx, cy)
                    ptr += "Here is a new path!"
                    print ptr
                    path2 = ENV.tg.get_shortest_path(s2, t2)
                    tg2 = path2[-1][0]
                    patha2, dista2 = ENV.tg.compute_shortest_reverse(tg2)
                    mpath2 = ENV.tg.get_shortest_midpath(s2, t2, SCREEN)
                elif group1_done and s3 is None and t3 is None and not car4_done and not group3_done:
                    s3 = (cx, cy)
                    ptr = "car 3 start position is chosen (%d, %d)! " % (cx, cy)
                    ptr += "click end position."
                elif group1_done and s3 is not None and t3 is None and not car4_done and not group3_done:
                    t3 = (cx, cy)
                    ptr = "car 3 end position is chosen (%d, %d)! " % (cx, cy)
                    ptr += "Here is a new path!"
                    print ptr
                    path3 = ENV.tg.get_shortest_path(s3, t3)
                    tg3 = path3[-1][0]
                    patha3, dista3 = ENV.tg.compute_shortest_reverse(tg3)
                    mpath3 = ENV.tg.get_shortest_midpath(s3, t3, SCREEN)
                    mpath3_3d = convert_to_midpath3d(mpath3)
                elif group1_done and s4 is None and t4 is None and car3_done and not group3_done:
                    s4 = (cx, cy)
                    ptr = "car 4 start position is chosen (%d, %d)! " % (cx, cy)
                    ptr += "click end position."
                elif group1_done and s4 is not None and t4 is None and car3_done and not group3_done:
                    t4 = (cx, cy)
                    ptr = "car 4 end position is chosen (%d, %d)! " % (cx, cy)
                    ptr += "Here is a new path!"
                    print ptr
                    path4 = ENV.tg.get_shortest_path(s4, t4)
                    tg4 = path4[-1][0]
                    patha4, dista4 = ENV.tg.compute_shortest_reverse(tg4)
                    mpath4 = ENV.tg.get_shortest_midpath(s4, t4, SCREEN)
                    mpath4_3d = convert_to_midpath3d(mpath4)
                elif s5 is None and t5 is None and group1_done and group2_done and not group3_done:
                    s5 = (cx, cy)
                    ptr = "car 5 start position is chosen (%d, %d)! " % (cx, cy)
                    t5 = mpath4[-1][0]
                    ptr += "car 5 end position is (%04.1f, %04.1f)! " % (t5[0], t5[1])
                    ptr += "Here is a new path!"
                    print ptr
                    path5 = ENV.tg.get_shortest_path(s5, t5)
                    tg5 = path5[-1][0]
                    patha5, dista5 = ENV.tg.compute_shortest_reverse(tg5)
                    mpath5 = ENV.tg.get_shortest_midpath(s5, t5, SCREEN)
                    mpath5_3d = convert_to_midpath3d(mpath5)
                elif t6 is None and group1_done and group2_done and not group3_done:
                    s6 = mpath5[0][0]
                    t6 = (cx, cy)
                    ptr = "car 6 end position is chosen (%d, %d)! " % (cx, cy)
                    ptr += "car 6 start position is (%04.1f, %04.1f)!" % (s6[0], s6[1])
                    print ptr
                    path6 = ENV.tg.get_shortest_path(s6, t6)
                    tg6 = path6[-1][0]
                    patha6, dista6 = ENV.tg.compute_shortest_reverse(tg6)
                    mpath6 = ENV.tg.get_shortest_midpath(s6, t6, SCREEN)
                    mpath6_3d = convert_to_midpath3d(mpath6)

            pygame.display.update()
