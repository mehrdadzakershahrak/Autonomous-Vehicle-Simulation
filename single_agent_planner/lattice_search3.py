# lattice_search3.py {{{
# Description:
#   Low Level Planner
#
#   It uses:
#     1. map_loader
#     2. lattice dictionary 
#       * primitive2.py.0
#       * grid_primitives.py.0
#       * local_points3.py.2
#       * local_points4.py.0
#
# Date: 04-20-2017
#
# Author: Kangjin Kim (kangjin.kim@asu.edu)
#
# Usage:
#   $ python lplan.py
#
# Related codes:
#   map_loader4_8.py, tgraph3.py, primitive2.py,
#   local_search2.py, local_search3.py
# }}}

#---Package imports-------------------------------------{{{1
from math import fabs, tan, sin, cos, pi, sqrt, atan2, floor, ceil
#import car2d as vehicle
import car3d as vehicle
import pygame
from pygame.locals import *
import time
import sys
from pylygon import Polygon
from numpy import ndarray, array, dot
from numpy import isclose, linspace, interp
import numpy as np
from scipy.ndimage import gaussian_filter1d as gau
import treetemp2d as tr
from treetemp2d import Node as Node2D, Tree as Tree2D
from treetemp3d import Node as Node3D, Tree as Tree3D
#from map_loader4 import load_map, get_near_borders_lpoly
from map_loader4_8 import load_map
from random import uniform, random, randint, choice
from collections import defaultdict
import heapq
from heapq import heappush as hpush, heappop as hpop
import tgraph3
import uuid
import itertools
from lineequ5 import is_intersected as itsec, get_intersection
import kdtree
import os
#import pickle
import cPickle as pickle
import copy
import gc

#---Classes---------------------------------------------{{{1
class Environment():                                    #{{{2
    pass

class NodeLT(object):                                   #{{{2
    def __init__(self, nvalue, ctrl = None, DIR = 0, weights=(0.0, 0.0)):
        #self.nid = uuid.uuid1()
        #self.nid = uuid.uuid1().time_low
        self.nvalue = nvalue
        self.cweight = weights[0]
        self.tweight = weights[1]
        self.parent = None
        self.children = set()
        self.ctrl = ctrl
        self.DIR = DIR
        #self.nid = 0
        #self.nid = uuid.uuid1().time_low

    def __repr__(self):                                 #{{{3
        return "N(%03.1f, %03.1f, %06.4f, %4.2f)" % self.nvalue
        
    def __eq__(self, other):
        if self.nvalue[:3] == other.nvalue[:3]:
            return True
        else:
            return False

    # def __hash__(self):
    #     #return self.nid.time_low
    #     #return hash(self.nid)
    #     return hash(self.nvalue)

class TreeLT(object):                                   #{{{2
    def __init__(self, root):
        self.nodes = defaultdict(set)
        self.root = root
        self.add_node(self.root)

    def __len__(self):
        return len(self.nodes)

    def __repr__(self):
        name = self.__class__.__name__
        h = [(0, self.root)]
        output = "\n"
        while len(h) > 0:
            (dth, node) = h.pop()
            output += ' ' * dth + str(repr(node)) + "\n"
            if len(node.children) > 0:
                h += [(dth + 1, child) for child in node.children]
        return output

    def add_node(self, node):
        (x, y, th) = node.nvalue[:3]
        self.nodes[(x, y)].add(node)

    def add_tree(self, pnode, node, ctrl, DIR, duration = None):
        node.ctrl = ctrl
        node.DIR = DIR

        if node.parent is not None:
            raise BaseException, "node:%s already has its parent!" % repr(node)

        pnode.children.add(node)
        node.parent = pnode
        if duration is not None:
            node.tweight = duration
        else:
            node.tweight = self.distance(pnode, node)
        node.cweight = pnode.cweight + node.tweight

        self.add_node(node)

    def compute_path(self, node):
        assert node != self.root
        path = [(node.parent, node)]
        cursor = node.parent
        while cursor != self.root and len(path) < len(self.nodes):
            path = [(cursor.parent, cursor)] + path
        if len(path) >= len(self.nodes):
            raise BaseException, "Path has cycle!"
        return path

    def distance(self, node1, node2):                   #{{{3
        return self.distance_2d(node1.nvalue, node2.nvalue)

    def distance_2d(self, p1, p2):                      #{{{3
        (x1, y1) = (list(p1)[0], list(p1)[1])
        (x2, y2) = (list(p2)[0], list(p2)[1])
        return sqrt((x1 - x2)**2 + (y1 - y2)**2)
class PartialPath(object):                              #{{{2
    def __init__(self):
        self.pid = uuid.uuid1()
        self.path = []
        self.mpath = []
        self.ptype = -2 # -2:default, -1:reverse, 0:stay, 1:forward
        self.t2b = None
        self.t2mp = None
        self.wpoints = []
        self.gpoints = []
        self.times = -1
        self.plast = None
        self.mpath_close = []
        self.passed = []
        self.past_path = []

    def set_type(self, ptype):
        self.ptype = ptype

    def add_nodes(self, (u, v), (mu, mv)):
        if len(self.path) > 0:
            lu, lv = self.path[-1]
            if lv != u:
                p = "%s\n" % self
                p += "Can't connect %s to %s" % (repr((u,v)), repr((lu,lv)))
                raise BaseException, p
        self.path.append((u, v))
        self.mpath.append((mu, mv))

    def __repr__(self):
        return "P[%d]" % self.ptype + ",".join(map(str, self.path))

    def __len__(self):
        return len(self.path)

    def __eq__(self, other):
        if self.ptype == other.ptype and self.path == other.path:
            return True
        else:
            return False

    def __hash__(self):
        return self.pid.time_low

#---Distance--------------------------------------------{{{1
def distance_2d(p1, p2):                                #{{{2
    (x1, y1) = (list(p1)[0], list(p1)[1])
    (x2, y2) = (list(p2)[0], list(p2)[1])
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def valid_state_2d(p1, ENV):                            #{{{2
    #global ENV
    BELOW_MIN = p1[0] <= ENV.MIN_X or p1[1] <= ENV.MIN_Y
    ABOVE_MAX = p1[0] >= ENV.MAX_X or p1[1] >= ENV.MAX_Y
    return not (BELOW_MIN or ABOVE_MAX)

def enforce_velocity(v1, DIR, acc, ENV):                #{{{2
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

def enforce_theta(th1, th2):                            #{{{2
    th2 = th2 + th1
    if th2 <= -pi:
        th2 += 2.0 * pi
    elif th2 > pi:
        th2 -= 2.0 * pi
    return th2

def fabs3(th1, th2):                                    #{{{2
    th1, th2 = fabs_helper(th1), fabs_helper(th2)
    return abs(fabs_helper(fabs(th1 - th2)))

def rev_th(th):                                         #{{{2
    return fabs_helper(th + pi)

def fabs_helper(th):                                    #{{{2
    while th >= pi or th < -pi:
        if th < -pi:
            th += 2.0 * pi
        elif th >= pi:
            th -= 2.0 * pi
    return th

def thdiff(p1, p2, p3):                                 #{{{2
    th21 = atan2(p2[1] - p1[1], p2[0] - p1[0])
    th32 = atan2(p3[1] - p2[1], p3[0] - p2[0])
    return fabs3(th21, th32)

def get_children2(th, cset):
    #th = tnode.nvalue[2]
    thkey = float(format(th, '.2f'))
    return cset[thkey]
#---Collision-------------------------------------------{{{1
def collide((x1, y1, th1), near_lpolys, ENV):           #{{{2
    #global ENV
    #car = vehicle.Vehicle3D((x1, y1), th1)
    #car = vehicle.Vehicle3D((x1, ENV.MAX_Y - y1), th1)
    car = vehicle.Vehicle3D((x1, y1), th1)
    for lpoly in near_lpolys:
        proj = car.collidepoly(lpoly)
        if type(proj) is bool and proj:
            return True
        if type(proj) is ndarray and proj.size > 0:
            return True
        if type(proj) is array and proj.size > 0:
            return True
    return False

#---Models----------------------------------------------{{{1
def simple_kinematic_model((x, y, th), (v, w), dt, nsteps):#{{{2
    (x1, y1, th1) = (x, y, th)
    for i in range(nsteps):
        x2 = x1 + v * cos(th1) * dt
        y2 = y1 + v * sin(th1) * dt
        th2 = th1 + w * dt
        th2 = fabs_helper(th2)
        (x1, y1, th1) = (x2, y2, th2)
    return (x1, y1, th1, v) # since v is constant, it is just added here.

def kinematic_model((x, y, th), (v, w), dt, L, nsteps): #{{{2
    (x1, y1, th1) = (x, y, th)
    for i in range(nsteps):
        x2 = x1 + v * cos(th1) * dt
        y2 = y1 + v * sin(th1) * dt
        th2 = th1 + v * (1.0 / L) * w * dt
        th2 = fabs_helper(th2)
        (x1, y1, th1) = (x2, y2, th2)
    return (x1, y1, th1, v) # since v is constant, it is just added here.

def kinodynamic_model(state, ctrl, dt, L, vmax, threshold, nsteps):#{{{2
    (x, y, th, v), (accel, steer) = state, ctrl
    for i in range(nsteps):
        x2 = x + v * cos(th) * dt
        y2 = y + v * sin(th) * dt
        th2 = th + v * (1.0 / L) * tan(steer) * dt
        v2 = v + accel * dt
        th2 = fabs_helper(th2)
        v2 = enforece_velocity(v2, v, vmax, threshold)
        (x, y, th, v) = (x2, y2, th2, v2)
    return (x, y, th, v)

def transform3(p1, p2):
    return local_to_global2(p1, p2)

def global_to_local2(p1, p2):                           #{{{2
    (x1, y1, th1), (x2, y2, th2) = p1[:3], p2[:3]
    d1 = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    th21 = atan2(y2 - y1, x2 - x1)
    th20 = fabs_helper(th21 - th1)

    x4 = 0.0 + d1 * cos(th20)
    y4 = 0.0 + d1 * sin(th20)
    return (x4, y4, fabs_helper(th2 - th1))

def local_to_global2(p1, p2):                           #{{{2
    (x1, y1, th1), (x2, y2, th2) = p1[:3], p2[:3]
    x3 = x1 + x2 * cos(th1) - y2 * sin(th1)
    y3 = y1 + x2 * sin(th1) + y2 * cos(th1)
    th3 = fabs_helper(th2 + th1)
    return (x3, y3, th3)

#---Simulate--------------------------------------------{{{1
def simulate_start3(tx, closest, ppath, DIR, ENV):      #{{{2
    # NOTE:
    #   1. adjusting the orientation to the closest degree:
    #     * back & forth would be allowable
    #     * slow velocity only
    #   2. stop after orientation is evenly matched.

    ### load closest node
    Xnode = closest
    xnew = closest.nvalue
    (x, y, th, v) = xnew

    ### check the initial orientation
    if isclose(xnew[2] % ENV._1d, 0.0, atol = 0.002):
        Xnode2 = closest
        even_heading = True
    else:
        off = th % ENV._1d
        print "off: %f, _1d:%f" % (off, ENV._1d)
        print "th: %f" % th
        even_heading = False

    p = "even_heading:%s, th:%f, off:%f, " % (repr(even_heading), th, xnew[2] % ENV._1d)
    if even_heading:
        p += "target_th:%f" % target_th
    print p

    if not even_heading:
        th = xnew[2]
        off = th % ENV._1d
        sb = ENV._1d * 0.05
        accel = 0.005       # FIXME: min accel exceeding threshold with one dt
        target_th = floor(th / ENV._1d) * ENV._1d
        print "target_th0:%f" % target_th
        if off < ENV._1d * 0.5: # steering left (negative)
            #steering = -ENV.MAX_STEER / 0.125
            steering = -ENV.MAX_STEER / 16
            sd = -ENV._1d * 0.05
        else:               # steering right (positive)
            #steering = ENV.MAX_STEER * 0.125
            steering = ENV.MAX_STEER / 16
            sd = ENV._1d * 0.05
            target_th += ENV._1d if target_th >= 0 else -ENV._1d
            target_th = fabs_helper(target_th)

        # TODO: fix it with DIR consideration
        print "simulate_th, off:%06.4f, _1d:%06.4f, sd:%08.6f"%(off,ENV._1d,sd)
        print "target_th:%06.4f " % target_th
        result = simulate_th2(xnew, target_th, (accel, steering), DIR, sd, ENV)
        if result[0]:
            (lsteps, x, y, th, v), cweight = result[3:]
            th = target_th
        else:
            # We may need to adjust the steering from the MAX to 0.0 by SB
            # In that case, we have to add multiple node to the closest node.
            raise BaseException

        ### add it to the tree
        print "add it to the tree"
        Xnode2 = Node3D((x, y, th, v))
        xctrl2 = (accel, steering)
        try:
            tx.add_tree(closest, Xnode2, xctrl2, cweight)
        except BaseException:
            raise BaseException
        (x, y, th, v) = Xnode2.nvalue
        closest = Xnode2
        p = "xnew:(%03.1f, %03.1f, " % (xnew[0], xnew[1])
        p += "%06.4f, %03.1f), target_th:" % (xnew[2], xnew[3])
        p += "%06.4f, (%06.4f, %06.4f), " % (target_th, accel, steering)
        p += "(%03.1f, %03.1f, %06.4f, %03.1f), " % (x, y, th, v)
        p += "cweight:%06.4f" % cweight
        #p += "lsteps:%d, xdist:%06.4f" % (lsteps, xdist)
        print p

    ### Stop
    tx, stop_node = simulate_break2(tx, Xnode2, ppath, ppath.ptype, ENV)

    return tx, stop_node

def simulate_break2(tx, closest, ppath, DIR, ENV):      #{{{2
    Xnode = closest
    xnew = closest.nvalue
    if xnew[3] == 0.0:
        return tx, closest
    #(found, accel, lsteps) = break_helper(xnew, DIR, ENV)
    (x, y, th, v) = xnew
    cursor = closest
    path = break_helper(xnew, DIR, ENV)
    for j, (_, __, accel, lsteps) in enumerate(path):
        steering = 0.0
        cweight = 0.0
        #(x, y, th, v) = closest.nvalue
        valid = True
        for i in range(lsteps):
            x2 = x + v * cos(th) * ENV.dt
            y2 = y + v * sin(th) * ENV.dt
            self_th = (v / ENV.L) * tan(steering) * ENV.dt
            th2 = enforce_theta(th, self_th)
            (v2, terminate) = enforce_velocity(v, DIR, accel, ENV)
            valid = valid and valid_state_2d((x2, y2), ENV)
            if not valid:
                break
            # skip collision check here
            dist2 = distance_2d((x, y), (x2, y2))
            cweight += dist2
            (x, y, th, v) = (x2, y2, th2, v2)
            if terminate:
                break
        if not valid:
            raise BaseException, "not valid"
        Xnode = Node3D((x, y, th, v))
        xctrl = (accel, steering)
        try:
            tx.add_tree(cursor, Xnode, xctrl, cweight)
        except BaseException:
            raise BaseException
        (x, y, th, v), cursor = Xnode.nvalue, Xnode
    if len(path) == 0:
        raise BaseException, "simulate_break failed"

    return tx, Xnode

def simulate_th2(src, target_th, (accel, steering), DIR, SB, ENV): #{{{2
    steering2 = steering
    while -ENV.MAX_STEER <= steering2 and steering2 <= ENV.MAX_STEER:
        x, y, th, v = src
        cweight = 0.0
        for i in range(1000):
            x2 = x + v * cos(th) * ENV.dt
            y2 = y + v * sin(th) * ENV.dt
            th2 = th + (v / ENV.L) * tan(steering) * ENV.dt
            v2 = v + DIR * accel * ENV.dt
            dist2 = distance_2d((x, y), (x2, y2))
            (x, y, th, v) = (x2, y2, th2, v2)
            cweight += dist2
            if isclose(th, target_th, atol = 0.001):
                return (True,target_th,(accel,steering),(i+1,x,y,th,v),cweight)
        steering2 += SB
    return (False,None,None,None,None)

def break_helper((x0, y0, th0, v0), DIR, ENV):          #{{{2
    prev_v, lsteps, itr, found, path = v0, -1, 0, False, []
    if DIR != 1 and DIR != -1:
        raise BaseException, "Wrong DIR:%s" % repr(DIR)
    v = prev_v
    while v != 0 and itr < 1000:
        accel = min(abs(prev_v), ENV.MAX_ACCEL) * random() * -1
        v = prev_v
        for i in range(ENV.MAX_NSTEP):
            v += DIR * accel * ENV.dt
            if abs(v) < ENV.V_THRESHOLD:
                found = True
                break
            if v * DIR < -ENV.V_THRESHOLD:
                break
        if found:
            path.append((prev_v, v, accel, i + 1))
            break
        if 0 < v * DIR and v * DIR < prev_v * DIR:
            path.append((prev_v, v, accel, i + 1))
            prev_v = v
        itr += 1
    return path

#---Path Related----------------------------------------{{{1
def get_mpoints2(mplast, p0, ENV):                      #{{{2
    assert type(mplast[0]) is tuple
    mp1, mp2 = [mplast], []
    for i in range(len(p0)-1):
        (s0, s1), (s1, s2) = p0[i], p0[i+1]
        if s0 != s1 and s1 != s2 and s0 != s2:
            m0 = mp1[-1][1]
            m1 = get_mpoint_id(s0, s1, ENV)
            m2 = get_mpoint_id(s1, s2, ENV)
            m_start, m_end = m0, get_mpoint_coord(m1, m2, ENV)
        elif s0 != s1 and s1 != s2 and s2 == s0:
            m0 = mp1[-1][1]
            m1 = get_mpoint_id(s0, s1, ENV)
            m2, _ = ENV.tg.V[s1]    # it is a centroid of s1
            m_start, m_end = m0, m2
        elif s0 != s1 and s1 == s2:
            m0 = mp1[-1][1]
            m1 = get_mpoint_id(s0, s1, ENV)
            m2, _ = ENV.tg.V[s1]    # it is a centroid of s1
            m_start, m_end = m0, m2
        elif s0 == s1 and s1 != s2:
            m0 = mp1[-1][1]
            m1 = mp1[-1][1] # since it stops at time_i
            m_start, m_end = m0, m1
        elif s0 == s1 and s1 == s2:
            m0 = mp1[-1][1]
            m1 = mp1[-1][1]
            m_start, m_end = m0, m1
        else:
            raise BaseException
        mp1.append((m_start, m_end))
        if i == len(p0) - 2:
            m_final, _ = ENV.tg.V[s2]
            mp1.append((m_end, m_final))
    return mp1[1:]  

def get_mpoint_id(s0, s1, ENV):
    (x1, y1), _ = ENV.tg.V[s0]
    (x2, y2), _ = ENV.tg.V[s1]
    a = set(ENV.tg.idflsmap[s0])
    b = set([(p2, p1) for (p1, p2) in a])
    common_edges = (a.union(b)).intersection(ENV.tg.idflsmap[s1])
    if len(common_edges) != 1:
        raise BaseException, "No common edge exists between (%d,%d)!" % (s0, s1)
    return tgraph3.get_midpoint(list(common_edges)[0])

def get_mpoint_coord(s0, s1, ENV):
    return tgraph3.get_midpoint((s0, s1))


def convert_to_lpolies(t2b):                            #{{{2
    #global ENV
    new_t2b = {}
    for key, value in t2b.iteritems():
        polies = set()
        for u, v in value:
            #(x0, y0), (x1, y1) = u, v
            #u1, v1 = (x0, ENV.MAX_Y - y0), (x1, ENV.MAX_Y - y1)
            polies.add(Polygon([u, v]))
            #polies.add(Polygon([u1, v1]))
        new_t2b[key] = polies
        #new_t2b[key] = set([Polygon([u, v]) for u, v in value])
    #return {k:set([Polygon([u,v]) for u,v in p]) for k,p in t2b.iteritems()}
    return new_t2b

def get_intermediates((u, v), DIR, ENV):                #{{{2
    (ux, uy, uth), (vx, vy, vth) = u[:3], v[:3]
    uv_dist = distance_2d((ux, uy), (vx, vy))
    h_max = ceil(uv_dist / ENV.EPSILON)
    uv_th = atan2(vy - uy, vx - ux)
    if DIR == -1:
        uv_th = rev_th(uv_th)
    intermediates = []
    hdist_base = uv_dist / h_max
    for h in range(int(h_max)):
        if h == 0:
            intermediates.append((ux, uy, uth))
        else:
            hdist = h * hdist_base
            ux1, uy1 = ux + hdist * cos(uv_th), uy + hdist * sin(uv_th)
            ux0, uy0, _ = intermediates[-1]
            uth1 = atan2(uy1 - uy0, ux1 - ux0)
            intermediates.append((ux1, uy1, uth1))
    if int(h_max) == 0:
        intermediates.append((ux, uy, uth))
    intermediates.append((vx, vy, vth))
    return intermediates

def get_smooth_path(ppath, prev_mpath_c, ENV):          #{{{2
    # NOTE: this function should be called whenever times value is updated.
    #print "get_smooth_path #1, prev_mpath_c:%s" % repr(prev_mpath_c)
    mpath_close, mpath_wpoints = [], []
    for i in range(len(ppath.mpath) - 1):
        (p1, p2), (_, p3) = ppath.mpath[i:i+2]
        if len(mpath_close) == 0:
            (p1, p2) = prev_mpath_c[0]
        elif mpath_close[-1][1][:3] == p2[:3]: # then, we just did a do_smooth.
            (p1, p2) = mpath_close[-1]
        th0, v0, thl, vl = p1[2], p1[3], p3[2], p3[3]
        #p = "i:%d, p1(%3.1f, %3.1f, %6.4f, %3.1f), " % (i,p1[0],p1[1],th0,v0)
        #p += "p2(%3.1f, %3.1f, %6.4f, %3.1f), " % p2
        #p += "p3(%3.1f, %3.1f, %6.4f, %3.1f)" % (p3[0], p3[1], thl, vl)
        #print p
        if thdiff(p1, p2, p3) > ENV.MAX_STEER * 0.5:
            mplist = do_smooth(p1, p2, p3, (5, 1.5))
            mpath2 = [(p1, p1)]
            for j in range(len(mplist) - 1):
                _0, (_1, _2, th1, _4) = mpath2[-1]
                (x1, y1), (x2, y2) = mplist[j:j+2]
                #th1 = atan2(y2 - y1, x2 - x1)
                #th2 = thl
                th2 = atan2(y2 - y1, x2 - x1)
                #(_, __) = mpath2[-1]
                #mpath2[-1] = (_, (x1, y1, th1, v0))
                #mpath2.append(((x1, y1, th1, v0), (x2, y2, thl, vl)))
                mpath2[-1] = (_0, (x1, y1, th1, v0))
                if j < len(mplist) - 2:
                    mpath2.append(((x1, y1, th1, v0), (x2, y2, th2, vl)))
                else:
                    mpath2.append(((x1, y1, th1, v0), (x2, y2, thl, vl)))
            mpath2 = mpath2[1:]
            mplist2 = []
            for (u, v) in mpath2:
                intm = get_intermediates((u, v), ppath.ptype, ENV)
                intmv = [(x, y, z, v0) for (x, y, z) in intm]
                if len(mplist2) > 0 and intmv[0][:3] == mplist2[-1][:3]:
                    mplist2 += intmv[1:]
                else:
                    mplist2 += intmv
            close_point, close_dist_p2 = mplist2[1],distance_2d(mplist2[1],p2)
            for v in mplist2[2:]:
                dist = distance_2d(v, p2)
                if dist < close_dist_p2:
                    close_point, close_dist_p2 = v, dist
            if len(mpath_close) > 0 and p2[:3] == mpath_close[-1][1][:3]:
                _ = mpath_close.pop()
                v = mpath_wpoints.pop()
                while v[:3] != p1[:3]:
                    v = mpath_wpoints.pop()
                mpath_wpoints.append(v)
            mpath_close += [(p1, close_point), (close_point, p3)]
            if len(mpath_wpoints) == 0:
                mpath_wpoints += mplist2
            elif mplist2[0][:3] == mpath_wpoints[-1][:3]:
                mpath_wpoints += mplist2[1:]
            else:
                mpath_wpoints += mplist2
        else:
            if len(mpath_close) == 0 or p2 != mpath_close[-1][1]:
                mpath_close.append((p1, p2))
                intm = get_intermediates((p1, p2), ppath.ptype, ENV)
                intmv = [(x, y, z, v0) for (x, y, z) in intm]
                if len(mpath_wpoints) == 0:
                    mpath_wpoints += intmv
                elif intmv[0][:3] == mpath_wpoints[-1][:3]:
                    mpath_wpoints += intmv[1:]
                else:
                    mpath_wpoints += intmv
    if len(ppath.mpath) == 1:
        #p = "get_smooth_path #1, ppath.ptype:%d, " % ppath.ptype
        #p += "ppath.mpath:%s, " % repr(ppath.mpath)
        #p += "prev_mpath_c[0]:%s" % repr(prev_mpath_c[0])
        #print p
        (p1, p2) = prev_mpath_c[0]
        th0, v0 = p1[2], p1[3]
        #print "get_smooth_path #2"
        mpath_close.append((p1, p2))
        #print "get_smooth_path #3"
        intm = get_intermediates((p1, p2), ppath.ptype, ENV)
        #print "get_smooth_path #4, intm:%s" % repr(intm)
        intmv = [(x, y, z, v0) for (x, y, z) in intm]
        #print "get_smooth_path #5, intmv:%s" % repr(intmv)
        if len(mpath_wpoints) == 0:
            #print "get_smooth_path #6"
            mpath_wpoints += intmv
        #print "get_smooth_path #7, mpath_wpoints:%s" % repr(mpath_wpoints)
        #elif intmv[0][:3] == mpath_wpoints[-1][:3]:
        #    mpath_wpoints += intmv[1:]
        #else:
        #    mpath_wpoints += intmv
        return (mpath_close, mpath_wpoints)
    (p2, p3) = ppath.mpath[-1]
    if p2 != p3 and mpath_close[-1][1][:3] == p2[:3]:
        mpath_close.append((p2, p3))
        v0 = p2[-1]
        intm = get_intermediates((p2, p3), ppath.ptype, ENV)
        intmv = [(x, y, z, v0) for (x, y, z) in intm]
        if intmv[0][:3] == mpath_wpoints[-1][:3]:
            mpath_wpoints += intmv[1:]
        else:
            mpath_wpoints += intmv
    elif p2 == p3 and mpath_close[-1][1] == p3:
        pass    # then, we just did a do_smoth.
    else:
        #p = "p2 != p3 and mpath_close[-1][1] == p3, "
        #p += "wpoints[-1]:%s, p3:%s, " % (repr(mpath_wpoints[-1]), repr(p3))
        #p += "mpath_close[-1][1]:%s" % (repr(mpath_close[-1][1]))
        #print p
        pass
        # p = "|mpath|:%d, " % len(ppath.mpath)
        # p += "|_close|:%d, " % len(mpath_close)
        # p += "|_wpoints|:%d, \n" % len(mpath_wpoints)
        # p += "p2, p3 from mpath: %s\n" % repr(ppath.mpath[-1])
        # p += "p2, p3 from _close: %s\n" % repr(mpath_close[-1])
        # for wp in mpath_wpoints[-10:]:
        #     p += "wp:%s\n" % repr(wp)
        # print p
        # raise BaseException, "We have to check on mpath, _close, and _wpoints"
    assert len(ppath.mpath) == len(mpath_close),"|ppath.mpath|!=|mpath_close|"
    _u = None
    for u in mpath_wpoints:
        assert len(u) == 4
        assert u != _u
        _u = u
    #print "get_smooth_path #2, mpath_close[0]:%s" % repr(mpath_close[0])
    return (mpath_close, mpath_wpoints)

def create_partial_path(path, cnode, times, ENV):       #{{{2
    ### check if cnode is in the correct tid which is matched with times.
    cnode_tid = get_tid(cnode.nvalue, ENV)
    (uid, vid) = path[times]
    if uid != cnode_tid:
        raise LookupError

    ### get the mpoints, starting from cnode.nvalue
    p0 = path[times:]
    mplast = (cnode.nvalue[:2], cnode.nvalue[:2])
    mp0 = get_mpoints2(mplast, p0, ENV)

    ### generate midpath from mp0 and cnode's heading
    wpath, ppath = [], PartialPath()
    ppath.times = times
    mp1 = [(cnode.nvalue, cnode.nvalue)]
    for i in range(len(mp0)):
        ((x1, y1), (x2, y2)) = mp0[i]
        (_0, (_1, _2, th1, _4)) = mp1[-1]
        th2 = atan2(y2 - y1, x2 - x1)
        if x1 == x2 and y1 == y2:
            th2, v1, v2, ptype = th1, 0, 0, 0
        elif fabs3(th1, th2) <= pi * 0.5:
            v1, v2, ptype = 1, 1, 1
        else:
            th2, v1, v2, ptype = rev_th(th2), -1, -1, -1
        mu, mv = (x1, y1, th1, v1), (x2, y2, th2, v2)
        (uid, vid) = p0[i]
        assert uid == get_tid(mu, ENV) and vid == get_tid(mv, ENV)
        if ppath.ptype == -2:   # for the initial ppath
            ppath.ptype = ptype
        elif ppath.ptype != ptype:  # then ptype was just changed.
            ### get smooth_path
            _close, _wpoints = get_smooth_path(ppath, [ppath.mpath[0]], ENV)
            ppath.mpath_close, ppath.wpoints = _close, _wpoints

            ### get borders and lpolys
            t2b = get_borders7(ppath, ENV)
            ppath.t2b = convert_to_lpolies(t2b)
            wpath.append(ppath)

            ppath = PartialPath()
            ppath.ptype = ptype
            ppath.times = times + i
        #p = "i:%d, (%d, %d), " % (i, uid, vid)
        #p += "mu(%3.1f, %3.1f, %6.4f, %3.1f), " % mu
        #p += "mv(%3.1f, %3.1f, %6.4f, %3.1f)" % mv
        #print p
        ppath.add_nodes((uid, vid), (mu, mv))
        mp1[-1] = (_0, (_1, _2, th1, v1))
        mp1.append((mu, mv))

    ### get smooth_path
    _close, _wpoints = get_smooth_path(ppath, [ppath.mpath[0]], ENV)
    ppath.mpath_close, ppath.wpoints = _close, _wpoints
    #for wp in _wpoints[:12]:
    #    print "=> wp(%3.1f, %3.1f, %6.4f, %3.1f)" % wp

    ### get borders and lpolys
    t2b = get_borders7(ppath, ENV)
    ppath.t2b = convert_to_lpolies(t2b)
    wpath.append(ppath)
    return wpath

def adjust_time_step(prev_goal, ppath, times, tx, ENV):         #{{{2
    tid = get_tid(prev_goal, ENV)
    assert times >= ppath.times
    (mu, mv) = ppath.mpath_close[times - ppath.times]
    if tid != get_tid(mu, ENV) and tid != get_tid(mv, ENV):
        (_, mw) = ppath.mpath_close[(times + 1) - ppath.times]
        if tid == get_tid(mw, ENV):
            times += 1
            return times + 1
        else:
            raise UnboundLocalError
    if ppath.wpoints.index(ppath.gpoints[0]) >= ppath.wpoints.index(mv):
        return times + 1
    else:
        if tx.close_node is None:
            return times
        _, cnode = get_first_lgoal_path(tx, ppath.gpoints[0])
        pwpoints = get_partial_wpoints(ppath, cnode, ENV)
        #dist_table = get_dist_table(pwpoints, cnode)
        dist_table = get_dist_table2(pwpoints, cnode, ppath.ptype)
        _0, _1, _2, wp, _4 = dist_table[0]
        dist_mu, dist_mv = distance_2d(wp, mu), distance_2d(wp, mv)
        if dist_mu > dist_mv and dist_mv < 5:
            return times + 1
        return times

def get_tid(nvalue, ENV):                               #{{{2
    tids = ENV.tg.get_near_triangle_id(nvalue[:2])
    if len(tids) == 0:
        raise LookupError
    else:
        _, tid = list(tids)[0]
    return tid


def at_target(prev_goal, wpath, ppath, ENV):            #{{{2
    if len(wpath) > 0:
        return False
    cid = get_tid(prev_goal, ENV)
    (uid, vid) = ppath.path[-1]
    if cid not in [uid, vid]:
        return False
    mu, mv = ppath.mpath_close[-1]
    if prev_goal == mv:
        p = "Goal Reached !! prev_goal:(%s), " % repr(prev_goal)
        p += "mpath_close[-1][1]:(%s)" % repr(ppath.mpath_close[-1][1])
        print p
        return True
    return False

def get_partial_wpoints(ppath, cnode, ENV):                 #{{{2
    #print "get_partial_wpoints #1"
    mpclose = []
    cnode_tid = get_tid(cnode.nvalue, ENV)
    #print "get_partial_wpoints #2, cnode_tid:%d" % cnode_tid
    for i, (uid, vid) in enumerate(ppath.path):
        if uid == cnode_tid:
            mpclose = ppath.mpath_close[max(i-1, 0):min(i+5, len(ppath.path))]
            break
        elif vid == cnode_tid and i == len(ppath.path) - 1:
            mpclose = ppath.mpath_close[max(i-1, 0):min(i+5, len(ppath.path))]
            break
    assert len(mpclose) > 0
    #print "get_partial_wpoints #3, |mpclose|:%d" % len(mpclose)
    try:
        wpsidx = ppath.wpoints.index(mpclose[0][0])
    except ValueError:
        #print "mpclose[0][0]: (%6.4f, %6.4f, %10.8f, %3.1f)" % mpclose[0][0]
        print "mpclose[0][0]: (%s)" % repr(mpclose[0][0])
        for wp in ppath.wpoints[:36]:
            #p = "  wp(%6.4f, %6.4f, %10.8f, %3.1f)" % wp
            p = "  wp(%s)" % repr(wp)
            if wp == mpclose[0][0]:
                p += " \"EQUAL\""
            else:
                (x0, y0, th0, v0) = mpclose[0][0]
                (x1, y1, th1, v1) = wp
                if isclose(x0, x1, atol = 0.0001) and \
                    isclose(y0, y1, atol = 0.0001) and \
                    isclose(th0, th1, atol = 0.00001) and \
                    isclose(v0, v1, atol = 0.0001):
                        p += " \"ALMOST EQUAL!!\""
            print p
        raise ValueError
    #print "get_partial_wpoints #3, wpsidx:%d" % wpsidx
    try:
        wpeidx = ppath.wpoints.index(mpclose[-1][1])
    except ValueError:
        #print "mpclose[-1][1]: (%6.4f, %6.4f, %10.8f, %3.1f)" % mpclose[-1][1]
        print "mpclose[-1][1]: (%s)" % repr(mpclose[-1][1])
        for wp in ppath.wpoints[-36:]:
            #p = "  wp(%6.4f, %6.4f, %10.8f, %3.1f)" % wp
            p = "  wp(%s)" % repr(wp)
            if wp == mpclose[-1][1]:
                p += " \"EQUAL\""
            else:
                (x0, y0, th0, v0) = mpclose[-1][1]
                (x1, y1, th1, v1) = wp
                if isclose(x0, x1, atol = 0.0001) and \
                    isclose(y0, y1, atol = 0.0001) and \
                    isclose(th0, th1, atol = 0.00001) and \
                    isclose(v0, v1, atol = 0.0001):
                        p += " \"ALMOST EQUAL!!\""
            print p
        raise ValueError
    #print "get_partial_wpoints #3, wpeidx:%d" % wpeidx
    assert wpsidx < wpeidx
    #p = "get_partial_wpoints #4, wps:(%s), " % repr(ppath.wpoints[wpsidx])
    #p += "wpe(%s)" % repr(ppath.wpoints[wpeidx])
    #print p
    return ppath.wpoints[wpsidx:wpeidx + 1]

def setup_localgoals_future6(ppath, cnode, ENV):            #{{{2
    #print "setup_localgoals_future6 #1, cnode:%s" % repr(cnode)
    pwpoints = get_partial_wpoints(ppath, cnode, ENV)
    dist_table = get_dist_table2(pwpoints, cnode, ppath.ptype)
    if nearbygoal2(pwpoints, dist_table, cnode, 5.0):
        if pwpoints[-1] in ppath.gpoints:
            raise StopIteration
        gp1 = pwpoints[-1]
        return [gp1]

    #print "setup_localgoals_future6 #2, |pwpoints|:%d, -1:%s" % (len(pwpoints), repr(pwpoints[-1]))
    goal_idx, goal_pt, toofar, min_thd1 = -1, None, False, float("Inf")
    for (dist, thd1, thd2, wp, j) in dist_table:
        if dist < 5.0:
            continue
        if thd1 > ENV.MAX_STEER * 4:
            continue
        if dist > ENV.EPSILON * 2:
            if goal_pt is None:
                toofar = True
            break
        if thd1 < min_thd1:
            min_thd1 = thd1
            goal_pt = wp
            goal_idx = j

    #print "setup_localgoals_future6 #3, goal_pt:%s" % repr(goal_pt)
    if toofar:
        for (dist, thd1, thd2, wp, j) in dist_table:
            p = "d:%3.1f, thd1:%6.4f, thd2:%6.4f, " % (dist, thd1, thd2)
            p += "(%3.1f, %3.1f, %6.4f, %4.2f), " % wp
            p += "j:%d" % j
            print p
            if dist > ENV.EPSILON * 3:
                break
        raise LookupError

    if goal_pt is None:         # when thd1 > ENV.MAX_STEER * 3
        print "goal_pt is None, so it should raise UnboundLocalError"
        for (dist, thd1, thd2, wp, j) in dist_table:
            p = "d:%3.1f, thd1:%6.4f, thd2:%6.4f, " % (dist, thd1, thd2)
            p += "(%3.1f, %3.1f, %6.4f, %4.2f), " % wp
            p += "j:%d" % j
            print p
            if dist > ENV.EPSILON * 3:
                break
        raise UnboundLocalError

    gp1 = goal_pt
    cnode1 = Node3D(gp1)

    #print "setup_localgoals_future6 #4, cnode1:%s" % repr(cnode1)
    pwpoints = get_partial_wpoints(ppath, cnode1, ENV)
    #print "setup_localgoals_future6 #4.1, |pwpoints|:%d, -1:%s" % (len(pwpoints), repr(pwpoints[-1]))
    #dist_table = get_dist_table(pwpoints, cnode1)
    dist_table = get_dist_table2(pwpoints, cnode1, ppath.ptype)
    #print "setup_localgoals_future6 #4.2, dist_table[0]:%s" % repr(dist_table[0])
    if nearbygoal2(pwpoints, dist_table, cnode1, 5.0):
        #print "setup_localgoals_future6 #4.3, gp1:%s" % repr(gp1)
        if pwpoints[-1] == gp1:
            return [gp1]
        gp2 = pwpoints[-1]
        return [gp1, gp2]

    #print "setup_localgoals_future6 #4.4"
    goal2_idx, goal2_pt, toofar, min_thd1 = -1, None, False, float("Inf")
    for (dist, thd1, thd2, wp, j) in dist_table:
        if dist < 5.0:
            continue
        if thd1 > ENV.MAX_STEER * 4:
            continue
        if dist > ENV.EPSILON * 2:
            if goal2_pt is None:
                toofar = True
            break
        if thd1 < min_thd1:
            min_thd1 = thd1
            goal2_pt = wp
            goal2_idx = j

    #print "setup_localgoals_future6 #5, goal2_pt:%s" % repr(goal2_pt)
    if toofar:
        if pwpoints[-1] == gp1:
            return [gp1]
        for (dist, thd1, thd2, wp, j) in dist_table:
            p = "d:%3.1f, thd1:%6.4f, thd2:%6.4f, " % (dist, thd1, thd2)
            p += "(%3.1f, %3.1f, %6.4f, %4.2f), " % wp
            p += "j:%d" % j
            print p
            if dist > ENV.EPSILON * 3:
                break
        raise LookupError

    if goal2_pt is None:        # when thd1 > ENV.MAX_STEER * 3
        #print "setup_localgoals_future6 #6"
        if pwpoints[-1] == gp1:
            return [gp1]
        for (dist, thd1, thd2, wp, j) in dist_table:
            p = "d:%3.1f, thd1:%6.4f, thd2:%6.4f, " % (dist, thd1, thd2)
            p += "(%3.1f, %3.1f, %6.4f, %4.2f), " % wp
            p += "j:%d" % j
            print p
        # NOTE: not possible
        raise UnboundLocalError

    gp2 = goal2_pt

    #print "setup_localgoals_future6 #7, gpoints:%s" % repr([gp1, gp2])
    return [gp1, gp2]

def get_dist_table2(wpoints, cnode, ptype):             #{{{2
    dist_table = []
    assert ptype == 1 or ptype == -1
    for j, wp in enumerate(wpoints):
        dist = distance_2d(wp, cnode.nvalue)
        (x0, y0, th0), (x1, y1, th1) = wp[:3], cnode.nvalue[:3]
        if ptype == 1:
            th01 = atan2(y0 - y1, x0 - x1)
        elif ptype == -1:
            th01 = rev_th(atan2(y0 - y1, x0 - x1))
        else:
            raise LookupError
        th_diff1 = fabs3(th01, th1)
        th_diff2 = fabs3(th01, th0)
        th_diff = fabs3(wp[2], cnode.nvalue[2])
        dist_table.append((dist, th_diff1, th_diff2, wp, j))
    dist_table.sort()
    return dist_table

def nearbygoal2(wpoints, dist_table, cnode, near_distance):     #{{{2
    nearbygoal = True
    _0, _1, _2, cnode_near, j = dist_table[0]   # j is the index in wpoint
    for dist, _1, _2, wp, i in dist_table:
        if dist >= 5.0 and i >= j:  # i is the index in wpoints
            nearbygoal = False
    return nearbygoal

def rotate((x0, y0), (x1, y1), theta):                  #{{{2
    # mid-point (x0, y0)
    d01 = distance_2d((x0, y0), (x1, y1))
    th01 = atan2(y1 - y0, x1 - x0)
    x01 = x0 + d01 * cos(th01 + theta)
    y01 = y0 + d01 * sin(th01 + theta)
    return (x01, y01)

def rerotate((x0, y0), (x1, y1), theta):                #{{{2
    # mid-point (x0, y0)
    d01 = distance_2d((x0, y0), (x1, y1))
    th01 = atan2(y1 - y0, x1 - x0)
    x01 = x0 + d01 * cos(th01 - theta)
    y01 = y0 + d01 * sin(th01 - theta)
    return (x01, y01)

def get_theta(p1, p2, p3):                              #{{{2
    mp13 = get_mpoint(p1, p3)
    th_mp13_2 = atan2(mp13[1] - p2[1], mp13[0] - p2[0])
    theta = pi * 0.5 - th_mp13_2
    return theta

def get_mpoint((x0, y0), (x1, y1)):                     #{{{2
    th10 = atan2(y1 - y0, x1 - x0)
    d01 = distance_2d((x0, y0), (x1, y1))
    x01, y01 = x0 + d01 * 0.5 * cos(th10), y0 + d01 * 0.5 * sin(th10)
    return (x01, y01)

def do_smooth_helper(p1, p2, p3, moffset):              #{{{2
    theta = get_theta(p1, p2, p3)
    p1_ = rotate(p2, p1, theta)
    p3_ = rotate(p2, p3, theta)

    assert p1_[0] != p3_[0], "it shouldn't be equal!"
    if p1_[0] < p3_[0]:
        plist = [p1_, p2, p3_]
    else:
        plist = [p3_, p2, p1_]

    xp, yp = [x for x, _ in plist], [y for _, y in plist]
    xval = linspace(min(xp), max(xp), 30)
    yintp = interp(xval, xp, yp)

    sigma = 10
    gaued = gau2(xval, yintp, sigma, 'nearest')
    xp2, yp2 = gaued
    plist2_ = [(xp2[i], yp2[i]) for i in range(len(xp2))]

    plist2 = []
    for px in plist2_:
        px_ = rerotate(p2[:2], px, theta)
        plist2.append(px_)

    cweight = 0.0
    mplist = [plist2[0]]
    for i in range(len(plist2) - 1):
        u, v = plist2[i:i+2]
        d_uv = distance_2d(u, v)
        cweight += d_uv
        if cweight >= moffset:
            cweight = 0.0
            mplist.append(v)

    if plist[0] == p3_:
        mplist2 = [p3] + mplist + [p1]
        mplist2.reverse()
    else:
        mplist2 = [p1] + mplist + [p3]
    return mplist2

def do_smooth(p1, p2, p3, moffset):                     #{{{2
    d12_ = distance_2d(p1, p2)
    d23_ = distance_2d(p2, p3)
    th12 = atan2(p1[1] - p2[1], p1[0] - p2[0])
    th32 = atan2(p3[1] - p2[1], p3[0] - p2[0])
    d12 = d12_ * 0.5 if d12_ < moffset[0] * 2 else moffset[0]
    d23 = d23_ * 0.5 if d23_ < moffset[0] * 2 else moffset[0]
    p1_ = (p2[0] + d12 * cos(th12), p2[1] + d12 * sin(th12))
    p3_ = (p2[0] + d23 * cos(th32), p2[1] + d23 * sin(th32))
    mplist = do_smooth_helper(p1_, p2[:2], p3_, moffset[1])
    #mplist = do_smooth_orig(p1_, p2, p3_, moffset[1])

    return [p1[:2]] + mplist + [p3[:2]]

def gau2(xv, yv, sigma, mode):                          #{{{2
    xv2 = gau(xv, sigma=sigma, mode=mode)
    yv2 = gau(yv, sigma=sigma, mode=mode)
    return (xv2, yv2)

def convert_to_tx3d(txdict, rootnid):                   #{{{2
    rootelm = txdict[rootnid]
    rootnode = Node3D(rootelm[0])
    tx3d = Tree3D(rootnode)
    h = rootelm[3]
    h = [(rootnode, child_nid) for child_nid in rootelm[3]]
    while len(h) > 0:
        (unode, cnid) = h.pop(0)
        (nvalue, (cweight, tweight), pnid, cnids, ctrl, DIR) = txdict[cnid]
        vnode = Node3D(nvalue)

        ### add_tree operation
        unode.children.add(vnode)
        vnode.parent = unode
        vnode.cweight = unode.cweight + tweight
        assert ctrl is not None
        vnode.ctrl = ctrl

        ### add_node operation
        (x, y) = nvalue[:2]
        tx3d.nodes[(x, y)].add(vnode)

        h += [(vnode, ccnid) for ccnid in cnids]

    ### add kdtree
    tx3d.kdt = kdtree.create(tx3d.nodes.keys(), dimensions = 2)

    ### rebalance
    tx3d.kdt = tx3d.kdt.rebalance()

    return tx3d

def get_borders7(ppath, ENV):                           #{{{2
    comb = itertools.combinations
    V = ENV.tg.V
    t2b = defaultdict(set)
    if ppath.ptype == 0:
        (s0, _) = ppath.path[0]
        t2b[s0] = set()
        return t2b
    for i in range(len(ppath.path)):
        if len(ppath.path[i:]) > 1:
            (s0, s1), (_, s2) = ppath.path[i:i+2]
            bs0, bs1 = ENV.tg.get_borders_id(s0), ENV.tg.get_borders_id(s1)
            bs2 = ENV.tg.get_borders_id(s2)
            t2b[s0] = add_border(bs1 | bs2, s0, t2b[s0])
            t2b[s1] = add_border(bs0 | bs2, s1, t2b[s1])
        else:
            (s0, s1) = ppath.path[i]
            bs0, bs1 = ENV.tg.get_borders_id(s0), ENV.tg.get_borders_id(s1)
            t2b[s0] = add_border(bs1, s0, t2b[s0])
            t2b[s1] = add_border(bs0, s1, t2b[s1])
    #print "t2b.keys():%s" % repr(t2b.keys())
    #print "t2b:%s" % repr(t2b)
    return t2b

def add_border(borders, vid, t2b_vid):                  #{{{2
    for (p1, p2) in borders | ENV.tg.get_borders_id(vid):
        if (p1, p2) in t2b_vid or (p2, p1) in t2b_vid:
            continue
        else:
            t2b_vid.update(set([(p1, p2)]))
    return t2b_vid

def get_first_lgoal_path(tx, gpoint0):                  #{{{2
    # NOTE: it returns some path from tx.root to a near node by gpoints[0]
    #cursor = tx.closer_node
    cursor = tx.close_node
    close_dist = distance_2d(cursor.nvalue, gpoint0)
    nprev_goal = cursor

    ### get the next prev_goal
    while cursor != tx.root:
        dist = distance_2d(cursor.nvalue, gpoint0)
        if dist < close_dist:
            nprev_goal = cursor
            close_dist = dist
        cursor = cursor.parent

    ### extract the path from the nprev_goal
    dpath = []
    cursor = nprev_goal
    while cursor != tx.root and len(dpath) < len(tx):
        dpath = [(cursor.parent, cursor)] + dpath
        cursor = cursor.parent
    return dpath, nprev_goal

#---Planning--------------------------------------------{{{1
def drive_to_localgoals4(ppath, cnode, ENV):            #{{{2
    ### init tree
    Xnode = Node3D(cnode.nvalue) # initialize the root node
    tx = Tree3D(Xnode)

    ### build the tree
    #tx = build_the_tree4(tx, ppath, ENV)
    tx = build_the_tree5(tx, ppath, ENV)

    if tx.close_node is None:
        return (False, tx)
    return (True, tx)

def build_the_tree5(tx, ppath, ENV):                    #{{{2
    lpolys = get_border_polys2(tx.root, ppath, ENV)
    if ppath.ptype == 1:
        tx3d = ENV.tx3d
    elif ppth.ptype == -1:
        tx3d = ENV.tx3drev
    else:
        raise LookupError
    (x0, y0, th0, v0) = tx.root.nvalue
    if len(ppath.gpoints) == 1:
        gp1 = ppath.gpoints[0]
        print "gp1:%s" % repr(gp1)
        p1 = tx.root.nvalue
        nlist1 = get_closer_nodes5(tx3d, p1, gp1, 10)
        print "|nlist1|:%d" % len(nlist1)
        h1 = []
        for n in nlist1:
            x1, y1, th1, _ = n.nvalue
            #x2, y2 = x0 + x1, y0 + y1
            (x2, y2, th2) = transform3(p1, (x1, y1, th1))
            thdiff = fabs3(th2, gp1[2])
            heapq.heappush(h1, (thdiff, n))
        found = False
        while len(h1) > 0:
            thdiff, n = heapq.heappop(h1)

            path = tx3d.compute_path(n)
            cweight = 0.0

            ### collision check
            for j, (_, vnode) in enumerate(path):
                (x1, y1, th1, v2) = vnode.nvalue
                (x2, y2, th2) = transform3(p1, (x1, y1, th1))
                cweight += vnode.tweight
                if cweight < 3.0 and j < len(path) - 1:
                    continue
                cweight -= 3.0
                if collide((x2, y2, th2), lpolys, ENV):
                    break
                if j == len(path) - 1:
                    found = True
            if found:
                u = tx.root
                for _, vnode in path:
                    (x1, y1, th1, v2) = vnode.nvalue
                    (x2, y2, th2) = transform3(p1, (x1, y1, th1))
                    child = Node3D((x2, y2, th2, v2))
                    tx.add_tree(u, child, vnode.ctrl, vnode.tweight)
                    u = child
                tx.close_node = child
                return tx
        if not found:
            draw_lpolys(ENV.SCREEN, lpolys, ENV)
            print "not found"
            raise BaseException
    elif len(ppath.gpoints) == 2:
        gp1, gp2 = ppath.gpoints
        print "gp1:%s" % repr(gp1)
        print "gp2:%s" % repr(gp2)
        p1 = tx.root.nvalue
        nlist1 = get_closer_nodes5(tx3d, p1, gp1, 10)
        print "|nlist1|:%d" % len(nlist1)
        h1 = []
        for n in nlist1:
            x1, y1, th1, _ = n.nvalue
            (x2, y2, th2) = transform3(p1, (x1, y1, th1))
            thdiff = fabs3(th2, gp1[2])
            heapq.heappush(h1, (thdiff, n))
        h2 = []
        visited = defaultdict(dict)
        while len(h1) > 0:
            _, n1 = heapq.heappop(h1)
            (x1, y1, th1, v2) = n1.nvalue
            (x2, y2, th2) = transform3(p1, (x1, y1, th1))
            nlist2 = get_closer_nodes5(tx3d, (x2, y2, th2), gp2, 10)
            for n2 in nlist2:
                (x3, y3, th3, v4) = n2.nvalue
                (x4, y4, th4) = transform3((x2, y2, th2), (x3, y3, th3))
                if (x4, y4, th4) in visited:
                    continue
                visited[(x4, y4, th4)] = None
                thdiff = fabs3(th4, gp2[2])
                heapq.heappush(h2, (thdiff, n1, n2))
        print "|h2|:%d" % len(h2)
        found1, found2 = False, False
        itr1 = 0
        col_cnt1 = 0
        col_cnt2 = 0
        while len(h2) > 0:
            _, n1, n2 = heapq.heappop(h2)

            try:
                path1 = tx3d.compute_path(n1)
            except AttributeError:
                print "==> path1, AttributeError, n1:%s <==" % repr(n1)
                cursor = n1
                while cursor is not None and cursor != tx.root:
                    p = "csr.p:%s, csr:%s" % (repr(cursor.parent),repr(cursor))
                    print p
                    cursor = cursor.parent
                continue
            try:
                path2 = tx3d.compute_path(n2)
            except AttributeError:
                print "==> path2, AttributeError, n2:%s <==" % repr(n2)
                cursor = n2
                while cursor is not None and cursor != tx.root:
                    p = "csr.p:%s, csr:%s" % (repr(cursor.parent),repr(cursor))
                    print p
                    cursor = cursor.parent
                continue

            ### collision check
            cweight = 0.0
            found1 = False
            for j, (_, vnode) in enumerate(path1):
                (x1, y1, th1, v2) = vnode.nvalue
                (x2, y2, th2) = transform3(p1, (x1, y1, th1))
                cweight += vnode.tweight
                if cweight < 3.0 and j < len(path1) - 1:
                    continue
                cweight -= 3.0
                if collide((x2, y2, th2), lpolys, ENV):
                    col_cnt1 += 1
                    break
                if j == len(path1) - 1:
                    found1 = True
            if not found1:
                continue
            cweight = 0.0
            found2 = False

            (x1, y1, th1, v2) = n1.nvalue
            (x2, y2, th2) = transform3(p1, (x1, y1, th1))
            #(x3, y3, th3, v4) = n2.nvalue
            #(x4, y4, th4) = transform3((x2, y2, th2), (x3, y3, th3))

            for j, (_, vnode) in enumerate(path2):
                (x3, y3, th3, v4) = vnode.nvalue
                (x4, y4, th4) = transform3((x2, y2, th2), (x3, y3, th3))
                cweight += vnode.tweight
                if cweight < 3.0 and j < len(path2) - 1:
                    continue
                cweight -= 3.0
                if collide((x4, y4, th4), lpolys, ENV):
                    col_cnt2 += 1
                    break
                if j == len(path2) - 1:
                    found2 = True
            if not found2:
                continue
            u = tx.root
            for _, vnode in path1:
                (x1, y1, th1, v2) = vnode.nvalue
                (x2, y2, th2) = transform3(p1, (x1, y1, th1))
                child = Node3D((x2, y2, th2, v2))
                tx.add_tree(u, child, vnode.ctrl, vnode.tweight)
                u = child
            (x1, y1, th1, v2) = n1.nvalue
            (x2, y2, th2) = transform3(p1, (x1, y1, th1))
            for _, vnode in path2:
                (x3, y3, th3, v4) = vnode.nvalue
                (x4, y4, th4) = transform3((x2, y2, th2), (x3, y3, th3))
                child = Node3D((x4, y4, th4, v4))
                tx.add_tree(u, child, vnode.ctrl, vnode.tweight)
                u = child
            tx.close_node = child
            return tx
        if not found1 or not found2:
            draw_lpolys_debug(ENV.SCREEN, lpolys, 2, ENV)
            for i in range(len(ppath.wpoints) - 1):
                wp1, wp2 = ppath.wpoints[i:i+2]
                draw_line_2d(ENV.SCREEN, wp1[:2], wp2[:2])
                (x6, y6) = wp1[:2]
                pygame.draw.circle(ENV.SCREEN,(255,0,0),(int(x6),int(y6)),2)
                pygame.display.update()
            pygame.draw.circle(ENV.SCREEN,ENV.C['red'],(int(x0),int(y0)),2)
            pygame.draw.circle(ENV.SCREEN,ENV.C['limegreen'],(int(x2),int(y2)),2)
            pygame.draw.circle(ENV.SCREEN,ENV.C['dodgerblue'],(int(x4),int(y4)),2)
            pygame.display.update()
            print "not found, itr1:%d, c1_cnt:%d, c2_cnt:%d" % (itr1, col_cnt1, col_cnt2)
            raise BaseException
    else:
        draw_lpolys(ENV.SCREEN, lpolys, ENV)
        print "there should be some gpoints:%s" % repr(ppath.gpoints)
        raise BaseException


def get_border_polys2(u, ppath, ENV):                   #{{{2
    tids = ENV.tg.get_near_triangle_id(u.nvalue[:2])
    if len(tids) == 0:
        raise BaseException
    else:
        _, u_tid = list(tids)[0]
    #u_tid = u.tid
    try:
        lpolys = ppath.t2b[u_tid]
    except KeyError:
        #u.errors += 1
        print "%d is not in ppath.t2b:%s" % (u_tid, repr(ppath.t2b.keys()))
        u_tid = ppath.path[0][0]
        lpolys = ppath.t2b[u_tid]
        #raise KeyError, "%d is not in ppath.t2b:%s" % (u_tid, repr(ppath.t2b))
        #return tx
    neighbors = set(ENV.tg.neighbors_id(u_tid))
    neighbors.intersection_update(set(ppath.t2b.keys()))
    for nid in neighbors:
        lpolys.update(ppath.t2b[nid])
    return lpolys

def get_closer_nodes5(tx3d, p1, p2, kth):               #{{{2
    (x3, y3, th3) = global_to_local2(p1, p2)
    nn1 = tx3d.kdt.search_knn((x3, y3), kth)
    nlist = []
    for n, _ in nn1:
        nlist += list(tx3d.nodes[n.data])
    return nlist

def stop_motion(tx, cnode, ppath, ENV):                 #{{{2
    speed = cnode.nvalue[-1]
    if ppath.ptype == -1:
        if speed == -0.4:
            tx, cnode = add_slow_down4(tx, cnode, ppath, -0.16, ENV)
        tx, cnode = add_stop_motion4(tx, cnode, ENV)
    elif ppath.ptype == 1:
        if speed == 0.4:
            tx, cnode = add_slow_down4(tx, cnode, ppath, 0.16, ENV)
        tx, cnode = add_stop_motion4(tx, cnode, ENV)
    else:
        if speed != 0:
            raise UnboundLocalError
    return tx, cnode

def speed_up(tx, cnode, ppath, ENV):                    #{{{2
    speed = cnode.nvalue[-1]
    if ppath.ptype == -1:
        if speed == 0:
            tx, cnode = add_speed_up4(tx, cnode, -0.16, ppath, ENV)
        elif speed == -0.16:
            tx, cnode = add_speed_up4(tx, cnode, -0.4, ppath, ENV)
    elif ppath.ptype == 1:
        if speed == 0:
            tx, cnode = add_speed_up4(tx, cnode, 0.16, ppath, ENV)
        elif speed == -0.16:
            tx, cnode = add_speed_up4(tx, cnode, 0.4, ppath, ENV)
    return tx, cnode

def add_stop_motion4(tx, Xnode, ENV):                   #{{{2
    ### get the current speed
    (x, y, th, speed) = Xnode.nvalue

    if speed == 0.0:    # if already stopped, just return it.
        return tx, Xnode

    ### find the proper cset from the current speed to the 0.0
    cset = ENV.cset1[(speed, 0.0)]

    ### add the cset to the Xnode's child
    state, ctrl, lsteps, cweight = get_children2(th, cset)
    (x1, y1, th2, v2) = state
    x2, y2 = x + x1, y + y1
    w = Node3D((x2, y2, th2, v2))
    w.ctrl = ctrl
    w.tweight = cweight

    try:
        tx.add_tree(Xnode, w, w.ctrl, w.tweight)
    except BaseException:
        raise BaseException

    ### return tx with the child node
    return tx, w

def add_speed_up4(tx, Xnode, target_speed, ppath, ENV): #{{{2
    ### get the current speed
    (x, y, th, current_speed) = Xnode.nvalue

    if current_speed == target_speed:   # then, no need to speed up.
        return tx, Xnode

    ### find the proper cset from the current speed to the 0.0
    cset = ENV.cset1[(current_speed, target_speed)]

    ### add the cset to the Xnode's child
    state, ctrl, lsteps, cweight = get_children2(th, cset)
    (x1, y1, th2, v2) = state
    x2, y2 = x + x1, y + y1
    w = Node3D((x2, y2, th2, v2))
    w.ctrl = ctrl
    w.tweight = cweight

    try:
        tx.add_tree(Xnode, w, w.ctrl, w.tweight)
    except BaseException:
        raise BaseException

    ### return tx with the child node
    return tx, w

def add_slow_down4(tx, Xnode, ppath, target_speed, ENV): #{{{2
    ### get the current speed
    (x, y, th, current_speed) = Xnode.nvalue

    if current_speed == target_speed:   # then, no need to speed up.
        return tx, Xnode

    ### find the proper cset from the current speed to the 0.0
    cset = ENV.cset1[(current_speed, target_speed)]

    ### add the cset to the Xnode's child
    state, ctrl, lsteps, cweight = get_children2(th, cset)
    (x1, y1, th2, v2) = state
    x2, y2 = x + x1, y + y1
    w = Node3D((x2, y2, th2, v2))
    w.ctrl = ctrl
    w.tweight = cweight

    try:
        tx.add_tree(Xnode, w, w.ctrl, w.tweight)
    except BaseException:
        raise BaseException

    ### return tx with the child node
    return tx, w

#---Drawing---------------------------------------------{{{1
def draw_path_2d(screen, tree):
    #screen.fill((0,0,0))
    for (u, v) in tree.path:
        draw_line_2d_red(screen, u.nvalue, v.nvalue)
    pygame.display.update()

def draw_line_2d(screen, (x1, y1), (x2, y2), update=False):
    global ENV
    DIR_COLOR = (255, 255, 20)    # yellow
    pygame.draw.line(screen, DIR_COLOR, (int(x1), int(y1)), (int(x2), int(y2)))
    if update:
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

def draw_line_2d_yellow(screen, (x1, y1), (x2, y2), update=False):
    global ENV
    DIR_COLOR = (255, 255, 0)   # yellow
    pygame.draw.line(screen, DIR_COLOR, (int(x1), int(y1)), (int(x2), int(y2)), 3)
    if update:
        pygame.display.update()

def draw_lpolys(screen, lpolys, ENV):
    DIR_COLOR = (255, 255, 0)   # yellow
    for lpoly in lpolys:
        nodes = [(int(x), int(y)) for (x, y) in lpoly.P]
        assert len(nodes) == 2
        p1, p2 = nodes
        pygame.draw.line(screen, DIR_COLOR, p1, p2, 6)
    pygame.display.update()

def draw_traj_iter(lgoal_path, SCREEN, ENV):            #{{{2
    # NOTE: we will draw from tx.root to a near node by ppath.gpoints[0]
    (x, y, th, _) = lgoal_path[0][0].nvalue
    color = ENV.C['red']
    clear_and_draw_base(SCREEN, ENV, ENV.colors)
    vehicle.draw_temp_car2(SCREEN, (x, y, th), color)
    pygame.display.update()
    #print "lgoal_path:%s" % repr(lgoal_path)
    for n1, n2 in lgoal_path:
        #print "---- ----"
        #print "n1:%s" % repr(n1)
        #print "n2:%s" % repr(n2)
        #print "n2.ctrl:%s" % repr(n2.ctrl)
        (x2, y2, th2, _) = n2.nvalue
        clear_and_draw_base(SCREEN, ENV, ENV.colors)
        vehicle.draw_temp_car2(SCREEN, (x2, y2, th2), color)
        draw_line_2d_blue(SCREEN, (x, y), (x2, y2), update=True)
        #handle_quit_event2(0.01)
        handle_quit_event2(0.001)
        x, y = x2, y2
        (x0, y0, th0, v0) = n1.nvalue
        (x1, y1, th1, v1) = n2.nvalue
        acc, steer = n2.ctrl
        p = "(%3.1f, %3.1f, %3.1f, %3.1f)-" % (x0, y0, th0, v0)
        p += "[%3.1f,%3.1f]->" % (acc, steer)
        p += "(%3.1f, %3.1f, %3.1f, %3.1f)" % (x1, y1, th1, v1)
        print p
    #handle_quit_event2(0.001)

def draw_traj_iter2(tx, nnode, SCREEN, ENV):            #{{{2
    ### get the path from tx and nnode
    cursor = nnode
    dpath = []
    while tx.root != cursor and len(dpath) < len(tx):
        #dpath += [(cursor.parent, cursor)]
        dpath = [(cursor.parent, cursor)] + dpath
        cursor = cursor.parent

    ### draw the path
    draw_traj_iter(dpath, SCREEN, ENV)

def draw_lpolys_debug(screen, lpolys, timeout, ENV):
    clear_and_draw_base(SCREEN, ENV, ENV.colors)
    pygame.display.update()
    draw_lpolys(screen, lpolys, ENV)
    handle_quit_event2(timeout)

def handle_quit_event2(timeout = 0.5):
    tick = time.time()
    while time.time() - tick < timeout:
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                pygame.quit()
                sys.exit(0)
        pygame.display.update()
        time.sleep(0.001)
        
def clear_and_draw_base(screen, ENV, colors):           #{{{2
    screen.fill((0, 0, 0))
    for index, nodes in enumerate(ENV.obstacles):
        pygame.draw.polygon(screen, colors[index % (len(colors) - 1)], nodes, 0)

    ENV.tg.draw_triangles3(screen)

#---Main Codes------------------------------------------{{{1
def run_lane_follower4(SCREEN, ENV):                    #{{{2
    ### initial global route
    path = ENV.tg.get_shortest(ENV.car.src[:2], ENV.car.dest[:2], True)
    tids = ENV.tg.get_near_triangle_id(ENV.car.src[:2])
    gtids = ENV.tg.get_near_triangle_id(ENV.car.dest[:2])
    (_, tid), (__, gtid) = tids[0], gtids[0]
    cnode = Node3D(ENV.car.src)

    ### find the partial paths
    times = 0
    wpath = create_partial_path(path, cnode, times, ENV)
    ppath = wpath.pop(0)
    prev_ppaths = []

    ### simulate start
    tick = time.time()
    tx = Tree3D(cnode)
    tx, Xnode = simulate_start3(tx, cnode, ppath, ppath.ptype, ENV)
    tx, cnode = speed_up(tx, cnode, ppath, ENV)
    pwpoints = get_partial_wpoints(ppath, cnode, ENV)
    dist_table = get_dist_table2(pwpoints, cnode, ppath.ptype)
    _0, _1, _2, wp, _4 = dist_table[0]
    ppath.gpoints = [wp]
    prev_goal = wp

    itr = 0

    ### drive the parital path until reaching the global goal
    while not at_target(prev_goal, wpath, ppath, ENV):
        times = adjust_time_step(prev_goal, ppath, times, tx, ENV)
        print "== prev_goal:(%3.1f,%3.1f,%6.4f,%4.2f) ==" % prev_goal
        print "path[%d:%d+3]:%s" % (times, times, repr(path[times:times+3]))
        try:
            ppath.gpoints = setup_localgoals_future6(ppath, cnode, ENV)
        except StopIteration:
            tx, cnode = stop_motion(tx, cnode, ppath, ENV)
            if len(wpath) == 0:
                raise StopIteration
            wpath.pop(0)
            ppath = wpath[0]
            tx, cnode = speed_up(tx, cnode, ppath, ENV)
            prev_goal = ppath.wpoints[0]
            continue
        except UnboundLocalError:
            raise UnboundLocalError
        except LookupError:
            raise LookupError
        except BaseException:
            print "couldn't setup the localgoals future"
            raise BaseException

        ### 2. drive to the local goal
        succeed, tx = drive_to_localgoals4(ppath, cnode, ENV)

        ### 3. if succeeded, repeat again.
        if succeed:
            prev_goal = ppath.gpoints[0]
            lgoal_path, cnode = get_first_lgoal_path(tx, prev_goal)
            draw_traj_iter(lgoal_path, SCREEN, ENV)
            itr += 1
        else:
            print "couldn't get the drive_to_localgoals"
            raise BaseException
    tx, cnode = stop_motion(tx, cnode, ppath, ENV)
    print "It took %06.3f sec. itr:%d" % (time.time() - tick, itr)
    print "final position: (%s)" % repr(cnode.nvalue)
    print "global destination: (%s)" % repr(ENV.car.dest)

#---Global Codes----------------------------------------{{{1
ENV = load_map("exenv8.svg")
ENV.MIN_X, ENV.MIN_Y = 0, 0
ENV.MAX_X, ENV.MAX_Y = ENV.map_size
ENV.V_MAX = 1.6
#ENV.dt = 0.2
#ENV.L = 6
ENV.L = 12.0
ENV.EPSILON = 10.0
ENV.D_MAX = 10
ENV.depth_max = 100
ENV.depth_limit = 70
#ENV.depth_limit = 5
#ENV.max_iter = 10
#ENV.max_iter = 500
ENV.max_iter = 20000
ENV.max_citer = 3
ENV.rrt_cutoff = 1000
ENV.goal_chance = 0.05
#ENV.goal_near = 7.0
ENV.goal_near = 5.0
#ENV.goal_near = 4.0
#ENV.goal_near = 2.0
#ENV.goal_near = 10.0
ENV.V_THRESHOLD = 0.001
ENV.MAX_NSTEP = 20
ENV.MAX_ACCEL = 0.3
#ENV.MAX_STEER = pi/5.0
#ENV.MAX_STEER = pi/8.0
ENV.MAX_STEER = pi/12.0
ENV.safe_sim_distance = 2   # This if for 3D distance (x, y, th)
#ENV.lambda_steps = 10
ENV.lambda_steps = 20
ENV.dt = 0.3
ENV._1d = 2 * pi / 360              # 1 degree
ENV.car = Environment()
#(s1, _) = ENV.kdt_cg.search_nn((167.083, 65.0))
#(s1, _) = ENV.kdt_cg.search_nn((37.5, 51.6))
#(d1, _) = ENV.kdt_cg.search_nn((362.5, 51.6))
#(s1, _) = ENV.tg.get_near_triangle((37.5, 51.6)) # <- original one exenv2
#(s1, _) = ENV.tg.get_near_triangle((270.4, 65.0)) # <- middle one
(s1, _) = ENV.tg.get_near_triangle((131.0, 63.0)) # <- exenv8
#(d1, _) = ENV.tg.get_near_triangle((362.5, 51.6)) # <- original one exenv2
(d1, _) = ENV.tg.get_near_triangle((537.0, 181.0)) # <- exenv8
#ENV.car.src = (s1.data[0], s1.data[1], -pi / 2)
#ENV.car.dest = (d1.data[0], d1.data[1], -pi / 2)
#ENV.car.src = (s1.data[0], s1.data[1], 0.0, 0.0)
#ENV.car.dest = (d1.data[0], d1.data[1], 0.0, 0.0)
#ENV.car.src = (s1[0], s1[1], 0.0, 0.0)
#ENV.car.src = (s1[0], s1[1], 0.0 + ENV._1d * random(), 0.0) # <- orig. exenv2
ENV.car.src = (s1[0], s1[1], 0.5816, 0.0)      # <- exenv8
ENV.car.dest = (d1[0], d1[1], -1.4959, 0.0)    # <- exenv8
#ENV.car.dest = (d1[0], d1[1], 0.0, 0.0)
ENV.A4 = ENV.car.src
#s1 = (50, 50)
#d1 = (100, 100)
#ENV.car.src = s1
#ENV.car.dest = d1
ENV.itr1_csr = -1
ENV.itr2_csr = -1


ENV.MAX_TREE_SIZE = 1000000
ENV.actions = [(0, 1), (0, 0), (2, 1), (2, 0), (1, 1), (1, 0)]
#ENV.A, ENV.B, ENV.C_ = 1.0, 5.0, 30.0
ENV.A, ENV.B, ENV.C_ = 1.0, 5.0, 5.0
ENV.car.vmax = ENV.V_MAX * 0.1
ENV.car.amax = ENV.MAX_ACCEL
ENV.car.steer_max = ENV.MAX_STEER
ENV.car.L = ENV.L
ENV.car.threshold = ENV.V_THRESHOLD
#ENV.car.dt = ENV.dt
ENV.car.dt = 0.1
ENV.car.nsteps = 50
ENV.nsteps = 50
ENV.car.model_type = 0  # 0 for simple, 1 for kinematic, 2 for kinodynamic

ENV.colors = [(255, 238, 170), (233, 198, 175), (136, 170, 0), (0, 0, 0)]

ENV.C = {}
ENV.C['limegreen'] = (50, 205, 50)
ENV.C['lime'] = (0,255,0)
ENV.C['forestgreen'] = (34,139,34)
ENV.C['green'] = (0,128,0)
ENV.C['red'] = (255,0,0)
ENV.C['dodgerblue'] = (30,144,255)
ENV.C['cherry'] = (196,0,0)
ENV.C['kryloncherry'] = (121,6,4)
ENV.C['oranged'] = (255,69,0)
ENV.C['gold'] = (255,215,9)
ENV.C['orange'] = (255,169,0)
ENV.C['darkorange'] = (255,140,0)
ENV.C['skyblue'] = (135,206,250)
ENV.C['deepskyblue'] = (0,191,255)
ENV.C['yellow'] = (255, 255, 0)
ENV.C['darkyellow'] = (204, 204, 0)
ENV.C['lightyellow'] = (255, 255, 153)
ENV.C['khaki'] = (240, 230, 140)
ENV.C['chocolate'] = (210, 105, 30)

# rfile = "lattice4.py."
# #rfile = "lattice2.py."
# #rfile = "lattice3.py."
# #rfile = "lattice.py."
# #rev = 0
# #rev = 2
# #rev = 19
# #rev = 30
# rev = 1
# if os.path.isfile(rfile+str(rev)):
#     pass
# else:
#     raise BaseException, "%s%s doesn't exist." % (rfile, rev)
# rfile_ds = open(rfile+str(rev), 'rb')
# children_set = pickle.load(rfile_ds)
# rfile_ds.close()
# ENV.cset = children_set
# #print ENV.cset
# #sys.exit(1)
# 
# rfile2 = "lattice5.py."
# rev2 = 8
# 
# if os.path.isfile(rfile2+str(rev2)):
#     pass
# else:
#     raise BaseException, "%s%s doesn't exist." % (rfile2, rev2)
# rfile2_ds = open(rfile2+str(rev2), 'rb')
# children_set2 = pickle.load(rfile2_ds)
# rfile2_ds.close()
# ENV.cset2 = children_set2

# rfile = "primitives1.py."
# rev = 1
# if os.path.isfile(rfile + str(rev)):
#     pass
# else:
#     raise BaseException, "%s%s doesn't exist." % (rfile, rev)
# rfile_ds = open(rfile + str(rev), 'rb')
# cset = pickle.load(rfile_ds)
# rfile_ds.close()
# ENV.cset = cset

rfile1 = "primitive2.py.0"
if not os.path.isfile(rfile1):
    raise BaseException, "%s doesn't exist." % rfile1
rfile1_ds = open(rfile1, 'rb')
cset1 = pickle.load(rfile1_ds)
rfile1_ds.close()
ENV.cset1 = cset1

rfile2 = "grid_primitives.py.0"
if not os.path.isfile(rfile2):
    raise BaseException, "%s doesn't exist." % rfile2
rfile2_ds = open(rfile2, 'rb')
cset2 = pickle.load(rfile2_ds)
rfile2_ds.close()
ENV.cset2 = cset2

tick = time.time()
print "loading local_points..."
rfile4a = "local_points3.py.2"
if not os.path.isfile(rfile4a):
    raise BaseException, "%s doesn't exist." % rfile4a
rfile4a_ds = open(rfile4a, 'rb')
tx_lt = pickle.load(rfile4a_ds)
rfile4a_ds.close()

print "done loading local_points in %08.3f sec" % (time.time() - tick)

print "converting local_points to tx3d dict..."
tick = time.time()
txdict, rootnid = tx_lt
ENV.tx3d = convert_to_tx3d(txdict, rootnid)
print "done converting in %08.3f sec" % (time.time() - tick)

rfile4b = "local_points4.py.0"
if not os.path.isfile(rfile4b):
    raise BaseException, "%s doesn't exist." % rfile4a
rfile4b_ds = open(rfile4b, 'rb')
tx_lt_rev = pickle.load(rfile4b_ds)
rfile4b_ds.close()

txdict, rootnid = tx_lt_rev
ENV.tx3drev = convert_to_tx3d(txdict, rootnid)

if __name__ == '__main__':
    pygame.init()
    SCREEN = pygame.display.set_mode(ENV.map_size) # load screen
    ENV.SCREEN = SCREEN

    clear_and_draw_base(SCREEN, ENV, ENV.colors)

    pygame.display.update()

    run_lane_follower4(SCREEN, ENV)

    while True:
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                pygame.quit()
                sys.exit(1)

# modelines {{{1
# vim:fdm=marker:fdl=0:
# vim:foldtext=getline(v\:foldstart).'...'.(v\:foldend-v\:foldstart):
# #vim:nofen
