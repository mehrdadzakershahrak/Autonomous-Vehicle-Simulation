# local_search4.py {{{
#
# Description:
#   Given an input of current position and the local goal postion,
#   it returns a list of near positions.
#   Or it returns a path from the current position to the one of 
#   the near positions.
#
# Date: 04-19-2017
#
# Author: Kangjin Kim (kangjin.kim@gail.com)
#
# Usage:
#   $ python local_search4.py
#
# }}}
#


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
#from treetemp2d import Node as Node2D, Tree as Tree2D
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

class NodeLT(object):
    def __init__(self, nvalue, ctrl = None, DIR = 0, weights=(0.0, 0.0)):
        #self.nid = uuid.uuid1()
        self.nid = uuid.uuid1().time_low
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

class TreeLT(object):
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

#---Distance--------------------------------------------{{{1
def distance_2d(p1, p2):
    (x1, y1) = (list(p1)[0], list(p1)[1])
    (x2, y2) = (list(p2)[0], list(p2)[1])
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def get_key(coord):
    (x, y, th) = coord[:3]
    x0 = float(format(x * 0.2, '.1f'))
    y0 = float(format(y * 0.2, '.1f'))
    th0 = float(format(th, '.2f'))
    return (x0, y0, th0)

def get_key2(coord):
    (x, y, th) = coord[:3]
    x0 = float(format(x, '.0f'))
    y0 = float(format(y, '.0f'))
    th0 = float(format(th, '.2f'))
    return (x0, y0, th0)

def get_tree(tx_lt):
    root_node = Node3D(tx_lt.root.nvalue)
    tx3d = Tree3D(root_node)
    h = [(0, tx_lt.root, tx3d.root)]
    while len(h) > 0:
        (dth, u1, u2) = h.pop(0)
        for v1 in u1.children:
            v2 = Node3D(v1.nvalue)

            ### add_tree operation
            if v2.parent is not None:
                raise BaseException
            u2.children.add(v2)
            v2.parent = u2
            v2.cweight = u2.cweight + v1.tweight
            v2.ctrl = v1.ctrl

            ### add_node operation
            (x, y) = v1.nvalue[:2]
            tx3d.nodes[(x, y)].add(v2)

            h.append((dth+1, v1, v2))

    ### add kdtree
    tx3d.kdt = kdtree.create(tx3d.nodes.keys(), dimensions = 2)

    ### rebalance
    tx3d.kdt = tx3d.kdt.rebalance()

    return tx3d

def convert_to_dict(tx_lt):
    tx_dict = {}
    h = [tx_lt.root]
    while len(h) > 0:
        anode = h.pop(0)
        alist = []
        alist.append(anode.nvalue)
        alist.append((anode.cweight, anode.tweight))
        if anode.parent is not None:
            alist.append(anode.parent.nid)
        else:
            alist.append(None)
        if len(anode.children) > 0:
            alist.append([child.nid for child in anode.children])
            h += list(anode.children)
        else:
            alist.append([])
        assert anode.ctrl is not None or anode == tx_lt.root
        alist.append(anode.ctrl)
        alist.append(anode.DIR)
        tx_dict[anode.nid] = alist
    return (tx_dict, tx_lt.root.nid)

def convert_to_tx3d(txdict, rootnid):
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
        vnode.cweight = vnode.cweight + tweight
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

def get_closer(tx, (x1, y1, th1), th21, d1):
    x3 = 0.0 + d1 * cos(th21)
    y3 = 0.0 + d1 * sin(th21)
    nn1 = tx.kdt.search_knn((x3, y3), 5)
    nlist = []
    for n, _ in nn1:
        nlist += list(tx.nodes[n.data])
    nlist2 = []
    for n in nlist:
        (x4, y4, th4, v5) in n.nvalue
        nlist2.append((x1 + x4, y1 + y4, th1 + th4, v5))
    return nlist2

def get_closer(tx, p1, p2, kth):
    (x1, y1, th1), (x2, y2, th2) = p1[:3], p2[:3]
    dist1 = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    th21 = atan2(y2 - y1, x2 - x1)
    x3, y3 = 0.0 + dist1 * cos(th21), 0.0 + dist * sin(th21)
    nn1 = tx.kdt.search_knn((x3, y3), kth)
    nlist = []
    for n, _ in nn1:
        nlist += list(tx.nodes[n.data])
    nlist2 = []
    for n in nlist:
        (x4, y4, th4, v5) = n.nvalue
        nlist2.append((x1 + x4, y1 + y4, th1 + th4, v5))
    return nlist2

def get_closer_nodes(tx, p1, p2, kth):
    (x1, y1, th1), (x2, y2, th2) = p1[:3], p2[:3]
    dist1 = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    th21 = atan2(y2 - y1, x2 - x1)
    x3, y3 = 0.0 + dist1 * cos(th21), 0.0 + dist * sin(th21)
    nn1 = tx.kdt.search_knn((x3, y3), kth)
    nlist = []
    for n, _ in nn1:
        nlist += list(tx.nodes[n.data])
    return nlist

#---Build Lattice Tree----------------------------------{{{1
def build_lattice(tx, cset, max_cweight, ENV):
    h = []
    occupied = defaultdict(set)
    heapq.heappush(h, (0.0, tx.root))
    itr = 0
    cweight_bound = max_cweight * 2
    while len(h) > 0:
        if itr % 500 == 0:
            mincw, _ = h[0]
            p = "itr:%d, |tx|:%d, " % (itr, len(tx))
            p += "|h|:%d, |occupied|:%d, " % (len(h),len(occupied))
            #p += "mincw:%06.4f" % (cweight_bound - mincw)
            p += "mincw:%06.4f" % mincw
            print p
        itr += 1
        cost, node = heapq.heappop(h)
        (x0, y0, th0, v0) = node.nvalue
        thkey = float(format(th0, '.2f'))
        okey = get_key2(node.nvalue)
        #occupied[(x0, y0, th0)].add(node)
        if okey in occupied:
            continue
        occupied[okey].add(node)
        for (x1, y1, th2, v2), x2ctrl, lsteps2, cweight2 in cset[thkey]:
            x2, y2 = x0 + x1, y0 + y1
            okey = get_key2((x2, y2, th2))
            #if (x2, y2, th2) in occupied:
            if okey in occupied:
                continue
            weights = (node.cweight + cweight2, cweight2)
            if weights[0] > max_cweight:
                continue
            #child = Node3D((x2, y2, th2, v2), x2ctrl, weights, node)
            #child = Node3D((x2, y2, th2, v2))
            child = NodeLT((x2, y2, th2, v2))
            tx.add_tree(node, child, x2ctrl, ENV.DIR, cweight2)
            #heapq.heappush(h, (cweight_bound - weights[0], child))
            if th2 == th0:
                ccost = cost + weights[1]
            else:
                ccost = cost + weights[1] + 3
            heapq.heappush(h, (ccost, child))
    return tx

def build_lattice2(tx, cset, max_cweight, ENV):
    h = []
    occupied = defaultdict(set)
    (x0, y0, th0, v0) = tx.root.nvalue
    thkey = float(format(th0, '.2f'))
    for (x1, y1, th2, v2), x2ctrl, lsteps2, cweight2 in cset[thkey]:
        x2, y2 = x0 + x1, y0 + y1
        weights = (tx.root.cweight + cweight2, cweight2)
        child = NodeLT((x2, y2, th2, v2))
        child = NodeLT((x2, y2, th2, v2), x2ctrl, ENV.DIR, weights)
        if th2 == th0:
            ccost = 0.0 + weights[1]
        else:
            ccost = 0.0 + weights[1] + 3
        heapq.heappush(h, (ccost, (tx.root, child)))
    okey = get_key2(tx.root.nvalue)
    #okey = get_key(tx.root.nvalue)
    occupied[okey].add(tx.root)

    itr = 0
    cweight_bound = max_cweight * 2
    while len(h) > 0:
        # if itr % 500 == 0:
        #     mincw, _ = h[0]
        #     p = "itr:%d, |tx|:%d, " % (itr, len(tx))
        #     p += "|h|:%d, |occupied|:%d, " % (len(h),len(occupied))
        #     p += "mincw:%06.4f" % mincw
        #     print p
        itr += 1
        cost, (unode, vnode) = heapq.heappop(h)
        (x0, y0, th0, v0) = vnode.nvalue
        thkey = float(format(th0, '.2f'))
        okey = get_key2(vnode.nvalue)
        #okey = get_key(vnode.nvalue)
        if okey in occupied:
            continue
        occupied[okey].add(vnode)
        tx.add_tree(unode, vnode, vnode.ctrl, vnode.DIR, vnode.tweight)
        for (x1, y1, th2, v2), x2ctrl, lsteps2, cweight2 in cset[thkey]:
            x2, y2 = x0 + x1, y0 + y1
            okey = get_key2((x2, y2, th2))
            #okey = get_key((x2, y2, th2))
            if okey in occupied:
                continue
            weights = (vnode.cweight + cweight2, cweight2)
            if weights[0] > max_cweight:
                continue
            child = NodeLT((x2, y2, th2, v2), x2ctrl, ENV.DIR, weights)
            #child = NodeLT((x2, y2, th2, v2))
            #tx.add_tree(vnode, child, x2ctrl, ENV.DIR, cweight2)
            if th2 == th0:
                ccost = cost + weights[1]
            else:
                ccost = cost + weights[1] + 3
            heapq.heappush(h, (ccost, (vnode, child)))
    return tx

ENV = Environment()
max_cweight = 30
#max_cweight = 10

#rfile3 = "primitive3.py.0"
rfile3 = "primitive4.py.0"
if not os.path.isfile(rfile3):
    raise BaseException, "%s doesn't exist." % rfile3
rfile3_ds = open(rfile3, 'rb')
cset3 = pickle.load(rfile3_ds)
rfile3_ds.close()
ENV.cset3 = cset3
ENV.DIR = 1
ENV._1d = 2 * pi / 360              # 1 degree

#src = (0.0, 0.0, 0.0, 0.4)
src = (0.0, 0.0, 0.0, -0.4)
root = NodeLT(src)

print "1. generating the tree..."
txset = {}
#thrange = range(-180, 179+1)
#thrange = range(-1, 2+1)
thrange = range(0, 0 + 1)
for src_degree in thrange:
    #print "---- ---- ---- ----"
    src_itr = (src[0], src[1], src_degree * ENV._1d, src[3])
    thkey = float(format(src_degree * ENV._1d, '.2f'))
    tick = time.time()
    tx = TreeLT(NodeLT(src_itr))
    tx = build_lattice2(tx, ENV.cset3, max_cweight, ENV)
    (txdict, rootnid) = convert_to_dict(tx)
    #txset[thkey] = tx
    #txset[thkey] = (txdict, rootnid)
    txset = (txdict, rootnid)
    p = "thkey:%07.4f (%04d) |tx|:%d. " % (thkey, src_degree, len(tx))
    p += "|txdict|:%d, rootnid:%s, " % (len(txdict), repr(rootnid))
    p += "It took %010.3f sec. " % (time.time() - tick)
    #p += "%s" % repr(txdict[rootnid])
    #tx3d = convert_to_tx3d(txdict, rootnid)
    #p += "|tx3d|:%d, rootnode:%s" % (len(tx3d), repr(tx3d.root))
    print p
#sys.exit(1)

print "2. dumping (pickling) the set of trees..."
rfile4 = "local_points4.py."
rev4 = 0
while os.path.isfile(rfile4+str(rev4)):
    rev4 += 1
rfile4_ds = open(rfile4+str(rev4), 'wb')
gc.disable()
pickle.dump(txset, rfile4_ds)
gc.enable()
rfile4_ds.close()

print "3. loading (unpickling) the set of trees..."
rfile4a = rfile4+str(rev4)
if not os.path.isfile(rfile4a):
    raise BaseException, "%s doesn't exist." % rfile4a
rfile4a_ds = open(rfile4a, 'rb')
gc.disable()
tx_lt = pickle.load(rfile4a_ds)
gc.enable()
rfile4a_ds.close()

print "4. converting to tx3d set..."
tick = time.time()
txdict, rootnid = tx_lt
txset2 = convert_to_tx3d(txdict, rootnid)
p = "|txset2|:%d. " % (len(txset2))
p += "It took %010.3f sec." % (time.time() - tick)
print p
