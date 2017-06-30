import triangle
from triangle.plot import plot
from numpy import *
import matplotlib.pyplot as plt

from numpy import ones, vstack, isclose
from numpy.linalg import lstsq

from lineequ5 import is_intersected, on_borders

from itertools import combinations as comb
from collections import defaultdict
import kdtree
from pylygon import Polygon
import sys

import json
import random

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

def get_centroid(t1, t2, t3):
    ((x1, y1), (x2, y2), (x3, y3)) = (t1, t2, t3)
    return ((x1 + x2 + x3) / 3, (y1 + y2 + y3) / 3)

def get_centroid_3d(t1, t2, t3):
    ((x1, y1, z1), (x2, y2, z2), (x3, y3, z3)) = (t1, t2, t3)
    return ((x1 + x2 + x3) / 3, (y1 + y2 + y3) / 3, (z1 + z2 + z3) / 3)

def generate_tgraph_testcase(inner_point, nodes, edges):
    A = {"vertices": array(nodes), "segments": array(edges)}
    B = triangle.triangulate(A, 'qc')
    Asegs = [set([i, j]) for (i, j) in A['segments']]
    Bcoords = {}
    for i, node in enumerate(B['vertices']):
        Bcoords[i] = node
    Bvt = {}
    cvtmap = {}
    for ti, (u, v, w) in enumerate(B['triangles']):
        Bvt[ti] = set([u, v, w])
        centroid = get_centroid(Bcoords[u], Bcoords[v], Bcoords[w])
        cvtmap[centroid] = ti

    BG2 = defaultdict(dict)
    for (ti, tj) in comb(Bvt.keys(), 2):
        if len(Bvt[ti].intersection(Bvt[tj])) == 2:
            BG2[ti][tj] = 1
            BG2[tj][ti] = 1

    #segments2 = []
    # for ti in BG2.keys():
    #     for tj in BG2[ti].keys():
    #         common_vs = Bvt[ti].intersection(Bvt[tj])
    #         if len(common_vs) < 2:
    #             continue
    #         if common_vs in Asegs:
    #             segments2.append(common_vs)
    #             del BG2[ti][tj]
    #             continue
    #         ps = [Bcoords[i] for i in common_vs]
    #         for aseg in Asegs:
    #             if on_border(aseg, ps) or on_borders(aseg, ps):
    #                 segments2.append(common_vs)
    #                 del BG2[ti][tj]
    #                 break

    for ti in BG2.keys():
        for tj in BG2[ti].keys():
            common_vs = Bvt[ti].intersection(Bvt[tj])
            if len(common_vs) == 2:
                #print "BG2[%d][%d], cvs:%s" % (ti, tj, repr(common_vs))
                if common_vs in Asegs:
                    #print "gtt #1"
                    #if common_vs not in segments2:
                    #    segments2.append(common_vs)
                    del BG2[ti][tj]
                else:
                    #print "gtt #2"
                    ps = tuple([tuple(Bcoords[i]) for i in common_vs])
                    #print "ps:", ps
                    for aseg in Asegs:
                        aseg2 = tuple([tuple(Bcoords[i]) for i in aseg])
                        #print "gtt #3, aseg2:%s" % repr(aseg2)
                        #if on_border(aseg2, ps) or on_borders(aseg2, ps):
                        if on_borders(aseg2, ps):
                            #print "gtt #4"
                        #if on_border(aseg, ps) or on_borders(aseg, ps):
                            #if common_vs not in segments2:
                            #    segments2.append(common_vs)
                            del BG2[ti][tj]
                            break

    #print "BG2.keys():%s" % repr(BG2.keys())
    #print "Asegs:%s" % repr(Asegs)
    #print "segments2:%s" % repr(segments2)

    centroids = cvtmap.keys()
    centroids.sort()
    kdt_cg = kdtree.create(centroids, dimensions = 2)

    # get near centroids from kdtree
    points = kdt_cg.search_knn(inner_point[0], 6)

    near_t_ids, found = [], False
    for (nn, nn_dist) in points:
        ti = cvtmap[nn.data]
        (u, v, w) = Bvt[ti]
        (p1, p2, p3) = Bcoords[u], Bcoords[v], Bcoords[w]
        poly = Polygon([p1, p2, p3])
        collided = poly.collidepoint((inner_point[0]))
        if collided == 1:
            found = True
            near_t_ids = [(nn_dist, ti)] + near_t_ids
            break
        elif collided == -1:
            near_t_ids.append((nn_dist, ti))
            near_t_ids.sort()

    if not found:
        raise BaseException, "not found |near_t_ids|:%d" % (len(near_t_ids))

    G3 = defaultdict(dict)
    visited = set()
    _, ti = near_t_ids[0]
    queue = [(ti, tj) for tj in BG2[ti].keys()]
    while len(queue) > 0:
        (ti, tj) = queue.pop()
        if (ti, tj) not in visited:
            visited.add((ti, tj))
            G3[ti][tj] = 1
            queue = queue + [(tj, tk) for tk in BG2[tj].keys()]

    # return (set(Bvt.keys()) & set(G3.keys()), triangles) from G3
    #
    #print "Bcoords.keys():",Bcoords.keys()
    #nBvt = [tuple(Bcoords[i]) for i in [j for j in Bcoords.keys()]]
    nBvt = {i: tuple(Bcoords[i]) for i in [j for j in Bcoords.keys()]}
    #print "nBvt:",nBvt
    #print "G3.keys():", G3.keys()
    #nTriangles = [tuple(Bvt[ti]) for ti in G3.keys()]
    nTriangles = {ti: tuple(Bvt[ti]) for ti in G3.keys()}
    #print "nTriangles:",nTriangles
    return (nBvt, nTriangles)

def get_tgraph(V, T):
    # 1. get an inner_point from one triangle in T
    #print "V:", V
    #print "T:", T
    #(t1, t2, t3) = [V[i] for i in T[random.sample(T.keys(), 1)[0]]]
    atriangle = T[random.sample(T.keys(), 1)[0]]
    #print "atriangle:", atriangle
    (t1, t2, t3) = [V[i] for i in atriangle]
    inner_point = [get_centroid(t1, t2, t3)]

    # 2. get segments from T
    segments = []
    TG1 = defaultdict(dict)
    for (ti, tj) in comb(T.keys(), 2):
        ti_ps = set(T[ti])
        tj_ps = set(T[tj])
        if len(ti_ps.intersection(tj_ps)) == 2:
            TG1[ti][tj] = 1
            TG1[tj][ti] = 1
    for ti in TG1.keys():
        edges = [set(pair) for pair in comb(T[ti], 2)]
        #print "TG1[%d].keys():%s,edges:%s"%(ti,repr(TG1[ti].keys()),repr(edges))
        for tj in TG1[ti].keys():
            common_edge = set(T[ti]).intersection(set(T[tj]))
            edges.remove(common_edge)
            #if set(T[tj]) in edges:
            #    edges.remove(set(T[tj]))
        if len(edges) > 0:
            #print "TG1[%d][%d]: remained edges:%s" % (ti, tj, repr(edges))
            segments += [tuple(pair) for pair in edges]
            #segments += edges

    # 3. triangulate it
    V_KEYS = V.keys()
    V_KEYS.sort()
    V2 = [V[i] for i in V_KEYS]
    #print "V2:", V2
    segments.sort()
    #segments = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 8), (8, 5), (5, 6), (6, 7), (7, 0)]
    #print "segments:", segments
    A = {"vertices": array(V2), "segments": array(segments)}
    #print "A['vertices']:",A['vertices']
    #print "A['segments']:",A['segments']
    B = triangle.triangulate(A, 'qc')
    Asegs = [set(pair) for pair in A['segments']]
    Bcoords = {}
    for i, node in enumerate(B['vertices']):
        Bcoords[i] = node
    Bvt = {}
    cvtmap = {}
    for ti, (u, v, w) in enumerate(B['triangles']):
        Bvt[ti] = set([u, v, w])
        centroid = get_centroid(Bcoords[u], Bcoords[v], Bcoords[w])
        cvtmap[centroid] = ti

    BG2 = defaultdict(dict)
    for (ti, tj) in comb(Bvt.keys(), 2):
        if len(Bvt[ti].intersection(Bvt[tj])) == 2:
            BG2[ti][tj] = 1
            BG2[tj][ti] = 1

    #for ti in BG2.keys():
    #    for tj in BG2[ti].keys():
    #        common_vs = Bvt[ti].intersection(Bvt[tj])
    #        if len(common_vs) == 2 and common_vs in Asegs:
    #            del BG2[ti][tj]
    segments2 = []
    for ti in BG2.keys():
        for tj in BG2[ti].keys():
            common_vs = Bvt[ti].intersection(Bvt[tj])
            if len(common_vs) == 2:
                if common_vs in Asegs:
                    del BG2[ti][tj]
                else:
                    #ps = tuple([Bcoords[i] for i in common_vs])
                    ps = tuple([tuple(Bcoords[i]) for i in common_vs])
                    for aseg in Asegs:
                        aseg2 = tuple([tuple(Bcoords[i]) for i in aseg])
                        #if on_border(aseg2, ps) or on_borders(aseg2, ps):
                        if on_borders(aseg2, ps):
                            del BG2[ti][tj]
                            break

    centroids = cvtmap.keys()
    centroids.sort()
    kdt_cg = kdtree.create(centroids, dimensions = 2)

    points = kdt_cg.search_knn(inner_point[0], 6)

    near_t_ids, found = [], False
    for (nn, nn_dist) in points:
        ti = cvtmap[nn.data]
        (u, v, w) = Bvt[ti]
        (p1, p2, p3) = Bcoords[u], Bcoords[v], Bcoords[w]
        poly = Polygon([p1, p2, p3])
        collided = poly.collidepoint((inner_point[0]))
        if collided == 1:
            found = True
            near_t_ids = [(nn_dist, ti)] + near_t_ids
            break
        elif collided == -1:
            near_t_ids.append((nn_dist, ti))
            near_t_ids.sort()

    if not found:
        raise BaseException, "not found |near_t_ids|:%d" % (len(near_t_ids))

    # 4. generate tgraph
    TG2 = defaultdict(dict)
    visited = set()
    queue = [(ti, tj) for tj in BG2[ti].keys()]
    while len(queue) > 0:
        (ti, tj) = queue.pop()
        if (ti, tj) not in visited:
            visited.add((ti, tj))
            TG2[ti][tj] = 1
            queue = queue + [(tj, tk) for tk in BG2[tj].keys()]

    segments3 = []
    for ti in TG2.keys():
        edges = [set(pair) for pair in comb(Bvt[ti], 2)]
        for tj in TG2[ti].keys():
            edges.remove(Bvt[ti].intersection(Bvt[tj]))
        segments3 += edges

    #print "segments3:", segments3
    
    # 5. return the nodes2, tgraph2 and segments2
    nBvt = {i: tuple(Bcoords[i]) for i in [j for j in Bcoords.keys()]}
    nTriangles = {ti: tuple(Bvt[ti]) for ti in TG2.keys()}
    nEdges = [(ti, tj) for ti in TG2.keys() for tj in TG2[ti].keys()]
    nSegments = [tuple(pair) for pair in segments3]
    #return (nBvt, nTriangles, nEdges, TG2)
    #return (nBvt, nTriangles, nEdges, segments2)
    return (nBvt, nTriangles, nEdges, nSegments)

    # 6. dump the data (nodes2, vertices2, edges2, segments2)

inner_point = [(6.5, 3.4)]
fs_nodes = [(5, 1), (8, 1), (8, 6), (12, 6), (12, 9), (1, 9), (1, 6), (5, 6)]
fs_edges = [(0,1),(1,2),(2,3),(3,4),(4,5),(5,6),(6,7),(7,0)]

(V, T) = generate_tgraph_testcase(inner_point, fs_nodes, fs_edges)

#print json.dumps({'vertices': V, 'triangles': T})
dumpped = json.dumps({'vertices': V, 'triangles': T})

loaded = json.loads(dumpped)
#V2 = loaded['vertices']
V2 = {int(k):tuple(loaded['vertices'][k]) for k in loaded['vertices'].keys()}
#V2 = {int(k):loaded['vertices'][k] for k in loaded['vertices'].keys()}
#T2 = loaded['triangles']
T2 = {int(k):tuple(loaded['triangles'][k]) for k in loaded['triangles'].keys()}
#T2 = {int(k):loaded['triangles'][k] for k in loaded['triangles'].keys()}

print "loaded:", loaded

#print "V2:", V2
#print "T2:", T2

# get an inner_point from one triangle in T2
# 1) take one triangle in T2
#(t1, t2, t3) = [V2[i] for i in T2[random.sample(T2.keys(), 1)]]
#inner_point = [get_centroid(t1, t2, t3)]
# 2) get segments from T2
#
# 3) call generate_tgraph(inner_point, V2, segments) :'
#    triangulate and take only reachable triangles from inner_point
#

(V3, T3, E3, S3) = get_tgraph(V2, T2)
dumpped2 = json.dumps({'vertices':V3,'triangles':T3,'edges':E3,'segments':S3})

print dumpped2

#
# 4) then dump it(?) or generate tgraph and dump the tgraph(?)
#    It is better to dump the tgraph:
#      * nodes for triangle points (3 three nodes for each triangle)
#      * vertices for indexing triangles; vertex (key): three nodes (value)
#      * edges for connecting between vertices of triangles
#      * segments (for representing the border, connnecting some nodes)
#
#    Based on the above data, we can share a sequence of way points.
