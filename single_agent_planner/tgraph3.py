from collections import defaultdict
from random import randint
from math import fabs, tan, sin, cos, pi, sqrt, atan2
from pylygon import Polygon
import kdtree
import heapq
import itertools
import pygame
from numpy import ones, vstack, isclose
from numpy.linalg import lstsq
from lineequ5 import is_intersected, on_borders
import copy

class Graph(object):
    def __init__(self):
        self.V = {}
        self.E = defaultdict(dict)
        self.cidmap = defaultdict(count_factory(1)) # coord to id map
        #self.cidmap = {}    # coord to id map
        self.cbmap = defaultdict(set)   # coord to border map
        self.idbmap = defaultdict(set)  # id to border map
        self.idlmap = defaultdict(set)  # id to line poly map
        self.idflsmap = defaultdict(set)   # id to free linesegs map
        self.kdt_cg = kdtree.create([], dimensions = 2)

    def add_node(self, (cx, cy), points):
        if (cx, cy) in self.cidmap:
            raise KeyError

        self.cidmap[(cx, cy)]
        uid = self.cidmap[(cx, cy)]

        # while True:
        #     uid = randint(0, 999)
        #     if uid not in self.V:
        #         break
        for vid, (c, p_set) in self.V.iteritems():
            if len(set(points).intersection(p_set)) == 2:
                dist = self.distance_2d((cx, cy), c)
                self.E[uid][vid] = dist
                self.E[vid][uid] = dist
        self.V[uid] = ((cx, cy), set(points))
        #self.cidmap[(cx, cy)] = uid
        comb = itertools.combinations
        self.idflsmap[uid] = set([edge for edge in comb(points, 2)])
        self.kdt_cg.add((cx, cy))

    def neighbors(self, (cx, cy), selfloop = False):
        try:
            vid = self.cidmap[(cx, cy)]
        except KeyError:
            raise KeyError
        return self.neighbors_id(vid, selfloop)

    def neighbors_id(self, vid, selfloop = False):
        if selfloop:
            return [vid] + self.E[vid].keys()
        else:
            return self.E[vid].keys()

    def degree(self, (cx, cy)):
        try:
            vid = self.cidmap[(cx, cy)]
        except KeyError:
            raise KeyError
        return self.degree_id(self, vid)

    def degree_id(self, vid):
        return len(self.E[vid].keys())

    def add_borders(self, (cx, cy), pset):
        vid = self.cidmap[(cx, cy)]
        self.add_borders_id(vid, pset)

    def add_borders_id(self, vid, pset):
        for (p1, p2) in pset:
            self.add_border_id(vid, (p1, p2))

    def add_border_id(self, vid, (p1, p2)):
        #coord, _ = self.V[vid]
        #self.cbmap[coord]
        borders = self.idbmap[vid]
        #if (p1, p2) not in self.idbmap[vid] and (p2, p1) not in self.idbmap[vid]:
            #self.idlmap[vid].add(Polygon([p1, p2]))
        if (p1, p2) not in borders and (p2, p1) not in borders:
            self.idlmap[vid].add(Polygon([p1, p2]))
        #self.idbmap[vid].add((p1, p2))
        borders.add((p1, p2))
        fls_removed = set()
        for (p3, p4) in self.idflsmap[vid]:
            (ps1, ps2) = ([p1, p2], [p3, p4])
            if on_borders(ps1, ps2) or on_border(ps1, ps2):
                fls_removed.add((p3, p4))
        self.idflsmap[vid] = self.idflsmap[vid] - fls_removed
        #self.idlmap[vid].add(Polygon([p1, p2]))

    def add_border(self, (cx, cy), (p1, p2)):
        vid = self.cidmap[(cx, cy)]
        self.add_border_id(vid, (p1, p2))

    def get_borders(self, (cx, cy)):
        vid = self.cidmap[(cx, cy)]
        return self.get_borders_id(vid)

    def get_borders_id(self, vid):
        return self.idbmap[vid]

    def get_linepolies(self, (cx, cy)):
        vid = self.cidmap[(cx, cy)]
        print "=> vid:",vid
        return self.get_linepolies_id(vid)

    def get_linepolies_id(self, vid):
        return self.idlmap[vid]

    def distance_2d(self,p1, p2):
        (x1, y1) = (list(p1)[0], list(p1)[1])
        (x2, y2) = (list(p2)[0], list(p2)[1])
        return sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def get_near_borders(self, (cx, cy), radius):
        nn = self.kdt_cg.search_nn_dist((cx, cy), radius * radius)
        nodes = [node.data for node in nn]
        neighbors = set()
        for node in nodes:
            neighbors = neighbors.union(set(self.neighbors(node)))
        return [ps for vid in neighbors for ps in self.get_borders_id(vid)]

    def get_near_borders_lpoly(self, (cx, cy), radius):
        #print "get_near_borders_lpoly #1"
        nn = self.kdt_cg.search_nn_dist((cx, cy), radius * radius)
        nodes = [node.data for node in nn]
        neighbors = set()
        for node in nodes:
            neighbors = neighbors.union(set(self.neighbors(node)))
        #lpolies = []
        lpolies = set()
        #for node in nodes:
        for vid in neighbors:
            #print "=> node:",node
            linepolies = self.get_linepolies_id(vid)
            #print "=> linepolies:",linepolies
            #for lpoly in self.get_linepolies(node):
            lpolies = lpolies.union(set(linepolies))
            # for lpoly in linepolies:
            #     lpolies.append(lpoly)
        return lpolies

    def get_near_triangle(self, (cx, cy)):
        nn, _ = self.kdt_cg.search_nn((cx, cy))
        return nn.data, self.V[self.cidmap[nn.data]][1]

    def get_near_triangle_id(self, (cx, cy)):
        #points = self.kdt_cg.search_knn((cx, cy), 3)
        points = self.kdt_cg.search_knn((cx, cy), 6)
        near_t_ids = []
        for (nn, nn_dist) in points:
            vid = self.cidmap[nn.data]
            centroid, ps = self.V[vid]
            poly = Polygon(list(ps))
            collided = poly.collidepoint((cx, cy))
            if collided == 1:
                near_t_ids.append((nn_dist, vid))
                return near_t_ids
            elif collided == -1:
                near_t_ids.append((nn_dist, vid))
                near_t_ids.sort()
        return near_t_ids

    def draw_triangles(self, screen):
        for vid, ((cx,cy), (t1, t2, t3)) in self.V.iteritems():
            nodes = [t1, t2, t3, t1]
            nodes2 = [(int(x), int(y)) for (x, y) in nodes]
            pygame.draw.polygon(screen, (255, 255, 255), nodes2, 1)
            pygame.draw.circle(screen, (255, 0, 255), (int(cx), int(cy)), 2)

    def draw_triangles2(self, screen):
        for vid, ((cx,cy), (t1, t2, t3)) in self.V.iteritems():
            nodes = [t1, t2, t3, t1]
            nodes2 = [(int(x), int(y)) for (x, y) in nodes]
            pygame.draw.polygon(screen, (255, 255, 255), nodes2, 1)
            free_lsegs = []
            for (p1, p2) in itertools.combinations([t1, t2, t3], 2):
                is_border = False
                for (p3, p4) in self.get_borders_id(vid):
                    (ps1, ps2) = ([p3, p4], [p1, p2])
                    if on_borders(ps1, ps2) or on_border(ps1, ps2):
                        is_border = True
                        break
                if not is_border:
                    free_lsegs.append((p1, p2))
            for edge in free_lsegs:
                (x1, y1) = get_midpoint(edge)
                pygame.draw.circle(screen, (255, 0, 255), (int(x1), int(y1)), 2)

    def draw_triangles3(self, screen):
        for vid, ((cx,cy), (t1, t2, t3)) in self.V.iteritems():
            nodes = [t1, t2, t3, t1]
            nodes2 = [(int(x), int(y)) for (x, y) in nodes]
            pygame.draw.polygon(screen, (80, 80, 80), nodes2, 1)
            #pygame.draw.polygon(screen, (255, 255, 255), nodes2, 1)
            #for edge in self.idflsmap[vid]:
            #    (x1, y1) = get_midpoint(edge)
            #    pygame.draw.circle(screen, (255, 0, 255), (int(x1), int(y1)), 2)

    def draw_borders(self, screen, (cx, cy), radius):
        for (p1, p2) in self.get_near_borders((cx, cy), radius):
            coord1 = (int(round(p1[0])), int(round(p1[1])))
            coord2 = (int(round(p2[0])), int(round(p2[1])))
            pygame.draw.line(screen, (255, 0, 0), coord1, coord2, 6)

    def get_shortest_midpath(self, s, t, screen):
        (s_n, _) = self.get_near_triangle(s)
        (t_n, _) = self.get_near_triangle(t)
        path = self.get_shortest(s_n, t_n, True)
        assert(len(path) > 0)
        #mpoints = self.get_midpoints(s, t, path)
        mpoints = self.get_midpoints2(s, t, path, screen)
        return [(mpoints[i],mpoints[i+1]) for i in range(len(mpoints)-2)]

    def get_coop_midpath(self, s, t, path, screen):
        mpoints = self.get_midpoints2(s, t, path, screen)
        #mpoints = self.get_midpoints3(s, t, path, screen)
        return [(mpoints[i],mpoints[i+1]) for i in range(len(mpoints)-2)]

    def get_shortest_path(self, s, t):
        (s_n, _) = self.get_near_triangle(s)
        (t_n, _) = self.get_near_triangle(t)
        path = self.get_shortest(s_n, t_n)
        assert(len(path) > 0)
        (sn0, sn1), (tn1, tn0) = path[0], path[-1]

        d_s_sn0, d_s_sn1 = self.distance_2d(s, sn0), self.distance_2d(s, sn1)
        d_tn1_t, d_tn0_t = self.distance_2d(tn1, t), self.distance_2d(tn0, t)
        if d_s_sn0 + self.distance_2d(sn0, sn1) > d_s_sn1:
            path = [(s, sn1)] + path[1:]
        else:
            path = [(s, sn0)] + path
        if self.distance_2d(tn1, tn0) + d_tn0_t > d_tn1_t:
            path = path[:-1] + [(tn1, t)]
        else:
            path = path + [(tn0, t)]
        return path

    def get_shortest(self, s, t, vid_return = False):
        h, visited, p, path, found = [], set(), defaultdict(dict), [], False
        dist = defaultdict(inf_factory(float("Inf")))
        sid, tid = self.cidmap[s], self.cidmap[t]
        visited.add(sid)
        #print "get_shortest((%04.1f,%04.1f):%d, (%04.1f,%04.1f))"% (s[0], s[1], sid, t[0], t[1])
        #print "ns:",self.neighbors_id(sid)
        for v in self.neighbors_id(sid):
            heapq.heappush(h, (self.E[sid][v], sid, v))
        #print "|h|:%d,%s"% (len(h), repr(h))
        itr = 0
        while len(h) > 0:
            dist_v, u, v = heapq.heappop(h)
            #print "itr:%d, |h|:%d, d:%04.1f, u:%d, v:%d" % (itr, len(h), dist_v, u, v)
            itr += 1
            if v == tid:
                p[v] = u
                dist[v] = dist_v
                found = True
                break
            if v not in visited:
                visited.add(v)
                p[v] = u
                dist[v] = dist_v
                for w in self.neighbors_id(v):
                    heapq.heappush(h, (dist_v + self.E[v][w], v, w))
            elif dist_v < dist[v]:
                p[v] = u
                dist[v] = dist_v
                for w in self.neighbors_id(v):
                    heapq.heappush(h, (dist_v + self.E[v][w], v, w))

        #print "found:",found
        if found:
            cursor = tid
            if vid_return:
                while cursor != sid:
                    path = [(p[cursor], cursor)] + path
                    cursor = p[cursor]
            else:
                while cursor != sid:
                    path = [(self.V[p[cursor]][0], self.V[cursor][0])] + path
                    cursor = p[cursor]
        return path

    def compute_shortest_reverse(self, t):
        h, visited, p, path = [], set(), defaultdict(dict), []
        dist = defaultdict(inf_factory(float("Inf")))
        tid = self.cidmap[t]
        visited.add(tid)
        #p[tid] = tid
        for v in self.neighbors_id(tid):
            heapq.heappush(h, (self.E[v][tid], v, tid))
        while len(h) > 0:
            dist_u, u, v = heapq.heappop(h)
            if u not in visited:
                visited.add(u)
                p[u] = v
                dist[u] = dist_u
                for w in self.neighbors_id(u):
                    heapq.heappush(h, (dist_u + self.E[w][u], w, u))
            elif dist_u < dist[u]:
                p[u] = v
                dist[u] = dist_u
                for w in self.neighbors_id(u):
                    heapq.heappush(h, (dist_u + self.E[w][u], w, u))

        return p, dist

    def get_path(self, p, s, t):
        print "get_path, s:%d, t:%d" % (s, t)
        path, cursor = [], s
        if cursor == t:
            path = path + [(s, t)]
        itr = 0
        while cursor != t:
            path = path + [(cursor, p[cursor])]
            cursor = p[cursor]
            itr += 1
            if itr > len(p):
                for key, value in enumerate(p):
                    print "    k:%d -> v:%s" % (key, repr(value))
                #for key, value in p.iteritems():
                #    print "   k:%d -> v:%d" % (key, value)
                msg = "itr:%d exceeded |p|:%d, " % (itr, len(p))
                msg += "path:%s" % (repr(path))
                raise BaseException, msg
        return path

    def get_midpoints2(self, s, t, path, screen):
        midpoints = []
        for i, (u, v) in enumerate(path):
            a = set(self.idflsmap[u])
            b = set([(p2, p1) for (p1, p2) in a])
            common_edges = (a.union(b)).intersection(self.idflsmap[v])
            if len(common_edges) != 1:
                #raise BaseException, "common_edges:",common_edges
                print "get_midpoints2: BaseException, commoon_edges:%s" % repr(common_edges)
                if len(common_edges) == 0:
                    (x, y) = self.V[u][0]
                    pygame.draw.circle(screen, (255, 0, 0), (int(x), int(y)), 2)
                    (x, y) = self.V[v][0]
                    pygame.draw.circle(screen, (255, 0, 0), (int(x), int(y)), 2)
                    pygame.display.update()
                    raise BaseException
                #for (p1, p2) in common_edges:
                #    pygame.draw.line(screen, (255, 0, 0), (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), 2)
                #pygame.display.update()
            midpoints.append(get_midpoint(list(common_edges)[0]))

        #midpoints2 = []
        #for i in range(len(midpoints) - 2):
        #    (x1, y1), (x2, y2) = midpoints[i], midpoints[i+1]
        #    (x3, y3) = midpoints[i+2]
        #    th1, th2 = atan2(y2 - y1, x2 - x1), atan2(y3 - y2, x3 - x2)
        #    if fabs(th1 - th2) < pi / 2.0:
        #        midpoints.append((x1, y1))

        # midpoints2, index = [], 0
        # while index < len(midpoints) - 2:
        #     (x1, y1) = midpoints[index]
        #     (x2, y2) = midpoints[index+1]
        #     (x3, y3) = midpoints[index+2]
        #     th1, th2 = atan2(y2 - y1, x2 - x1), atan2(y3 - y2, x3 - x2)
        #     th1_2 = atan2(y3 - y1, x3 - x1)
        #     if fabs(th1 - th2) < pi / 2.0:
        #         midpoints2.append((x1, y1))
        #         midpoints2.append((x2, y2))
        #     else:
        #         if fabs(th1_2 - th2) <= fabs(th1 - th2):
        #             midpoints2.append((x1, y1))
        #             midpoints2.append((x3, y3))
        #     index += 2

        (x1, y1), (x2, y2), (x3, y3) = s, midpoints[0], midpoints[1]
        th1, th2 = atan2(y2 - y1, x2 - x1), atan2(y3 - y2, x3 - x2)
        if fabs(th1 - th2) > pi / 2.0:
            (x4, y4) = midpoints[2]
            th3 = atan2(y4 - y3, x4 - x3)
            th1_3 = atan2(y3 - y1, x3 - x1)
            if fabs(th1_3 - th3) <= pi / 2.0:
                midpoints = [s] + midpoints[1:]
            elif fabs(th1 - th2) >= fabs(th1_3 - th3):
                midpoints = [s] + midpoints[1:]
            else:
                midpoints = [s] + midpoints
        else:
            midpoints = [s] + midpoints
        (x1, y1), (x2, y2), (x3, y3) = t, midpoints[-1], midpoints[-2]
        th1, th2 = atan2(y1 - y2, x1 - x2), atan2(y2 - y3, x2 - x3)
        if fabs(th2 - th1) > pi / 2.0:
            (x4, y4) = midpoints[-3]
            th3 = atan2(y3 - y4, x3 - x4)
            th1_3 = atan2(y1 - y3, x1 - x3)
            if fabs(th1_3 - th3) <= pi / 2.0:
                midpoints = midpoints[:-1] + [t]
            elif fabs(th1 - th2) >= fabs(th1_3 - th3):
                midpoints = midpoints[:-1] + [t]
            else:
                midpoints = midpoints + [t]
        else:
            midpoints = midpoints + [t]

        return midpoints

    def get_midpoints3(self, s, t, path, screen):
        midpoints = []
        for i, (u, v) in enumerate(path):
            a = set(self.idflsmap[u])
            b = set([(p2, p1) for (p1, p2) in a])
            common_edges = (a.union(b)).intersection(self.idflsmap[v])
            if len(common_edges) != 1:
                if len(common_edges) == 0:
                    raise BaseException
            midpoints.append(get_midpoint(list(common_edges)[0]))
        return [s] + midpoints + [t]

    def get_midpoints(self, s, t, path):
        midpoints = []
        first_flseg = set(self.idflsmap[path[0][0]])
        last_flseg = set(self.idflsmap[path[-1][1]])
        for i, (u, v) in enumerate(path):
            a = set(self.idflsmap[u])
            b = set([(p2, p1) for (p1, p2) in a])
            common_edges = (a.union(b)).intersection(self.idflsmap[v])
            if len(common_edges) != 1:
                raise BaseException
            midpoints.append(get_midpoint(list(common_edges)[0]))
            if i == 0:
                d = set([(p2, p1) for (p1, p2) in common_edges])
                first_flseg = first_flseg - (common_edges.union(d))
            elif i == len(path) - 1:
                d = set([(p2, p1) for (p1, p2) in common_edges])
                last_flseg = last_flseg - (common_edges.union(d))


        if len(first_flseg) == 0:
            # Then, there is no other free lineseg. for the first triangle u.
            # So, s must be in the same triangle and
            # we assume that the vehicle can head to midpoints[0] easily.
            # NOTE: We may need to consider the case when the vehicle should
            #   drive back in order to reach the first_flseg.
            midpoints = [s] + midpoints
        else:
            # Then, we have to check if s is in the same triangle or not.
            # If so, we don't need first_flseg.
            (s_n, _) = self.get_near_triangle(s)
            if self.cidmap[s_n] == path[0][0]:
                midpoints = [s] + midpoints
            else:
                # We assume that adding first_flseg
                # between s and midpoints[0] is not much detouring.
                # XXX: How can we check if it is too much or not?
                midpoints = [s] + list(first_flseg) + midpoints

        if len(last_flseg) == 0:
            midpoints = midpoints + [t]
        else:
            (t_n, _) = self.get_near_triangle(t)
            if self.cidmap[t_n] == path[-1][1]:
                midpoints = midpoints + [t]
            else:
                midpoints = midpoints + list(last_flseg) + [t]
                
       #(s_n, _) = self.get_near_triangle(s)
       #(s_t, _) = self.get_near_triangle(t)
       #if s_n != path[0][0] and len(first_flseg) > 0:
       #    midpoints = [s] + list(first_flseg) + midpoints
       #else:
       #    midpoints = [s] + midpoints
       #if t_n != path[-1][1] and len(last_flseg) > 0:
       #    midpoints = midpoints + list(last_flseg) + [t]
       #else:
       #    midpoints = midpoints + [t]

        return midpoints

def count_factory(counter):
    return itertools.count(counter).next

def inf_factory(value):
    return itertools.repeat(value).next

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

def get_m_c(points):
    x_coords, y_coords = zip(*points)
    A = vstack( [x_coords, ones(len(x_coords))] ).T
    m, c = lstsq( A, y_coords )[0]
    return (m, c)
