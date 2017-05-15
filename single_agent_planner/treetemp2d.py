# treetemp2d.py {{{
# Last Modified: 2016-11-21
# Description: Tree class for 2D (Temporarily)
#   Here we create a new 2D Tree class in order to replace current tree class.
#   This is based on treetemp.py.
# Author: Kangjin Kim
# Usage:
#   $ python treetemp2d.py
# Related codes:
#   single_2d.py
# Prerequired Packages:
#   kdtree, bounded_priority_queue
# }}}

#---Package Imports-------------------------------------{{{1
import uuid
from collections import defaultdict
from math import sqrt
import kdtree

#---TODO------------------------------------------------{{{1o
#---@2016-11-21-----------------------------------------{{{2
#   1. create this file
#   2. copy all from treetemp.py and change it for 2D.

class Node(object):                                     #{{{1
    def __init__(self, nvalue, weights = (0.0, 0.0), parent = None):
        self.nid = uuid.uuid1()
        self.nvalue = nvalue
        self.cweight = weights[0]   # cumulated weight
        self.tweight = weights[1]   # transition duration
        self.parent = parent
        self.children = set()

    def __repr__(self):
        name = self.__class__.__name__
        return "%s(%s)" % (name, repr(self.nvalue))

class Tree(object):                                     #{{{1
    def __init__(self, root):                           #{{{2
        self.nodes = defaultdict(set)   # key: coords, value: Node
        self.coords = defaultdict(set)
        self.root = root

        self.kdt = kdtree.create([], dimensions = 2)
        self.path = []
        self.path_cost = float("Inf")
        self.goal_node = None
        self.close_node = None
        self.close_dist = float("Inf")

        self.add_node(self.root)

    def __len__(self):                                  #{{{2
        return len(self.nodes)

    def __repr__(self):                                 #{{{2
        name = self.__class__.__name__
        h = [(0, self.root)]
        output = "\r\n"
        while len(h) > 0:
            (dth, node) = h.pop()
            output += ' ' * dth + str(repr(node)) + "\r\n"
            if len(node.children) > 0:
                h += [(dth + 1, child) for child in node.children]
        return output

    def add_node(self, node):                           #{{{2
        (x, y) = node.nvalue[:2]
        self.nodes[(x, y)].add(node)
        self.kdt.add((x, y))

        if self.kdt.height() > 0 and self.kdt.left and self.kdt.right:
            lh, rh = self.kdt.left.height(), self.kdt.right.height()
            if abs(lh - rh) > 100:
                print "tw.add_node #3"
                plist = [x.data for x in self.kdt.inorder()]
                kdt1 = kdtree.create(plist, dimensions = 2)
                self.kdt = kdt

    def get_node(self, coords):                         #{{{2
        return self.nodes[coords]

    def add_tree(self, pnode, node, duration = None):   #{{{2
        if node.parent is not None:
            raise BaseException
        pnode.children.add(node)
        node.parent = pnode
        if duration is not None:
            node.tweight = duration
        else:
            node.tweight = self.distance(pnode, node)
        node.cweight = pnode.cweight + node.tweight
        self.add_node(node)

    def compute_path(self, node):                       #{{{2
        path = [(node.parent, node)]
        cursor = node.parent
        while cursor != self.root:
            path = [(cursor.parent, cursor)] + path
            cursor = cursor.parent
        return path

    def remove_parent(self, nc):                        #{{{2
        nc.parent.children.remove(nc)
        (nc.parent, nc.tweight, nc.cweight) = (None, 0.0, 0.0)

    def remove_node(self, node):                        #{{{2
        (h, removed) = ([node], set())
        while len(h) > 0:
            nc = h.pop()
            nc.parent.children.remove(nc)
            (x, y) = nc.nvalue[:2]
            self.kdt.remove((x, y))
            if len(self.nodes[(x, y)]) > 1:
                self.nodes[(x, y)].remove(nc)
            elif len(self.nodes[(x, y)]) == 1:
                del self.nodes[(x, y)]
            h += list(nc.children)
            removed.add(nc)
        while len(removed) > 0:
            nd = removed.pop()
            del nd

    def search_nn(self, coord):                         #{{{2
        (nn1, n1_dist) = self.kdt.search_nn(coord)
        return (list(self.nodes[nn1.data])[0], n1_dist)

    def search_nn_dist(self, coord, dist):              #{{{2
        nn1 = self.kdt.search_nn_dist(coord, dist)
#        assert(len(nn1) > 0)
        return [n.data for n in nn1]

    def distance(self, node1, node2):                   #{{{2
        return self.distance_2d(node1.nvalue, node2.nvalue)

    def distance_2d(self, p1, p2):                      #{{{2
        (x1, y1) = (list(p1)[0], list(p1)[1])
        (x2, y2) = (list(p2)[0], list(p2)[1])
        return sqrt((x1 - x2)**2 + (y1 - y2)**2)


#---Helper Functions------------------------------------{{{1
def repropagate(tree, node, mnode):                     #{{{2
    tree.remove_parent(node)
    tree.add_tree(mnode, node)
    h = [child for child in node.children]
    # TODO: for online approach, we can add cutoff weight.
    #   (i.e., h = [(cutoff, c) for c in tree.get_children(node)])
    #   Here, the cutoff weight could be the path_cost from
    #   agent current node to its target node.
    #   However, then we also have to update each node's cweight
    #   while progressing its current plan.
    while len(h) > 0:
        nc = h.pop()
        nc.cweight = nc.parent.cweight + nc.tweight
        h += list(nc.children)

def distance_node(node1, node2):                        #{{{2
    return distance_coop(node1.nvalue, node2.nvalue)

def distance_coop(c1, c2):                              #{{{2
    return distance_2d(c1[0], c2[0]) + distance_2d(c1[1], c2[1])

def distance_2d(p1, p2):                                #{{{2
    (x1, y1) = (list(p1)[0], list(p1)[1])
    (x2, y2) = (list(p2)[0], list(p2)[1])
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)
#def distance_node(node1, node2):                        #{{{2
#    return distance_2d(node1.nvalue, node2.nvalue)
#
#def distance_2d(p1, p2):                                #{{{2
#    (x1, y1) = (list(p1)[0], list(p1)[1])
#    (x2, y2) = (list(p2)[0], list(p2)[1])
#    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

#---Main Codes------------------------------------------{{{1
if __name__ == '__main__':
    src = (50, 50)
    dest = (100, 100)

    node = Node(src)
    print node

    node2 = Node(dest)
    print node2

    print node2
    print "node2.nid:",node2.nid
    print "node2.nvalue:",node2.nvalue
    print "node2.cweight:",node2.cweight
    print "node2.tweight:",node2.tweight
    print "node2.parent:",node2.parent
    print "node2.children:",node2.children

    print "----- -----"
    tree = Tree(node)
    tree.add_tree(node, node2)
    print "tree:",tree

    mid1 = (70, 70)
    node3 = Node(mid1)
    tree.remove_parent(node2)
    tree.add_tree(node, node3)

    print "tree:",tree

    tree.add_tree(node3, node2)
    print "tree:",tree

# modelines                                             {{{1
# vim:fdm=marker:fdl=0:
# vim:foldtext=getline(v\:foldstart).'...'.(v\:foldend-v\:foldstart):
# #vim:nofen
