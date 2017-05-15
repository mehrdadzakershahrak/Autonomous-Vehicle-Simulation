# treetemp3d.py {{{
# Last Modified: 2016-11-29
# Description: Tree class for 2D (Temporarily) with orientation and velocity
#   This is based on treetemp2d.py.
# Author: Kangjin Kim
# Usage:
#   $ python treetemp3d.py
# Related codes:
#   single_3d.py
# Prerequired Packages:
#   kdtree, bounded_priority_queue
# }}}

#---Package Imports-------------------------------------{{{1
import uuid
from collections import defaultdict
from math import sqrt, pi
import kdtree
from treetemp2d import Node as Node2D, Tree as Tree2D

#---Classes---------------------------------------------{{{1
class Node(Node2D):
    def __init__(self, nvalue, ctrl=None, weights=(0.0, 0.0), parent=None):
        super(Node, self).__init__(nvalue, weights, parent)
        self.ctrl = ctrl

class Tree(Tree2D):
    def __init__(self, root):
        super(Tree, self).__init__(root)

    def add_tree(self, pnode, node, ctrl, duration = None):
        node.ctrl = ctrl
        super(Tree, self).add_tree(pnode, node, duration)
        pass

    def remove_parent(self, nc):                        #{{{2
        nc.ctrl = None
        super(Tree, self).remove_parent(nc)

#---Helper Functions------------------------------------{{{1
#---Main Codes------------------------------------------{{{1
if __name__ == '__main__':
    src = (50, 50, 0.0)
    dest = (100, 100, -pi/8)

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
    ctrl = (0.05, -0.0001)
    tree.add_tree(node, node2, ctrl)
    print "tree:",tree

    mid1 = (70, 70, -pi/16)
    node3 = Node(mid1)
    tree.remove_parent(node2)
    ctrl2 = (0.02, -0.0002)
    tree.add_tree(node, node3, ctrl2)

    print "tree:",tree

    tree.add_tree(node3, node2, ctrl)
    print "tree:",tree
