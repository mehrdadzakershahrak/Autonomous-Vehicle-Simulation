Single Agent Planner
====================

This is a simulation for a single agent planner. This planner consists of
a high level planner and a low level planner.

In high level, we plan a graph search in a triangulated graph environment.
Each triangle is connected with its neighbors.

In low level, we plan a lane follower. Especially, we follow the mid points
of the lane. Instead of sampling all the possible states with control inputs,
we use lattice search.



Authors
-------

Kangjin Kim (kangjin.kim@asu.edu)



Required packages
-----------------

* pygame
* pylygon
* scipy
* kdtree
* cPickle
* numpy
* numpy.linalg
* bs4
* svg.path
* triangle



Related Codes
-------------

* lattice_search3.py
* car3d.py
* treetemp2d.py
* treetemp3d.py
* map_loader4_8.py
* tgraph3.py
* single_3d_3.py
* lineequ5.py



Map data
--------

* exenv8.svg
* exenv2.svg



Lattice Data
------------

* primitive2.py.0
* grid_primitives.py.0
* local_points3.py.2
* local_points4.py.0



Usage
-----

$ python lattice_search3.py
