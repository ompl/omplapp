#!/usr/bin/env python

######################################################################
# Rice University Software Distribution License
#
# Copyright (c) 2010, Rice University
# All Rights Reserved.
#
# For a full description see the file named LICENSE.
#
######################################################################

# Author: Mark Moll

import sys
from os.path import abspath, dirname, join
from math import pi

ompl_app_root = dirname(dirname(dirname(abspath(__file__))))

try:
    from ompl import base as ob
    from ompl import control as oc
    from ompl import app as oa
except ImportError:
    sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings'))
    from ompl import base as ob
    from ompl import control as oc
    from ompl import app as oa

setup = oa.KinematicCarPlanning()
SE2 = setup.getStateSpace()

bounds = ob.RealVectorBounds(2)
bounds.setLow(-10)
bounds.setHigh(10)
SE2.setBounds(bounds)

# define start state
start = ob.State(SE2)
start().setX(0)
start().setY(0)
start().setYaw(0)

# define goal state
goal = ob.State(SE2)
goal().setX(2)
goal().setY(2)
goal().setYaw(pi)

# set the start & goal states
setup.setStartAndGoalStates(start, goal, .1)

# set the planner
planner = oc.RRT(setup.getSpaceInformation())
setup.setPlanner(planner)

# try to solve the problem
if setup.solve(20):
    # print the (approximate) solution path: print states along the path
    # and controls required to get from one state to the next
    path = setup.getSolutionPath()
    #path.interpolate(); # uncomment if you want to plot the path
    print(path.printAsMatrix())
    if not setup.haveExactSolutionPath():
        print("Solution is approximate. Distance to actual goal is %g" %
              setup.getProblemDefinition().getSolutionDifference())
