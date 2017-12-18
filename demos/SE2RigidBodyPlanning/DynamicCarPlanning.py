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

def dynamicCarDemo(setup):
    print("\n\n***** Planning for a %s *****\n" % setup.getName())
    # plan for dynamic car in SE(2)
    stateSpace = setup.getStateSpace()

    # set the bounds for the R^2 part of SE(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-10)
    bounds.setHigh(10)
    stateSpace.getSubspace(0).setBounds(bounds)

    # define start state
    start = ob.State(stateSpace)
    start[0] = start[1] = start[2] = start[3] = start[4] = 0.

    # define goal state
    goal = ob.State(stateSpace)
    goal[0] = goal[1] = 8.
    goal[2] = 0
    goal[3] = goal[4] = 0.

    # set the start & goal states
    setup.setStartAndGoalStates(start, goal, .5)

    # set the planner
    planner = oc.RRT(setup.getSpaceInformation())
    setup.setPlanner(planner)

    # try to solve the problem
    if setup.solve(40):
        # print the (approximate) solution path: print states along the path
        # and controls required to get from one state to the next
        path = setup.getSolutionPath()
        #path.interpolate(); # uncomment if you want to plot the path
        print(path.printAsMatrix())
        if not setup.haveExactSolutionPath():
            print("Solution is approximate. Distance to actual goal is %g" %
                  setup.getProblemDefinition().getSolutionDifference())

if __name__ == '__main__':
    car = oa.DynamicCarPlanning()
    dynamicCarDemo(car)
