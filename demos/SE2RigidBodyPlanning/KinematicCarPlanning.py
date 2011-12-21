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
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import app as oa
except:
    sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings' ) )
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import app as oa

def kinematicCarDemo(setup):
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
        n = len(path.states)
        for i in range(n):
            s = path.states[i]
            print s.getX(), s.getY(), s.getYaw(),
            if i==0:
                # null controls applied for zero seconds to get to start state
                print "0 0 0"
            else:
                # print controls and control duration needed to get from state i-1 to state i
                print path.controls[i-1][0], path.controls[i-1][1], path.controlDurations[i-1]
        if not setup.haveExactSolutionPath():
            print "Solution is approximate. Distance to actual goal is ", \
                setup.getGoal().getDifference()

if __name__ == '__main__':
    car = oa.KinematicCarPlanning()
    kinematicCarDemo(car)
