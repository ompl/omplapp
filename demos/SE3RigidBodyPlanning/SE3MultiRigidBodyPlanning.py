#!/usr/bin/env python

######################################################################
# Rice University Software Distribution License
#
# Copyright (c) 2012, Rice University
# All Rights Reserved.
#
# For a full description see the file named LICENSE.
#
######################################################################

# Author: Ryan Luna

import sys
from os.path import abspath, dirname, join

ompl_app_root = dirname(dirname(dirname(abspath(__file__))))
ompl_resources_dir = join(ompl_app_root, 'resources/3D')

try:
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import app as oa
except ImportError:
    sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import app as oa

# plan in SE(3) for two robots
setup = oa.SE3MultiRigidBodyPlanning(2)

# load the robots and the environment
setup.setRobotMesh(join(ompl_resources_dir, 'cubicles_robot.dae'))
setup.addRobotMesh(join(ompl_resources_dir, 'cubicles_robot.dae'))
setup.setEnvironmentMesh(join(ompl_resources_dir, 'cubicles_env.dae'))

# define start state
start = ob.State(setup.getSpaceInformation())
# set start for robot 1
start1 = start()[0]
start1.setX(-4.96)
start1.setY(-40.62)
start1.setZ(70.57)
start1.rotation().setIdentity()
# set start for robot 2
start2 = start()[1]
start2.setX(200.49)
start2.setY(-40.62)
start2.setZ(70.57)
start2.rotation().setIdentity()

# define goal state
goal = ob.State(setup.getSpaceInformation())
# set goal for robot 1
goal1 = goal()[0]
goal1.setX(200.49)
goal1.setY(-40.62)
goal1.setZ(70.57)
goal1.rotation().setIdentity()
# set goal for robot 2
goal2 = goal()[1]
goal2.setX(-4.96)
goal2.setY(-40.62)
goal2.setZ(70.57)
goal2.rotation().setIdentity()

# set the start & goal states
setup.setStartAndGoalStates(start, goal)

# setting collision checking resolution to 1% of the space extent
setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)

# use RRTConnect for planning
setup.setPlanner(og.RRTConnect(setup.getSpaceInformation()))

# we call setup just so print() can show more information
setup.setup()
print(setup)

# try to solve the problem
if setup.solve(60):
    # simplify & print the solution
    setup.simplifySolution()
    print(setup.getSolutionPath().printAsMatrix())
