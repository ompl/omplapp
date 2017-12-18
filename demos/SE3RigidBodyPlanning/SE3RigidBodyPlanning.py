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
ompl_resources_dir = join(ompl_app_root, 'resources/3D')

try:
    from ompl import base as ob
    from ompl import app as oa
except ImportError:
    sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings'))
    from ompl import base as ob
    from ompl import app as oa

# plan in SE(3)
setup = oa.SE3RigidBodyPlanning()

# load the robot and the environment
setup.setRobotMesh(join(ompl_resources_dir, 'cubicles_robot.dae'))
setup.setEnvironmentMesh(join(ompl_resources_dir, 'cubicles_env.dae'))

# define start state
start = ob.State(setup.getSpaceInformation())
start().setX(-4.96)
start().setY(-40.62)
start().setZ(70.57)
start().rotation().setIdentity()

goal = ob.State(setup.getSpaceInformation())
goal().setX(200.49)
goal().setY(-40.62)
goal().setZ(70.57)
goal().rotation().setIdentity()

# set the start & goal states
setup.setStartAndGoalStates(start, goal)

# setting collision checking resolution to 1% of the space extent
setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)

# we call setup just so print() can show more information
setup.setup()
print(setup)

# try to solve the problem
if setup.solve(10):
    # simplify & print the solution
    setup.simplifySolution()
    path = setup.getSolutionPath()
    path.interpolate(10)
    print(path.printAsMatrix())
    print(path.check())
