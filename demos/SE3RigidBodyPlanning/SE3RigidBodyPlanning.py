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
ompl_resources_dir = join(ompl_app_root, 'resources')

try:
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import app as oa
except:
    sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings' ) )
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import app as oa


setup = oa.SE3RigidBodyPlanning()
print setup.getStateManifold().getBounds().low[0]
setup.setRobotMesh(join(ompl_resources_dir, 'Twistycool_robot.dae'))
setup.setEnvironmentMesh(join(ompl_resources_dir, 'Twistycool_env.dae'))

start = ob.State(setup.getSpaceInformation())
rot = start().rotation()
(rot.w, rot.x, rot.y, rot.z) = (1,0,0,0)
start().setX(0)
start().setY(0)
start().setZ(0)

goal = ob.State(setup.getSpaceInformation())
rot = goal().rotation()
(rot.w, rot.x, rot.y, rot.z) = (1,0,0,0)
goal().setX(0)
goal().setY(0)
goal().setZ(100)

setup.setStartAndGoalStates(start, goal);
print start, goal

if setup.solve():
    setup.simplifySolution()
    path = setup.getSolutionPath()
    path.interpolate(10)
    print path, path.check()
