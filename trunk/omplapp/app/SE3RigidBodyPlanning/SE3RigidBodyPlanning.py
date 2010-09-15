#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
# 
#  Copyright (c) 2010, Rice University
#  All rights reserved.
# 
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
# 
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
# 
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
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
	path.interpolate(.1)
	print path, path.check()
