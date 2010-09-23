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

from sys import argv, setrecursionlimit
from os.path import abspath, dirname, join
import sys
sys.path.insert(0, join(dirname(dirname(abspath(__file__))),'ompl/py-bindings'))
print sys.path
from ompl.bindings_generator import code_generator_t, default_replacement


class ompl_app_generator_t(code_generator_t):
	def __init__(self):
		code_generator_t.__init__(self, 'app', 
			['../ompl/py-bindings/bindings/base', 
			 '../ompl/py-bindings/bindings/geometric'])
		#self.ompl_ns.member_functions('solve').already_exposed = False
		#print self.ompl_ns.class_('SimpleSetup').member_function('solve').already_exposed
	
	def filter_declarations(self):
		code_generator_t.filter_declarations(self)
		# The virtual functions "solve" and "clear" from SimpleSetup are not redefined
		# in these two derived classes, and for some reason Py++ doesn't export them
		# (even though it does generate some wrapper code for them)
		for cls in ['SE2RigidBodyPlanning', 'SE3RigidBodyPlanning']:
			self.ompl_ns.class_(cls).add_registration_code(
			'def("solve", &::ompl::geometric::SimpleSetup::solve, &%s_wrapper::default_solve, (bp::arg("time")=1.0e+0))' % cls)
			self.ompl_ns.class_(cls).add_registration_code(
			'def("clear", &::ompl::geometric::SimpleSetup::clear, &%s_wrapper::default_clear)' % cls)
			self.ompl_ns.class_(cls).add_registration_code(
			'def("getStateManifold", &::ompl::geometric::SimpleSetup::getStateManifold, bp::return_value_policy< bp::copy_const_reference >())')
		# include these pure virtual member functions, even though they are 
		# protected, so that Py++ can create the appropriate wrappers
		self.ompl_ns.member_functions('inferEnvironmentBounds').include()
		self.ompl_ns.member_functions('inferProblemDefinitionBounds').include()
		self.ompl_ns.member_functions('getRobotCenterAndStartState').include()
		self.ompl_ns.member_functions('allocStateValidityChecker').include()
		
if __name__ == '__main__':
	setrecursionlimit(50000)
	ompl_app_generator_t()
