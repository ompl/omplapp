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
            '../ompl/py-bindings/bindings/geometric',
            '../ompl/py-bindings/bindings/control'])

    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        for c in self.ompl_ns.classes():
            print c.decl_string
        self.mb.class_('::ompl::app::AppBase<(ompl::app::AppType)0>').rename('AppBaseGeometric')
        self.mb.class_('::ompl::app::AppBase<(ompl::app::AppType)1>').rename('AppBaseControl')
        self.mb.class_('::ompl::app::AppTypeSelector<(ompl::app::AppType)0>').rename('AppTypeGeometric')
        self.mb.class_('::ompl::app::AppTypeSelector<(ompl::app::AppType)1>').rename('AppTypeControl')

        # The virtual functions "solve" and "clear" from SimpleSetup are not redefined
        # in these two derived classes, and for some reason Py++ doesn't export them
        # (even though it does generate some wrapper code for them)
        for cls in ['SE2RigidBodyPlanning', 'SE3RigidBodyPlanning', 'GSE2RigidBodyPlanning', 'GSE3RigidBodyPlanning']:
            self.ompl_ns.class_(cls).add_registration_code(
            'def("solve", &::ompl::geometric::SimpleSetup::solve, &%s_wrapper::default_solve, (bp::arg("time")=1.0e+0))' % cls)
            self.ompl_ns.class_(cls).add_registration_code(
            'def("clear", &::ompl::geometric::SimpleSetup::clear, &%s_wrapper::default_clear)' % cls)
            self.ompl_ns.class_(cls).add_registration_code(
            'def("getStateSpace", &::ompl::geometric::SimpleSetup::getStateSpace, bp::return_value_policy< bp::copy_const_reference >())')

        for cls in ['KinematicCarPlanning', 'DubinsCarPlanning', 'ReedsSheppCarPlanning',
            'GKinematicCarPlanning', 'GDubinsCarPlanning', 'GReedsSheppCarPlanning',
            'DynamicCarPlanning', 'GDynamicCarPlanning',
            'BlimpPlanning', 'GBlimpPlanning',
            'QuadrotorPlanning', 'GQuadrotorPlanning']:
            self.ompl_ns.class_(cls).add_registration_code(
            'def("solve", &::ompl::control::SimpleSetup::solve, &%s_wrapper::default_solve, (bp::arg("time")=1.0e+0))' % cls)
            self.ompl_ns.class_(cls).add_registration_code(
            'def("clear", &::ompl::control::SimpleSetup::clear, &%s_wrapper::default_clear)' % cls)
            self.ompl_ns.class_(cls).add_registration_code(
            'def("getStateSpace", &::ompl::control::SimpleSetup::getStateSpace, bp::return_value_policy< bp::copy_const_reference >())')


if __name__ == '__main__':
    setrecursionlimit(50000)
    ompl_app_generator_t()
