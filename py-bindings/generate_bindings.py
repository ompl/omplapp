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

from os.path import abspath, dirname, join
import sys
sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings'))
from ompl.bindings_generator import code_generator_t


class ompl_app_generator_t(code_generator_t):
    def __init__(self):
        code_generator_t.__init__(self, 'app',
            ['../ompl/py-bindings/bindings/base',
            '../ompl/py-bindings/bindings/geometric',
            '../ompl/py-bindings/bindings/control'])

    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        # Py++/pygccxml generates the wrong code for scoped enums (enum class AppType),
        # so manually generate the right code
        self.mb.enum('::ompl::app::AppType').exclude()
        self.mb.add_registration_code("""bp::enum_< ompl::app::AppType>("AppType")
        .value("GEOMETRIC", ompl::app::AppType::GEOMETRIC)
        .value("CONTROL", ompl::app::AppType::CONTROL)
        .export_values()
        ;""")
        self.mb.class_('::ompl::app::AppBase< ompl::app::AppType::GEOMETRIC >').rename('AppBaseGeometric')
        self.mb.class_('::ompl::app::AppBase< ompl::app::AppType::CONTROL>').rename('AppBaseControl')
        self.mb.class_('::ompl::app::AppTypeSelector< ompl::app::AppType::GEOMETRIC >').rename('AppTypeGeometric')
        self.mb.class_('::ompl::app::AppTypeSelector< ompl::app::AppType::CONTROL >').rename('AppTypeControl')
        # The virtual functions "solve" and "clear" from SimpleSetup are not redefined
        # in these derived classes, and for some reason Py++ doesn't export them
        # (even though it does generate some wrapper code for them)
        for cls in ['SE2RigidBodyPlanning', 'SE3RigidBodyPlanning', 'SE2MultiRigidBodyPlanning', 'SE3MultiRigidBodyPlanning', 'GSE2RigidBodyPlanning', 'GSE3RigidBodyPlanning']:
            self.ompl_ns.class_(cls).add_registration_code("""
            def("solve", (::ompl::base::PlannerStatus(::ompl::app::%s::*)( double ))(&::ompl::app::%s::solve), (bp::arg("solveTime")) )""" % (cls, cls))
            self.ompl_ns.class_(cls).add_registration_code("""
            def("solve", (::ompl::base::PlannerStatus(::ompl::app::%s::*)( const ompl::base::PlannerTerminationCondition& ))(&::ompl::app::%s::solve), (bp::arg("ptc")) )""" % (cls,cls))
            self.ompl_ns.class_(cls).add_registration_code(
            'def("clear", &%s_wrapper::clear)' % cls)
            self.ompl_ns.class_(cls).add_registration_code(
            'def("getStateSpace", &::ompl::geometric::SimpleSetup::getStateSpace, bp::return_value_policy< bp::copy_const_reference >())')

        for cls in ['KinematicCarPlanning', 'GKinematicCarPlanning',
            'DynamicCarPlanning', 'GDynamicCarPlanning',
            'BlimpPlanning', 'GBlimpPlanning',
            'QuadrotorPlanning', 'GQuadrotorPlanning']:
            self.ompl_ns.class_(cls).add_registration_code("""
            def("solve", (::ompl::base::PlannerStatus(::ompl::app::%s::*)( double ))(&::ompl::app::%s::solve), (bp::arg("solveTime")) )""" % (cls, cls))
            self.ompl_ns.class_(cls).add_registration_code("""
            def("solve", (::ompl::base::PlannerStatus(::ompl::app::%s::*)( const ompl::base::PlannerTerminationCondition& ))(&::ompl::app::%s::solve), (bp::arg("ptc")) )""" % (cls,cls))
            self.ompl_ns.class_(cls).add_registration_code(
            'def("clear", &%s_wrapper::clear)' % cls)
            self.ompl_ns.class_(cls).add_registration_code(
            'def("getStateSpace", &::ompl::control::SimpleSetup::getStateSpace, bp::return_value_policy< bp::copy_const_reference >())')

        # workaround for internal compiler error in Xcode 4.3 (already fixed in MacPorts clang-3.1)
        rb = self.ompl_ns.class_('RigidBodyGeometry')
        rb.member_function('setEnvironmentMesh').exclude()
        rb.add_registration_code('def("setEnvironmentMesh",&::ompl::app::RigidBodyGeometry::setEnvironmentMesh)')
        rb.member_function('addEnvironmentMesh').exclude()
        rb.add_registration_code('def("addEnvironmentMesh",&::ompl::app::RigidBodyGeometry::addEnvironmentMesh)')
        rb.member_function('setRobotMesh').exclude()
        rb.add_registration_code('def("setRobotMesh",&::ompl::app::RigidBodyGeometry::setRobotMesh)')
        rb.member_function('addRobotMesh').exclude()
        rb.add_registration_code('def("addRobotMesh",&::ompl::app::RigidBodyGeometry::addRobotMesh)')
        rb.member_function('setStateValidityCheckerType').exclude()
        rb.add_registration_code('def("setStateValidityCheckerType",&::ompl::app::RigidBodyGeometry::setStateValidityCheckerType)')

if __name__ == '__main__':
    sys.setrecursionlimit(50000)
    ompl_app_generator_t()
