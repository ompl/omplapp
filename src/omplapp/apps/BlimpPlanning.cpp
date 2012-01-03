/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#include "omplapp/apps/BlimpPlanning.h"
#include "ompl/util/Profiler.h"

ompl::base::ScopedState<> ompl::app::BlimpPlanning::getDefaultStartState(void) const
{
    base::ScopedState<base::SE3StateSpace> s(getGeometricComponentStateSpace());
    aiVector3D c = getRobotCenter(0);

    s->setXYZ(c.x, c.y, c.z);
    s->rotation().setIdentity();
    return getFullStateFromGeometricComponent(s);
}

ompl::base::ScopedState<> ompl::app::BlimpPlanning::getFullStateFromGeometricComponent(
    const base::ScopedState<> &state) const
{
    const base::SO3StateSpace::StateType& rot = state->as<base::SE3StateSpace::StateType>()->rotation();
    double norm = sqrt(rot.z * rot.z + rot.w * rot.w);
    base::ScopedState<> s(getStateSpace());
    base::SO3StateSpace::StateType& rot2 =
        s->as<base::CompoundStateSpace::StateType>()->as<base::SE3StateSpace::StateType>(0)->rotation();
    s = 0.0;
    // set x,y,z
    for (unsigned int i=0; i<3; ++i)
        s[i] = state[i];
    // set heading
    rot2.z = rot.z / norm;
    rot2.w = rot.w / norm;
    return s;
}

void ompl::app::BlimpPlanning::propagate(const base::State *from, const control::Control *ctrl,
    const double duration, base::State *result)
{
    odeSolver.propagate (from, ctrl, duration, result);

    // Setting the quaternion representation of the blimp rotation
    base::CompoundStateSpace::StateType& s = *result->as<base::CompoundStateSpace::StateType>();
    base::SE3StateSpace::StateType& pose = *s.as<base::SE3StateSpace::StateType>(0);
    pose.rotation().setAxisAngle(0,0,1, pose.rotation().x);

    // Enforcing control bounds
    getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(1)->enforceBounds(s[1]);
}

void ompl::app::BlimpPlanning::ode(const std::vector<double>& q, const control::Control *ctrl, double /*time*/, std::vector<double>& qdot)
{
    // Retrieving control inputs
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = q[7];
    qdot[1] = q[8];
    qdot[2] = q[9];

    qdot[3] = q[10];

    qdot[7] = u[0] * cos(q[3]);
    qdot[8] = u[0] * sin(q[3]);
    qdot[9] = u[1]; 
    qdot[10] = u[2];
}

ompl::base::StateSpacePtr ompl::app::BlimpPlanning::constructStateSpace(void)
{
    base::StateSpacePtr stateSpace = base::StateSpacePtr(new base::CompoundStateSpace());

    stateSpace->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::SE3StateSpace()), 1.);
    stateSpace->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::RealVectorStateSpace(3)), .3);
    stateSpace->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::RealVectorStateSpace(1)), .3);
    stateSpace->as<base::CompoundStateSpace>()->lock();
    return stateSpace;
}

void ompl::app::BlimpPlanning::setDefaultBounds()
{
    base::RealVectorBounds velbounds(3), omegabounds(1), controlbounds(3);

    velbounds.setLow(-1);
    velbounds.setHigh(1);
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(1)->setBounds(velbounds);
    omegabounds.setLow(-.2);
    omegabounds.setHigh(.2);
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(2)->setBounds(omegabounds);
    controlbounds.setLow(-1);
    controlbounds.setHigh(1);
    controlbounds.setLow(2,-.3);
    controlbounds.setHigh(2,.3);
    getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(controlbounds);

}