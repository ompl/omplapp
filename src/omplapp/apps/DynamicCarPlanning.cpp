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

#include "omplapp/apps/DynamicCarPlanning.h"

ompl::base::ScopedState<> ompl::app::DynamicCarPlanning::getDefaultStartState(void) const
{
    base::ScopedState<base::CompoundStateManifold> s(getStateManifold());
    base::SE2StateManifold::StateType& pose = *s->as<base::SE2StateManifold::StateType>(0);
    base::RealVectorStateManifold::StateType& vel = *s->as<base::RealVectorStateManifold::StateType>(1);

    aiVector3D c = getRobotCenter(0);
    pose.setX(c.x);
    pose.setY(c.y);
    pose.setYaw(0.);
    vel.values[0] = 0.;
    vel.values[1] = 0.;
    return s;
}

void ompl::app::DynamicCarPlanning::propagate(const base::State *from, const control::Control *ctrl,
    const double duration, base::State *result)
{
    int i, nsteps = ceil(duration/timeStep_);
    double dt = duration/(double)nsteps;
    base::State *dstate = getStateManifold()->allocState();
    base::CompoundStateManifold::StateType& s = *result->as<base::CompoundStateManifold::StateType>();
    base::CompoundStateManifold::StateType& ds = *dstate->as<base::CompoundStateManifold::StateType>();
    base::SE2StateManifold::StateType& pose = *s.as<base::SE2StateManifold::StateType>(0);
    base::SE2StateManifold::StateType& dpose = *ds.as<base::SE2StateManifold::StateType>(0);
    base::RealVectorStateManifold::StateType& vel = *s.as<base::RealVectorStateManifold::StateType>(1);
    base::RealVectorStateManifold::StateType& dvel = *ds.as<base::RealVectorStateManifold::StateType>(1);

    getStateManifold()->copyState(result, from);
    for (i=0; i<nsteps; ++i)
    {
        ode(result, ctrl, dstate);
        pose.setX(pose.getX() + timeStep_ * dpose.getX());
        pose.setY(pose.getY() + timeStep_ * dpose.getY());
        pose.setYaw(pose.getYaw() + timeStep_ * dpose.getYaw());
        vel[0] += timeStep_ * dvel[0];
        vel[1] += timeStep_ * dvel[1];
    }
    getStateManifold()->freeState(dstate);
    getStateManifold()->enforceBounds(result);
}

void ompl::app::DynamicCarPlanning::ode(const base::State *state, const control::Control *ctrl,
    base::State *dstate)
{
    const base::CompoundStateManifold::StateType& s = *state->as<base::CompoundStateManifold::StateType>();
    base::CompoundStateManifold::StateType& ds = *dstate->as<base::CompoundStateManifold::StateType>();
    const base::SE2StateManifold::StateType& pose = *s.as<base::SE2StateManifold::StateType>(0);
    base::SE2StateManifold::StateType& dpose = *ds.as<base::SE2StateManifold::StateType>(0);
    const base::RealVectorStateManifold::StateType& vel = *s.as<base::RealVectorStateManifold::StateType>(1);
    base::RealVectorStateManifold::StateType& dvel = *ds.as<base::RealVectorStateManifold::StateType>(1);
    const double *u = ctrl->as<control::RealVectorControlManifold::ControlType>()->values;

    dpose.setX(pose.getX() * cos(pose.getYaw()));
    dpose.setY(pose.getY() * sin(pose.getYaw()));
    dpose.setYaw(vel[0] * mass_ * lengthInv_ * tan(vel[1]));
    dvel[0] = u[0];
    dvel[1] = u[1];
}

void ompl::app::DynamicCarPlanning::setDefaultBounds()
{
    base::RealVectorBounds bounds(2);
    bounds.setLow(-1.);
    bounds.setHigh(1.);
    getStateManifold()->as<base::CompoundStateManifold>()->as<base::SE2StateManifold>(0)->setBounds(bounds);
    getStateManifold()->as<base::CompoundStateManifold>()->as<base::RealVectorStateManifold>(1)->setBounds(bounds);
    getControlManifold()->as<control::RealVectorControlManifold>()->setBounds(bounds);
}
