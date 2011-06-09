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
    base::ScopedState<base::CompoundStateSpace> s(getStateSpace());
    base::SE2StateSpace::StateType& pose = *s->as<base::SE2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType& vel = *s->as<base::RealVectorStateSpace::StateType>(1);

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
    base::State *dstate = getStateSpace()->allocState();
    base::CompoundStateSpace::StateType& s = *result->as<base::CompoundStateSpace::StateType>();
    base::CompoundStateSpace::StateType& ds = *dstate->as<base::CompoundStateSpace::StateType>();
    base::SE2StateSpace::StateType& pose = *s.as<base::SE2StateSpace::StateType>(0);
    base::SE2StateSpace::StateType& dpose = *ds.as<base::SE2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType& vel = *s.as<base::RealVectorStateSpace::StateType>(1);
    base::RealVectorStateSpace::StateType& dvel = *ds.as<base::RealVectorStateSpace::StateType>(1);

    getStateSpace()->copyState(result, from);
    for (i=0; i<nsteps; ++i)
    {
        ode(result, ctrl, dstate);
        pose.setX(pose.getX() + timeStep_ * dpose.getX());
        pose.setY(pose.getY() + timeStep_ * dpose.getY());
        pose.setYaw(pose.getYaw() + timeStep_ * dpose.getYaw());
        vel[0] += timeStep_ * dvel[0];
        vel[1] += timeStep_ * dvel[1];
    }
    getStateSpace()->freeState(dstate);
    getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(1)->enforceBounds(s[1]);
}

void ompl::app::DynamicCarPlanning::ode(const base::State *state, const control::Control *ctrl,
    base::State *dstate)
{
    const base::CompoundStateSpace::StateType& s = *state->as<base::CompoundStateSpace::StateType>();
    base::CompoundStateSpace::StateType& ds = *dstate->as<base::CompoundStateSpace::StateType>();
    const base::SE2StateSpace::StateType& pose = *s.as<base::SE2StateSpace::StateType>(0);
    base::SE2StateSpace::StateType& dpose = *ds.as<base::SE2StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType& vel = *s.as<base::RealVectorStateSpace::StateType>(1);
    base::RealVectorStateSpace::StateType& dvel = *ds.as<base::RealVectorStateSpace::StateType>(1);
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    dpose.setX(vel[0] * cos(pose.getYaw()));
    dpose.setY(vel[0] * sin(pose.getYaw()));
    dpose.setYaw(vel[0] * mass_ * lengthInv_ * tan(vel[1]));
    dvel[0] = u[0];
    dvel[1] = u[1];
}

void ompl::app::DynamicCarPlanning::setDefaultBounds()
{
    base::RealVectorBounds bounds(2);
    bounds.low[0] = -1.;
    bounds.high[0] = 1.;
    bounds.low[1] = -M_PI * 30. / 180.;
    bounds.high[1] = M_PI * 30. / 180.;
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(1)->setBounds(bounds);
    bounds.low[0] = -.5;
    bounds.high[0] = .5;
    bounds.low[1] = -M_PI * 2. / 180.;
    bounds.high[1] = M_PI * 2. / 180.;
    getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(bounds);
}
