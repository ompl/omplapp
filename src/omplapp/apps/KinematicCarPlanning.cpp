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

#include "omplapp/apps/KinematicCarPlanning.h"

ompl::base::ScopedState<> ompl::app::KinematicCarPlanning::getDefaultStartState(void) const
{
    base::ScopedState<base::SE2StateManifold> sSE2(getStateManifold());
    aiVector3D s = getRobotCenter(0);

    sSE2->setX(s.x);
    sSE2->setY(s.y);
    sSE2->setYaw(0.);
    return sSE2;
}

void ompl::app::KinematicCarPlanning::propagate(const base::State *from, const control::Control *ctrl,
    const double duration, base::State *result)
{
    int i, nsteps = floor(0.5 + duration/timeStep_);
    double dt = duration/(double)nsteps;
    base::State *dstate = getStateManifold()->allocState();
    base::SE2StateManifold::StateType& s = *result->as<base::SE2StateManifold::StateType>();
    base::SE2StateManifold::StateType& ds = *dstate->as<base::SE2StateManifold::StateType>();

    getStateManifold()->copyState(result, from);
    for (i=0; i<nsteps; ++i)
    {
        ode(result, ctrl, dstate);
        s.setX(s.getX() + timeStep_ * ds.getX());
        s.setY(s.getY() + timeStep_ * ds.getY());
        s.setYaw(s.getYaw() + timeStep_ * ds.getYaw());
    }
    getStateManifold()->freeState(dstate);
        getStateManifold()->enforceBounds(result);
}

void ompl::app::KinematicCarPlanning::ode(const base::State *state, const control::Control *ctrl,
    base::State *dstate)
{
    const base::SE2StateManifold::StateType& q = *state->as<base::SE2StateManifold::StateType>();
    base::SE2StateManifold::StateType& qdot = *dstate->as<base::SE2StateManifold::StateType>();
    const double *u = ctrl->as<control::RealVectorControlManifold::ControlType>()->values;

    qdot.setX(u[0] * cos(q.getYaw()));
    qdot.setY(u[0] * sin(q.getYaw()));
    qdot.setYaw(u[0] * lengthInv_ * tan(u[1]));
}

void ompl::app::DubinsCarPlanning::DubinsControlSampler::sample(control::Control* control)
{
    const base::RealVectorBounds &bounds = static_cast<const control::RealVectorControlManifold*>(manifold_)->getBounds();

    control::RealVectorControlManifold::ControlType *rcontrol =
        static_cast<control::RealVectorControlManifold::ControlType*>(control);
    switch (rng_.uniformInt(0,1))
    {
        case 0: rcontrol->values[0] = 0;
        case 1: rcontrol->values[0] = bounds.high[0];
    }
    rcontrol->values[1] = rng_.uniformReal(bounds.low[1], bounds.high[1]);
}

void ompl::app::ReedsSheppCarPlanning::ReedsSheppControlSampler::sample(control::Control* control)
{
    const base::RealVectorBounds &bounds = static_cast<const control::RealVectorControlManifold*>(manifold_)->getBounds();

    control::RealVectorControlManifold::ControlType *rcontrol =
        static_cast<control::RealVectorControlManifold::ControlType*>(control);
    switch (rng_.uniformInt(-1,1))
    {
        case -1: rcontrol->values[0] = bounds.low[0];
        case 0: rcontrol->values[0] = 0;
        case 1: rcontrol->values[0] = bounds.high[0];
    }
    rcontrol->values[1] = rng_.uniformReal(bounds.low[1], bounds.high[1]);
}
