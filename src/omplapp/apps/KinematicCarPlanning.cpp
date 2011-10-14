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

ompl::app::KinematicCarPlanning::KinematicCarPlanning()
    : AppBase<CONTROL>(constructControlSpace(), Motion_2D), timeStep_(1e-2), lengthInv_(1.)
{
    name_ = std::string("Kinematic car");
    setDefaultControlBounds();
    si_->setStatePropagator(boost::bind(&KinematicCarPlanning::propagate, this, _1, _2, _3, _4));
}

ompl::app::KinematicCarPlanning::KinematicCarPlanning(const control::ControlSpacePtr &controlSpace)
    : AppBase<CONTROL>(controlSpace, Motion_2D), timeStep_(1e-2), lengthInv_(1.)
{
    setDefaultControlBounds();
    si_->setStatePropagator(boost::bind(&KinematicCarPlanning::propagate, this, _1, _2, _3, _4));
}

ompl::base::ScopedState<> ompl::app::KinematicCarPlanning::getDefaultStartState(void) const
{
    base::ScopedState<base::SE2StateSpace> sSE2(getStateSpace());
    aiVector3D s = getRobotCenter(0);

    sSE2->setX(s.x);
    sSE2->setY(s.y);
    sSE2->setYaw(0.);
    return sSE2;
}

void ompl::app::KinematicCarPlanning::setDefaultControlBounds(void)
{
    base::RealVectorBounds cbounds(2);
    cbounds.low[0] = -5.;
    cbounds.high[0] = 5.;
    cbounds.low[1] = -M_PI * 30. / 180.;
    cbounds.high[1] = M_PI * 30. / 180.;
    getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(cbounds);
}

void ompl::app::KinematicCarPlanning::propagate(const base::State *from, const control::Control *ctrl,
    const double duration, base::State *result)
{
    int i, nsteps = static_cast<int>(floor(0.5 + duration/timeStep_));
    double dt = duration/(double)nsteps;
    base::State *dstate = getStateSpace()->allocState();
    base::SE2StateSpace::StateType& s = *result->as<base::SE2StateSpace::StateType>();
    base::SE2StateSpace::StateType& ds = *dstate->as<base::SE2StateSpace::StateType>();

    getStateSpace()->copyState(result, from);
    for (i=0; i<nsteps; ++i)
    {
        ode(result, ctrl, dstate);
        s.setX(s.getX() + dt * ds.getX());
        s.setY(s.getY() + dt * ds.getY());
        s.setYaw(s.getYaw() + dt * ds.getYaw());
    }
    getStateSpace()->freeState(dstate);
    getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(1)->enforceBounds(s[1]);
}

void ompl::app::KinematicCarPlanning::ode(const base::State *state, const control::Control *ctrl,
    base::State *dstate)
{
    const base::SE2StateSpace::StateType& q = *state->as<base::SE2StateSpace::StateType>();
    base::SE2StateSpace::StateType& qdot = *dstate->as<base::SE2StateSpace::StateType>();
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    qdot.setX(u[0] * cos(q.getYaw()));
    qdot.setY(u[0] * sin(q.getYaw()));
    qdot.setYaw(u[0] * lengthInv_ * tan(u[1]));
}

void ompl::app::DubinsCarPlanning::DubinsControlSampler::sample(control::Control* control)
{
    const base::RealVectorBounds &bounds = static_cast<const control::RealVectorControlSpace*>(space_)->getBounds();

    control::RealVectorControlSpace::ControlType *rcontrol =
        static_cast<control::RealVectorControlSpace::ControlType*>(control);
    switch (rng_.uniformInt(0,1))
    {
        case 0: rcontrol->values[0] = 0; break;
        case 1: rcontrol->values[0] = bounds.high[0];
    }
    rcontrol->values[1] = rng_.uniformReal(bounds.low[1], bounds.high[1]);
}

void ompl::app::ReedsSheppCarPlanning::ReedsSheppControlSampler::sample(control::Control* control)
{
    const base::RealVectorBounds &bounds = static_cast<const control::RealVectorControlSpace*>(space_)->getBounds();

    control::RealVectorControlSpace::ControlType *rcontrol =
        static_cast<control::RealVectorControlSpace::ControlType*>(control);
    switch (rng_.uniformInt(-1,1))
    {
        case -1: rcontrol->values[0] = bounds.low[0]; break;
        case 0: rcontrol->values[0] = 0; break;
        case 1: rcontrol->values[0] = bounds.high[0];
    }
    rcontrol->values[1] = rng_.uniformReal(bounds.low[1], bounds.high[1]);
}
