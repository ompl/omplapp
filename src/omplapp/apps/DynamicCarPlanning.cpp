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
#include <boost/math/constants/constants.hpp>

ompl::base::ScopedState<> ompl::app::DynamicCarPlanning::getDefaultStartState() const
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

void ompl::app::DynamicCarPlanning::ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
{
    // Retrieving control inputs
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = q[3] * cos(q[2]);
    qdot[1] = q[3] * sin(q[2]);
    qdot[2] = q[3] * mass_ * lengthInv_ * tan(q[4]);

    qdot[3] = u[0];
    qdot[4] = u[1];
}

void ompl::app::DynamicCarPlanning::postPropagate(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State* result)
{
    // Normalize orientation value between 0 and 2*pi
    const base::SO2StateSpace* SO2 = getStateSpace()->as<base::CompoundStateSpace>()
        ->as<base::SE2StateSpace>(0)->as<base::SO2StateSpace>(1);
    auto* so2 = result->as<base::CompoundStateSpace::StateType>()
        ->as<base::SE2StateSpace::StateType>(0)->as<base::SO2StateSpace::StateType>(1);
    SO2->enforceBounds(so2);
}

void ompl::app::DynamicCarPlanning::setDefaultBounds()
{
    base::RealVectorBounds bounds(2);
    bounds.low[0] = -1.;
    bounds.high[0] = 1.;
    bounds.low[1] = -boost::math::constants::pi<double>() * 30. / 180.;
    bounds.high[1] = boost::math::constants::pi<double>() * 30. / 180.;
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(1)->setBounds(bounds);
    bounds.low[0] = -.5;
    bounds.high[0] = .5;
    bounds.low[1] = -boost::math::constants::pi<double>() * 2. / 180.;
    bounds.high[1] = boost::math::constants::pi<double>() * 2. / 180.;
    getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(bounds);
}
