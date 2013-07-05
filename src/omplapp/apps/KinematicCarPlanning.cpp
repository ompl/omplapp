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
#include <boost/math/constants/constants.hpp>

ompl::app::KinematicCarPlanning::KinematicCarPlanning()
    : AppBase<CONTROL>(constructControlSpace(), Motion_2D), timeStep_(1e-2), lengthInv_(1.), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&KinematicCarPlanning::ode, this, _1, _2, _3)))
{
    name_ = std::string("Kinematic car");
    setDefaultControlBounds();

    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&KinematicCarPlanning::postPropagate, this, _1, _2, _3, _4)));
}

ompl::app::KinematicCarPlanning::KinematicCarPlanning(const control::ControlSpacePtr &controlSpace)
    : AppBase<CONTROL>(controlSpace, Motion_2D), timeStep_(1e-2), lengthInv_(1.), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&KinematicCarPlanning::ode, this, _1, _2, _3)))
{
    setDefaultControlBounds();

    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&KinematicCarPlanning::postPropagate, this, _1, _2, _3, _4)));
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
    cbounds.low[1] = -boost::math::constants::pi<double>() * 30. / 180.;
    cbounds.high[1] = boost::math::constants::pi<double>() * 30. / 180.;
    getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(cbounds);
}

void ompl::app::KinematicCarPlanning::ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
{
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = u[0] * cos(q[2]);
    qdot[1] = u[0] * sin(q[2]);
    qdot[2] = u[0] * lengthInv_ * tan(u[1]);
}

void ompl::app::KinematicCarPlanning::postPropagate(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State* result)
{
    // Normalize orientation value between 0 and 2*pi
    const base::SO2StateSpace* SO2 = getStateSpace()->as<base::SE2StateSpace>()
        ->as<base::SO2StateSpace>(1);
    base::SO2StateSpace::StateType* so2 = result->as<base::SE2StateSpace::StateType>()
        ->as<base::SO2StateSpace::StateType>(1);
    SO2->enforceBounds(so2);
}
