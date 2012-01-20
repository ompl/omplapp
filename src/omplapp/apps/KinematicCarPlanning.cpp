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
    : AppBase<CONTROL>(constructControlSpace(), Motion_2D), timeStep_(1e-2), lengthInv_(1.), odeSolver(control::ODEBasicSolver<>(si_, boost::bind(&ompl::app::KinematicCarPlanning::ode, this, _1, _2, _3)))
{
    name_ = std::string("Kinematic car");
    setDefaultControlBounds();

    si_->setStatePropagator(odeSolver.getStatePropagator());
}

ompl::app::KinematicCarPlanning::KinematicCarPlanning(const control::ControlSpacePtr &controlSpace)
    : AppBase<CONTROL>(controlSpace, Motion_2D), timeStep_(1e-2), lengthInv_(1.), odeSolver(control::ODEBasicSolver<>(si_, boost::bind(&ompl::app::KinematicCarPlanning::ode, this, _1, _2, _3)))
{
    setDefaultControlBounds();

    si_->setStatePropagator(odeSolver.getStatePropagator());
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

void ompl::app::KinematicCarPlanning::ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
{
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = u[0] * cos(q[2]);
    qdot[1] = u[0] * sin(q[2]);
    qdot[2] = u[0] * lengthInv_ * tan(u[1]);
}
