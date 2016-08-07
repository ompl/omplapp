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

#include "omplapp/apps/QuadrotorPlanning.h"

ompl::base::ScopedState<> ompl::app::QuadrotorPlanning::getDefaultStartState() const
{
    base::ScopedState<base::SE3StateSpace> s(getGeometricComponentStateSpace());
    aiVector3D c = getRobotCenter(0);

    s->setXYZ(c.x, c.y, c.z);
    s->rotation().setIdentity();
    return getFullStateFromGeometricComponent(s);
}


ompl::base::ScopedState<> ompl::app::QuadrotorPlanning::getFullStateFromGeometricComponent(
    const base::ScopedState<> &state) const
{
    base::ScopedState<> s(getStateSpace());
    std::vector <double> reals = state.reals ();

    s = 0.0;
    for (size_t i = 0; i < reals.size (); ++i)
        s[i] = reals[i];
    return s;
}

void ompl::app::QuadrotorPlanning::ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
{
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // zero out qdot
    qdot.resize (q.size (), 0);

    // derivative of position
    qdot[0] = q[7];
    qdot[1] = q[8];
    qdot[2] = q[9];

    // derivative of orientation
    // 1. First convert omega to quaternion: qdot = omega * q / 2
    base::SO3StateSpace::StateType qomega;
    qomega.w = 0;
    qomega.x = .5*q[10];
    qomega.y = .5*q[11];
    qomega.z = .5*q[12];

    // 2. We include a numerical correction so that dot(q,qdot) = 0. This constraint is
    // obtained by differentiating q * q_conj = 1
    double delta = q[3] * qomega.x + q[4] * qomega.y + q[5] * qomega.z;

    // 3. Finally, set the derivative of orientation
    qdot[3] = qomega.x - delta * q[3];
    qdot[4] = qomega.y - delta * q[4];
    qdot[5] = qomega.z - delta * q[5];
    qdot[6] = qomega.w - delta * q[6];

    // derivative of velocity
    // the z-axis of the body frame in world coordinates is equal to
    // (2(wy+xz), 2(yz-wx), w^2-x^2-y^2+z^2).
    // This can be easily verified by working out q * (0,0,1).
    qdot[7] = massInv_ * (-2*u[0]*(q[6]*q[4] + q[3]*q[5]) - beta_ * q[7]);
    qdot[8] = massInv_ * (-2*u[0]*(q[4]*q[5] - q[6]*q[3]) - beta_ * q[8]);
    qdot[9] = massInv_ * (  -u[0]*(q[6]*q[6]-q[3]*q[3]-q[4]*q[4]+q[5]*q[5]) - beta_ * q[9]) - 9.81;

    // derivative of rotational velocity
    qdot[10] = u[1];
    qdot[11] = u[2];
    qdot[12] = u[3];
}

void ompl::app::QuadrotorPlanning::postPropagate(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State* result)
{
    const base::CompoundStateSpace* cs = getStateSpace()->as<base::CompoundStateSpace>();
    const base::SO3StateSpace* SO3 = cs->as<base::SE3StateSpace>(0)->as<base::SO3StateSpace>(1);
    base::CompoundStateSpace::StateType& csState = *result->as<base::CompoundStateSpace::StateType>();
    base::SO3StateSpace::StateType& so3State = csState.as<base::SE3StateSpace::StateType>(0)->rotation();

    // Normalize the quaternion representation for the quadrotor
    SO3->enforceBounds(&so3State);
    // Enforce velocity bounds
    cs->getSubspace(1)->enforceBounds(csState[1]);
}

ompl::base::StateSpacePtr ompl::app::QuadrotorPlanning::constructStateSpace()
{
    auto stateSpace(std::make_shared<base::CompoundStateSpace>());

    stateSpace->addSubspace(std::make_shared<base::SE3StateSpace>(), 1.);
    stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(6), .3);
    stateSpace->lock();
    return stateSpace;
}

void ompl::app::QuadrotorPlanning::setDefaultBounds()
{
    base::RealVectorBounds velbounds(6), controlbounds(4);

    velbounds.setLow(-1);
    velbounds.setHigh(1);
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(1)->setBounds(velbounds);
    controlbounds.setLow(-1);
    controlbounds.setHigh(1);
    controlbounds.setLow(0, 5.);
    controlbounds.setHigh(0, 15.);
    getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(controlbounds);
}
