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

namespace
{
    // quaternion dot product
    inline double quatdot(const ompl::base::SO3StateSpace::StateType& q0, const ompl::base::SO3StateSpace::StateType& q1)
    {
        return q0.w*q1.w + q0.x*q1.x + q0.y*q1.y + q0.z*q1.z;
    }
    // quaternion multiplication
    inline void qmultiply(ompl::base::SO3StateSpace::StateType& q0, const ompl::base::SO3StateSpace::StateType& q1)
    {
        q0.w = q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z;
        q0.x = q0.w * q1.x + q0.x * q1.w + q0.y * q1.z - q0.z * q1.y;
        q0.y = q0.w * q1.y + q0.y * q1.w + q0.z * q1.x - q0.x * q1.z;
        q0.z = q0.w * q1.z + q0.z * q1.w + q0.x * q1.y - q0.y * q1.x;
    }
}


ompl::base::ScopedState<> ompl::app::QuadrotorPlanning::getDefaultStartState(void) const
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

void ompl::app::QuadrotorPlanning::propagate(const base::State *from, const control::Control *ctrl,
    const double duration, base::State *result)
{
    odeSolver.propagate (from, ctrl, duration, result);

    // Enforce control bounds
    base::CompoundStateSpace::StateType& s = *result->as<base::CompoundStateSpace::StateType>();
    getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(1)->enforceBounds(s[1]);
}

void ompl::app::QuadrotorPlanning::ode(const std::vector<double>&q, const control::Control *ctrl,
    double time, std::vector<double>& qdot)
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
    base::SO3StateSpace::StateType rot;
    rot.x = q[3]; rot.y = q[4]; rot.z = q[5]; rot.w = q[6];
    double delta = quatdot(rot, qomega);

    // 3. Finally, set the derivative of orientation
    qdot[3] = qomega.x - delta * rot.x;
    qdot[4] = qomega.y - delta * rot.y;
    qdot[5] = qomega.z - delta * rot.z;
    qdot[6] = qomega.w - delta * rot.w;

    // derivative of velocity
    // the z-axis of the body frame in world coordinates is equal to
    // (2(wy+xz), 2(yz-wx), w^2-x^2-y^2+z^2).
    // This can be easily verified by working out q * (0,0,1).
    qdot[7] = massInv_ * (-2*u[0]*(rot.w*rot.y + rot.x*rot.z) - beta_ * q[7]);
    qdot[8] = massInv_ * (-2*u[0]*(rot.y*rot.z - rot.w*rot.x) - beta_ * q[8]);
    qdot[9] = massInv_ * (  -u[0]*(rot.w*rot.w-rot.x*rot.x-rot.y*rot.y+rot.z*rot.z) - beta_ * q[9]) - 9.81;

    // derivative of rotational velocity
    qdot[10] = u[1];
    qdot[11] = u[2];
    qdot[12] = u[3];
}

ompl::base::StateSpacePtr ompl::app::QuadrotorPlanning::constructStateSpace(void)
{
    base::StateSpacePtr stateSpace = base::StateSpacePtr(new base::CompoundStateSpace());

    stateSpace->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::SE3StateSpace()), 1.);
    stateSpace->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::RealVectorStateSpace(6)), .3);
    stateSpace->as<base::CompoundStateSpace>()->lock();
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
