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
    inline double qdot(const ompl::base::SO3StateSpace::StateType& q0, const ompl::base::SO3StateSpace::StateType& q1)
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
    s = 0.0;
    s << state;
    return s;
}

void ompl::app::QuadrotorPlanning::propagate(const base::State *from, const control::Control *ctrl,
    const double duration, base::State *result)
{
    int i, j, nsteps = ceil(duration/timeStep_);
    double dt = duration/(double)nsteps;
    base::State *dstate = getStateSpace()->allocState();
    base::CompoundStateSpace::StateType&      s = *result->as<base::CompoundStateSpace::StateType>();
    base::CompoundStateSpace::StateType&     ds = *dstate->as<base::CompoundStateSpace::StateType>();
    base::SE3StateSpace::StateType&        pose = *s.as<base::SE3StateSpace::StateType>(0);
    base::SE3StateSpace::StateType&       dpose = *ds.as<base::SE3StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType&  vel = *s.as<base::RealVectorStateSpace::StateType>(1);
    base::RealVectorStateSpace::StateType& dvel = *ds.as<base::RealVectorStateSpace::StateType>(1);
    base::SO3StateSpace::StateType&         rot = pose.rotation();
    base::SO3StateSpace::StateType&        drot = dpose.rotation();
    base::SO3StateSpace SO3;

    getStateSpace()->copyState(result, from);
    for (i=0; i<nsteps; ++i)
    {
        ode(result, ctrl, dstate);
        pose.setX(pose.getX() + timeStep_ * dpose.getX());
        pose.setY(pose.getY() + timeStep_ * dpose.getY());
        pose.setZ(pose.getZ() + timeStep_ * dpose.getZ());
        rot.w += timeStep_ * drot.w;
        rot.x += timeStep_ * drot.x;
        rot.y += timeStep_ * drot.y;
        rot.z += timeStep_ * drot.z;
        SO3.enforceBounds(&rot); // make sure we have a unit quaternion
        for (j=0; j<6; ++j)
            vel[j] += timeStep_ * dvel[j];
    }
    getStateSpace()->freeState(dstate);
    getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(1)->enforceBounds(s[1]);
}

void ompl::app::QuadrotorPlanning::ode(const base::State *state, const control::Control *ctrl,
    base::State *dstate)
{
    const base::CompoundStateSpace::StateType&     s = *state->as<base::CompoundStateSpace::StateType>();
    base::CompoundStateSpace::StateType&          ds = *dstate->as<base::CompoundStateSpace::StateType>();
    const base::SE3StateSpace::StateType&       pose = *s.as<base::SE3StateSpace::StateType>(0);
    base::SE3StateSpace::StateType&            dpose = *ds.as<base::SE3StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType& vel = *s.as<base::RealVectorStateSpace::StateType>(1);
    base::RealVectorStateSpace::StateType&      dvel = *ds.as<base::RealVectorStateSpace::StateType>(1);
    const base::SO3StateSpace::StateType&        rot = pose.rotation();
    base::SO3StateSpace::StateType&             drot = dpose.rotation();
    base::SO3StateSpace::StateType qomega;
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;
    double delta, w=rot.w, x=rot.x, y=rot.y, z=rot.z;

    // derivative of position
    dpose.setXYZ(vel[0], vel[1], vel[2]);
    // derivative of orientation
    // 1. First convert omega to quaternion: qdot = omega * q / 2
    qomega.w = 0;
    qomega.x = .5*vel[3];
    qomega.y = .5*vel[4];
    qomega.z = .5*vel[5];
    qmultiply(qomega,rot);
    // 2. We include a numerical correction so that dot(q,qdot) = 0. This constraint is
    // obtained by differentiating q * q_conj = 1
    delta = qdot(rot, qomega);
    // 3. Finally, set the derivative of orientation
    drot.x = qomega.x - delta * x;
    drot.y = qomega.y - delta * y;
    drot.z = qomega.z - delta * z;
    drot.w = qomega.w - delta * w;

    // derivative of velocity
    // the z-axis of the body frame in world coordinates is equal to
    // (2(wy+xz), 2(yz-wx), w^2-x^2-y^2+z^2).
    // This can be easily verified by working out q * (0,0,1).
    dvel[0] = massInv_*(-2*u[0]*(w*y+x*z)         - beta_ * vel[0]);
    dvel[1] = massInv_*(-2*u[0]*(y*z-w*x)         - beta_ * vel[1]);
    dvel[2] = massInv_*(  -u[0]*(w*w-x*x-y*y+z*z) - beta_ * vel[2]) - 9.81;

    // derivative of rotational velocity
    dvel[3] = u[1];
    dvel[4] = u[2];
    dvel[5] = u[3];
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
