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

#include "omplapp/apps/BlimpPlanning.h"

ompl::base::ScopedState<> ompl::app::BlimpPlanning::getDefaultStartState(void) const
{
    base::ScopedState<base::SE3StateSpace> s(getGeometricComponentStateSpace());
    aiVector3D c = getRobotCenter(0);

    s->setXYZ(c.x, c.y, c.z);
    s->rotation().setIdentity();
    return getFullStateFromGeometricComponent(s);
}

ompl::base::ScopedState<> ompl::app::BlimpPlanning::getFullStateFromGeometricComponent(
    const base::ScopedState<> &state) const
{
    const base::SO3StateSpace::StateType& rot = state->as<base::SE3StateSpace::StateType>()->rotation();
    double norm = sqrt(rot.z * rot.z + rot.w * rot.w);
    base::ScopedState<> s(getStateSpace());
    base::SO3StateSpace::StateType& rot2 =
        s->as<base::CompoundStateSpace::StateType>()->as<base::SE3StateSpace::StateType>(0)->rotation();
    s = 0.0;
    // set x,y,z
    for (unsigned int i=0; i<3; ++i)
        s[i] = state[i];
    // set heading
    rot2.z = rot.z / norm;
    rot2.w = rot.w / norm;
    return s;
}

void ompl::app::BlimpPlanning::propagate(const base::State *from, const control::Control *ctrl,
    const double duration, base::State *result)
{
    int i, j, nsteps = ceil(duration/timeStep_);
    double dt = duration/(double)nsteps;
    base::State *dstate = getStateSpace()->allocState();
    base::CompoundStateSpace::StateType& s = *result->as<base::CompoundStateSpace::StateType>();
    base::CompoundStateSpace::StateType& ds = *dstate->as<base::CompoundStateSpace::StateType>();
    base::SE3StateSpace::StateType& pose = *s.as<base::SE3StateSpace::StateType>(0);
    base::SE3StateSpace::StateType& dpose = *ds.as<base::SE3StateSpace::StateType>(0);
    double& theta = pose.rotation().x;
    double& dtheta = dpose.rotation().x;
    base::RealVectorStateSpace::StateType& vel = *s.as<base::RealVectorStateSpace::StateType>(1);
    base::RealVectorStateSpace::StateType& dvel = *ds.as<base::RealVectorStateSpace::StateType>(1);
    double& omega = s.as<base::RealVectorStateSpace::StateType>(2)->values[0];
    double& domega = ds.as<base::RealVectorStateSpace::StateType>(2)->values[0];

    getStateSpace()->copyState(result, from);
    // use first quaternion component to store heading
    theta = 2. * atan2(pose.rotation().z, pose.rotation().w);
    for (i=0; i<nsteps; ++i)
    {
        ode(result, ctrl, dstate);
        pose.setX(pose.getX() + dt * dpose.getX());
        pose.setY(pose.getY() + dt * dpose.getY());
        pose.setZ(pose.getZ() + dt * dpose.getZ());
        theta += dt * dtheta;
        for (j=0; j<3; ++j) vel[j] += dt * dvel[j];
        omega += dt * domega;
    }
    getStateSpace()->freeState(dstate);
    // convert heading back to a quaternion
    pose.rotation().setAxisAngle(0,0,1,theta);
    getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(1)->enforceBounds(s[1]);
}

void ompl::app::BlimpPlanning::ode(const base::State *state, const control::Control *ctrl,
    base::State *dstate)
{
    const base::CompoundStateSpace::StateType& s = *state->as<base::CompoundStateSpace::StateType>();
    base::CompoundStateSpace::StateType& ds = *dstate->as<base::CompoundStateSpace::StateType>();
    base::SE3StateSpace::StateType& dpose = *ds.as<base::SE3StateSpace::StateType>(0);
    double theta = s.as<base::SE3StateSpace::StateType>(0)->rotation().x;
    const base::RealVectorStateSpace::StateType& vel = *s.as<base::RealVectorStateSpace::StateType>(1);
    base::RealVectorStateSpace::StateType& dvel = *ds.as<base::RealVectorStateSpace::StateType>(1);
    double omega = s.as<base::RealVectorStateSpace::StateType>(2)->values[0];
    double& domega = ds.as<base::RealVectorStateSpace::StateType>(2)->values[0];
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    dpose.setXYZ(vel[0], vel[1], vel[2]);
    dpose.rotation().x = omega;
    dvel[0] = u[0] * cos(theta);
    dvel[1] = u[0] * sin(theta);
    dvel[2] = u[1];
    domega = u[2];
}

ompl::base::StateSpacePtr ompl::app::BlimpPlanning::constructStateSpace(void)
{
    base::StateSpacePtr stateSpace = base::StateSpacePtr(new base::CompoundStateSpace());

    stateSpace->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::SE3StateSpace()), 1.);
    stateSpace->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::RealVectorStateSpace(3)), .3);
    stateSpace->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::RealVectorStateSpace(1)), .3);
    stateSpace->as<base::CompoundStateSpace>()->lock();
    return stateSpace;
}

void ompl::app::BlimpPlanning::setDefaultBounds()
{
    base::RealVectorBounds velbounds(3), omegabounds(1), controlbounds(3);

    velbounds.setLow(-1);
    velbounds.setHigh(1);
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(1)->setBounds(velbounds);
    omegabounds.setLow(-.2);
    omegabounds.setHigh(.2);
    getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(2)->setBounds(omegabounds);
    controlbounds.setLow(-1);
    controlbounds.setHigh(1);
    controlbounds.setLow(2,-.3);
    controlbounds.setHigh(2,.3);
    getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(controlbounds);

}