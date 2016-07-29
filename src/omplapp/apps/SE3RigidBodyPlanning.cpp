/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include "omplapp/apps/SE3RigidBodyPlanning.h"

ompl::base::ScopedState<> ompl::app::SE3RigidBodyPlanning::getDefaultStartState() const
{
    base::ScopedState<base::SE3StateSpace> st(getGeometricComponentStateSpace());
    aiVector3D s = getRobotCenter(0);

    st->setX(s.x);
    st->setY(s.y);
    st->setZ(s.z);
    st->rotation().setIdentity();

    return st;
}
