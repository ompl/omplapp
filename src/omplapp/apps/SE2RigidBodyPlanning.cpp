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

#include "omplapp/apps/SE2RigidBodyPlanning.h"

ompl::base::ScopedState<> ompl::app::SE2RigidBodyPlanning::getDefaultStartState() const
{
    base::ScopedState<base::SE2StateSpace> st(getGeometricComponentStateSpace());
    aiVector3D s = getRobotCenter(0);

    st->setX(s.x);
    st->setY(s.y);
    st->setYaw(0.0);

    return st;
}
