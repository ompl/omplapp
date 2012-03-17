/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2012, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ryan Luna */

#include "MultiRigidBodyPlanning.h"

ompl::base::ScopedState<> ompl::app::MultiRigidBodyPlanning::getDefaultStartState(void) const
{
    base::ScopedState<> st(getStateSpace());
    base::CompoundStateSpace::StateType* c_st = st.get()->as<base::CompoundStateSpace::StateType>();

    for (unsigned int i = 0; i < n_; ++i)
    {
        aiVector3D s = getRobotCenter(i);
        ompl::base::SE3StateSpace::StateType* sub = c_st->as<ompl::base::SE3StateSpace::StateType>(i);
        sub->setX(s.x);
        sub->setY(s.y);
        sub->setZ(s.z);
        sub->rotation().setIdentity();
    }
   
    return st;
}

// This is overloaded because the AppBase methods that infer state space bounds
// assume either SE(2) or SE(3).  THIS NEEDS TO BE FIXED.
void ompl::app::MultiRigidBodyPlanning::setup(void)
{
    if (AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::getProblemDefinition()->getStartStateCount() == 0)
    {
        AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::msg_.inform("Adding default start state");
        AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::addStartState(getDefaultStartState());
    }

    const base::StateValidityCheckerPtr &svc = allocStateValidityChecker(AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::si_,
                                                                                     getGeometricStateExtractor(), isSelfCollisionEnabled());
    if (AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::si_->getStateValidityChecker() != svc)
        AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::si_->setStateValidityChecker(svc);

    AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::getStateSpace()->setup();

    if (!AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::getStateSpace()->hasDefaultProjection())
        AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::getStateSpace()->
            registerDefaultProjection(allocGeometricStateProjector(AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::getStateSpace(),
            mtype_, getGeometricComponentStateSpace(),
            getGeometricStateExtractor()));
    AppTypeSelector<ompl::app::GEOMETRIC>::SimpleSetup::setup();
}

