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

#include "omplapp/SE3RigidBodyPlanning.h"
#include "omplapp/graphics/detail/RenderPlannerData.h"
#include <limits>


/// @cond IGNORE
namespace ompl
{
    static const base::State* extractState(const base::State* state, unsigned int)
    {
        return state;
    }
}
/// @endcond

void ompl::app::SE3RigidBodyPlanning::inferProblemDefinitionBounds(void)
{
    RigidBodyGeometry::inferProblemDefinitionBounds(getProblemDefinition(), boost::bind(&extractState, _1, _2), 1, getStateManifold());
}

void ompl::app::SE3RigidBodyPlanning::inferEnvironmentBounds(void)
{
    RigidBodyGeometry::inferEnvironmentBounds(getStateManifold());
}

void ompl::app::SE3RigidBodyPlanning::setup(void)
{
    inferEnvironmentBounds();
    inferProblemDefinitionBounds();

    const base::StateValidityCheckerPtr &svc = allocStateValidityChecker(si_, boost::bind(&extractState, _1, _2), false);
    if (si_->getStateValidityChecker() != svc)
        si_->setStateValidityChecker(svc);

    if (getProblemDefinition()->getStartStateCount() == 0)
    {
        geometric::SimpleSetup::msg_.inform("Adding default start state");
        base::ScopedState<> start(si_);
        getEnvStartState(start);
        addStartState(start);
    }

    geometric::SimpleSetup::setup();
}

int ompl::app::SE3RigidBodyPlanning::renderPlannerData(void) const
{
    return RenderPlannerData(getPlannerData(), aiVector3D(0.0, 0.0, 0.0), Motion_3D, boost::bind(&extractState, _1, _2), 1);
}
