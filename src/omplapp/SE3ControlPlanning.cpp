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

#include "omplapp/SE3ControlPlanning.h"
#include "omplapp/graphics/detail/RenderPlannerData.h"
#include <limits>

/// @cond IGNORE
namespace ompl
{
    static const base::State* extractState(const base::State* state, unsigned int)
    {
        return state->as<base::CompoundState>()->components[0];
    }
}
/// @endcond

void ompl::app::SE3ControlPlanning::inferEnvironmentBounds(void)
{
    RigidBodyGeometry::inferEnvironmentBounds(getSE3StateManifold());
}

void ompl::app::SE3ControlPlanning::inferProblemDefinitionBounds(void)
{
    RigidBodyGeometry::inferProblemDefinitionBounds(getProblemDefinition(), boost::bind(&extractState, _1, _2), 1, getSE3StateManifold());
}

void ompl::app::SE3ControlPlanning::setup(void)
{
    inferEnvironmentBounds();
    inferProblemDefinitionBounds();

    const base::StateValidityCheckerPtr &svc = allocStateValidityChecker(si_, boost::bind(&extractState, _1, _2), false);
    if (si_->getStateValidityChecker() != svc)
        si_->setStateValidityChecker(svc);

    if (getProblemDefinition()->getStartStateCount() == 0)
    {
        control::SimpleSetup::msg_.inform("Adding default start state");
        base::ScopedState<> start(si_);
        base::ScopedState<> startSE3(getSE3StateManifold());
        getEnvStartState(startSE3);
        start << startSE3;
        start[7] = 0.0;
        addStartState(start);
    }

    control::SimpleSetup::setup();
}

int ompl::app::SE3ControlPlanning::renderPlannerData(void) const
{
    return RenderPlannerData(getPlannerData(), aiVector3D(0.0, 0.0, 0.0), Motion_3D, boost::bind(&extractState, _1, _2), 1);
}
