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

#include "omplapp/SE2RigidBodyPlanning.h"
#include "omplapp/graphics/detail/RenderPlannerData.h"
#include <limits>

void ompl::app::SE2RigidBodyPlanning::inferEnvironmentBounds(void)
{
    base::RealVectorBounds bounds = getStateManifold()->as<base::SE2StateManifold>()->getBounds();

    // if bounds are not valid
    if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
        getStateManifold()->as<base::SE2StateManifold>()->setBounds(GRigidBodyGeometry::inferEnvironmentBounds());
}

void ompl::app::SE2RigidBodyPlanning::inferProblemDefinitionBounds(void)
{
    // update the bounds based on start states, if needed
    base::RealVectorBounds bounds = getStateManifold()->as<base::SE2StateManifold>()->getBounds();

    std::vector<const base::State*> states;
    getProblemDefinition()->getInputStates(states);

    double minX = std::numeric_limits<double>::infinity();
    double minY = minX;
    double maxX = -minX;
    double maxY = maxX;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
        double x = states[i]->as<base::SE2StateManifold::StateType>()->getX();
        double y = states[i]->as<base::SE2StateManifold::StateType>()->getY();
        if (minX > x) minX = x;
        if (maxX < x) maxX = x;
        if (minY > y) minY = y;
        if (maxY < y) maxY = y;
    }
    double dx = (maxX - minX) * factor_ + add_;
    double dy = (maxY - minY) * factor_ + add_;

    if (bounds.low[0] > minX - dx) bounds.low[0] = minX - dx;
    if (bounds.low[1] > minY - dy) bounds.low[1] = minY - dy;

    if (bounds.high[0] < maxX + dx) bounds.high[0] = maxX + dx;
    if (bounds.high[1] < maxY + dy) bounds.high[1] = maxY + dy;

    getStateManifold()->as<base::SE2StateManifold>()->setBounds(bounds);
}


/// @cond IGNORE
namespace ompl
{
    static const base::State* stateIdentity(const base::State* state, unsigned int)
    {
        return state;
    }
}
/// @endcond

void ompl::app::SE2RigidBodyPlanning::setup(void)
{
    inferEnvironmentBounds();
    inferProblemDefinitionBounds();

    const base::StateValidityCheckerPtr &svc = allocStateValidityChecker(si_, boost::bind(&stateIdentity, _1, _2), false);
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

int ompl::app::SE2RigidBodyPlanning::renderPlannerData(void) const
{
    return RenderPlannerData(getPlannerData(), aiVector3D(0.0, 0.0, 0.0), Motion_2D, boost::bind(&stateIdentity, _1, _2), 1);
}
