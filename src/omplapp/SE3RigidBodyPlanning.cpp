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
#include <limits>

void ompl::app::SE3RigidBodyPlanning::inferProblemDefinitionBounds(void)
{
    // update the bounds based on start states, if needed
    base::RealVectorBounds bounds = getStateManifold()->as<base::SE3StateManifold>()->getBounds();

    std::vector<const base::State*> states;
    getProblemDefinition()->getInputStates(states);

    double minX = std::numeric_limits<double>::infinity();
    double minY = minX;
    double minZ = minX;
    double maxX = -minX;
    double maxY = maxX;
    double maxZ = maxX;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
        double x = states[i]->as<base::SE3StateManifold::StateType>()->getX();
        double y = states[i]->as<base::SE3StateManifold::StateType>()->getY();
        double z = states[i]->as<base::SE3StateManifold::StateType>()->getZ();
        if (minX > x) minX = x;
        if (maxX < x) maxX = x;
        if (minY > y) minY = y;
        if (maxY < y) maxY = y;
        if (minZ > z) minZ = z;
        if (maxZ < z) maxZ = z;
    }
    double dx = (maxX - minX) * (factor_ - 1.0) + add_;
    double dy = (maxY - minY) * (factor_ - 1.0) + add_;
    double dz = (maxZ - minZ) * (factor_ - 1.0) + add_;

    if (bounds.low[0] > minX - dx) bounds.low[0] = minX - dx;
    if (bounds.low[1] > minY - dy) bounds.low[1] = minY - dy;
    if (bounds.low[2] > minZ - dz) bounds.low[2] = minZ - dz;

    if (bounds.high[0] < maxX + dx) bounds.high[0] = maxX + dx;
    if (bounds.high[1] < maxY + dy) bounds.high[1] = maxY + dy;
    if (bounds.high[2] < maxZ + dz) bounds.high[2] = maxZ + dz;

    getStateManifold()->as<base::SE3StateManifold>()->setBounds(bounds);
}

void ompl::app::SE3RigidBodyPlanning::inferEnvironmentBounds(void)
{
    base::RealVectorBounds bounds = getStateManifold()->as<base::SE3StateManifold>()->getBounds();

    // if bounds are not valid
    if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
        getStateManifold()->as<base::SE3StateManifold>()->setBounds(GRigidBodyGeometry::inferEnvironmentBounds());
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

void ompl::app::SE3RigidBodyPlanning::setup(void)
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
