/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#ifndef OMPL_GEOMETRIC_SIMPLE_SETUP_
#define OMPL_GEOMETRIC_SIMPLE_SETUP_

#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include <boost/function.hpp>

namespace ompl
{

    namespace geometric
    {
		
	/** \brief Create the set of classes typically needed to solve a
	    geometric problem */
	class SimpleSetup
	{
	public:
	    SimpleSetup(const base::StateManifoldPtr &manifold, const base::PlannerAllocator &pa) : configured_(false)
	    {
		si_.reset(new base::SpaceInformation(manifold));
		pdef_.reset(new base::ProblemDefinition(si_));
		psk_.reset(new PathSimplifier(si_));
		pa_ = pa;
	    }
	    
	    ~SimpleSetup(void)
	    {
	    }
	    
	    const base::SpaceInformationPtr& getSpaceInformation(void) const
	    {
		return si_;
	    }
	    
	    const base::StateManifoldPtr& getStateManifold(void) const
	    {
		return si_->getStateManifold();
	    }
	    
	    const base::ProblemDefinitionPtr& getProblemDefinition(void) const
	    {
		return pdef_;
	    }
	    
	    const base::StateValidityCheckerPtr& getStateValidityChecker(void) const
	    {
		return si_->getStateValidityChecker();
	    }

	    void setStateValidityChecker(const base::StateValidityCheckerPtr &svc) const
	    {
		return si_->setStateValidityChecker(svc);
	    }

	    const base::GoalPtr& getGoal(void) const
	    {
		return pdef_->getGoal();
	    }

	    void setGoal(const base::GoalPtr &goal)
	    {
		pdef_->setGoal(goal);
	    }

	    const base::PlannerPtr& getPlanner(void) const
	    {
		return planner_;
	    }

	    const PathSimplifierPtr& getPathSimplifier(void) const
	    {
		return psk_;
	    }
	    
	    void setup(void)
	    {
		if (!configured_)
		{
		    si_->setup();
		    planner_ = pa_(si_);
		    planner_->setProblemDefinition(pdef_);
		    planner_->setup();
		    configured_ = true;
		}
	    }
	    
	    bool solve(double time)
	    {
		setup();
		return planner_->solve(time);
	    }
	    
	    void clear(void)
	    {
		if (planner_)
		    planner_->clear();
		if (pdef_->getGoal())
		    pdef_->getGoal()->clearSolutionPath();
		pdef_->clearStartStates();
	    }
	    	    
	    void simplifySolution(void)
	    {
		const base::PathPtr &p =  pdef_->getGoal()->getSolutionPath();
		if (p)
		    psk_->simplifyMax(static_cast<PathGeometric&>(*p));
		else
		    msg_.warn("No solution to simplify");
	    }
	    
	    const PathGeometric& getSolutionPath(void) const
	    {
		if (pdef_->getGoal())
		{
		    const base::PathPtr &p = pdef_->getGoal()->getSolutionPath();
		    if (p)
			return static_cast<const PathGeometric&>(*p);
		}
		throw Exception("No solution path");		
	    }	
	    
	    PathGeometric& getSolutionPath(void)
	    {
		if (pdef_->getGoal())
		{
		    const base::PathPtr &p = pdef_->getGoal()->getSolutionPath();
		    if (p)
			return static_cast<PathGeometric&>(*p);
		}
		throw Exception("No solution path");		
	    }
	    
	protected:
	    
	    base::SpaceInformationPtr     si_;
	    base::ProblemDefinitionPtr    pdef_;
	    base::PlannerPtr              planner_;
	    base::PlannerAllocator        pa_;
	    
	    PathSimplifierPtr             psk_;
	    bool                          configured_;
	    msg::Interface                msg_;
	    
	};
    }
    
}
#endif
