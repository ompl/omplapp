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
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

namespace ompl
{

    namespace geometric
    {
		
	/** \brief Create the set of classes typically needed to solve a
	    geometric problem */
	class SimpleSetup
	{
	public:
	    SimpleSetup(const base::StateManifoldPtr &manifold, const base::PlannerAllocator &pa = base::PlannerAllocator()) : configured_(false)
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
		si_->setStateValidityChecker(svc);
	    }
	    
	    void setStateValidityChecker(const base::StateValidityCheckerFn &svc) const
	    {
		si_->setStateValidityChecker(svc);
	    }

	    void setStartAndGoalStates(const base::ScopedState<> &start, const base::ScopedState<> &goal)
	    {
		pdef_->setStartAndGoalStates(start, goal);
	    }
	    
	    void addStartState(const base::ScopedState<> &state)
	    {
		pdef_->addStartState(state);
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
	    
	    void setPlanner(const base::PlannerPtr &planner)
	    {
		planner_ = planner;
	    }
	    
	    const PathSimplifierPtr& getPathSimplifier(void) const
	    {
		return psk_;
	    }
	    
	    void setup(void);
	    
	    bool solve(double time)
	    {
		setup();
		return planner_->solve(time);
	    }
	    
	    void clear(void);
	    	    
	    void simplifySolution(void);
	    const PathGeometric& getSolutionPath(void) const;	    
	    PathGeometric& getSolutionPath(void);
	    
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
