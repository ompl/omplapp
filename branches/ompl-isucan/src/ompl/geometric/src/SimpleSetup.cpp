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

#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRT.h"

void ompl::geometric::SimpleSetup::setup(void)
{
    if (!configured_)
    {
	si_->setup();
	if (!planner_)
	{
	    if (pa_)
		planner_ = pa_(si_);
	    else
	    {
		msg_.inform("No planner specified. Using default.");
		const base::GoalPtr &goal = getGoal();
		if (goal && dynamic_cast<const base::GoalSampleableRegion*>(goal.get()))
		    planner_ = base::PlannerPtr(new RRTConnect(si_));
		else
		    planner_ = base::PlannerPtr(new RRT(si_));
	    }
	}
	planner_->setProblemDefinition(pdef_);
	planner_->setup();
	configured_ = true;
    }
}

void ompl::geometric::SimpleSetup::clear(void)
{
    if (planner_)
	planner_->clear();
    if (pdef_->getGoal())
	pdef_->getGoal()->clearSolutionPath();
    pdef_->clearStartStates();
}

void ompl::geometric::SimpleSetup::simplifySolution(void)
{
    const base::PathPtr &p =  pdef_->getGoal()->getSolutionPath();
    if (p)
	psk_->simplifyMax(static_cast<PathGeometric&>(*p));
    else
	msg_.warn("No solution to simplify");
}

const ompl::geometric::PathGeometric& ompl::geometric::SimpleSetup::getSolutionPath(void) const
{
    if (pdef_->getGoal())
    {
	const base::PathPtr &p = pdef_->getGoal()->getSolutionPath();
	if (p)
	    return static_cast<const PathGeometric&>(*p);
    }
    throw Exception("No solution path");		
}	

ompl::geometric::PathGeometric& ompl::geometric::SimpleSetup::getSolutionPath(void)
{
    if (pdef_->getGoal())
    {
	const base::PathPtr &p = pdef_->getGoal()->getSolutionPath();
	if (p)
	    return static_cast<PathGeometric&>(*p);
    }
    throw Exception("No solution path");		
}
