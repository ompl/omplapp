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
	    SimpleSetup(const base::StateManifoldPtr &manifold, const base::PlannerAllocator &pa) : m_configured(false)
	    {
		m_si.reset(new base::SpaceInformation(manifold));
		m_pdef.reset(new base::ProblemDefinition(m_si));
		m_psk.reset(new PathSimplifier(m_si));
		m_pa = pa;
	    }
	    
	    ~SimpleSetup(void)
	    {
	    }
	    
	    const base::SpaceInformationPtr& getSpaceInformation(void) const
	    {
		return m_si;
	    }
	    
	    const base::StateManifoldPtr& getStateManifold(void) const
	    {
		return m_si->getStateManifold();
	    }
	    
	    const base::ProblemDefinitionPtr& getProblemDefinition(void) const
	    {
		return m_pdef;
	    }
	    
	    const base::StateValidityCheckerPtr& getStateValidityChecker(void) const
	    {
		return m_si->getStateValidityChecker();
	    }

	    void setStateValidityChecker(const base::StateValidityCheckerPtr &svc) const
	    {
		return m_si->setStateValidityChecker(svc);
	    }

	    const base::GoalPtr& getGoal(void) const
	    {
		return m_pdef->getGoal();
	    }

	    void setGoal(const base::GoalPtr &goal)
	    {
		m_pdef->setGoal(goal);
	    }

	    const base::PlannerPtr& getPlanner(void) const
	    {
		return m_planner;
	    }

	    const PathSimplifierPtr& getPathSimplifier(void) const
	    {
		return m_psk;
	    }
	    
	    void setup(void)
	    {
		if (!m_configured)
		{
		    m_si->setup();
		    m_planner = m_pa(m_si);
		    m_planner->setProblemDefinition(m_pdef);
		    m_planner->setup();
		    m_configured = true;
		}
	    }
	    
	    bool solve(double time)
	    {
		setup();
		return m_planner->solve(time);
	    }
	    
	    void clear(void)
	    {
		if (m_planner)
		    m_planner->clear();
		if (m_pdef->getGoal())
		    m_pdef->getGoal()->clearSolutionPath();
		m_pdef->clearStartStates();
	    }
	    	    
	    void simplifySolution(void)
	    {
		const base::PathPtr &p =  m_pdef->getGoal()->getSolutionPath();
		if (p)
		    m_psk->simplifyMax(static_cast<PathGeometric&>(*p));
		else
		    m_msg.warn("No solution to simplify");
	    }
	    
	    const PathGeometric& getSolutionPath(void) const
	    {
		if (m_pdef->getGoal())
		{
		    const base::PathPtr &p = m_pdef->getGoal()->getSolutionPath();
		    if (p)
			return static_cast<const PathGeometric&>(*p);
		}
		throw Exception("No solution path");		
	    }	
	    
	    PathGeometric& getSolutionPath(void)
	    {
		if (m_pdef->getGoal())
		{
		    const base::PathPtr &p = m_pdef->getGoal()->getSolutionPath();
		    if (p)
			return static_cast<PathGeometric&>(*p);
		}
		throw Exception("No solution path");		
	    }
	    
	protected:
	    
	    base::SpaceInformationPtr     m_si;
	    base::ProblemDefinitionPtr    m_pdef;
	    base::PlannerPtr              m_planner;
	    base::PlannerAllocator        m_pa;
	    
	    PathSimplifierPtr             m_psk;
	    bool                          m_configured;
	    msg::Interface                m_msg;
	    
	};
    }
    
}
#endif
