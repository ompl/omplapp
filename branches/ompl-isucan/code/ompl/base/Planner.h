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

/* \author Ioan Sucan */

#ifndef OMPL_BASE_PLANNER_
#define OMPL_BASE_PLANNER_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/util/Console.h"
#include "ompl/util/Time.h"
#include "ompl/util/ClassForward.h"
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>
#include <vector>

namespace ompl
{

    namespace base
    {
	
	/** \brief Different planners may be able to handle only specific types of goal regions. For instance, the most
	    general goal representation is not suitable for bi-directional planners. Planners set their type to specify 
	    which type of goal regions they can handle.*/
	enum PlannerType
	    {
		/** \brief This value should not be set */
		PLAN_UNKNOWN        = 0,

		/** \brief This bit is set if planning to goal states (ompl::base::GoalState) is possible */
		PLAN_TO_GOAL_STATE  = 1,

		/** \brief This bit is set if planning to sampleable goal regions (ompl::base::GoalSampleableRegion) is possible */
		PLAN_TO_GOAL_SAMPLEABLE_REGION = 2 | PLAN_TO_GOAL_STATE,

		/** \brief This bit is set if planning to goal regions (ompl::base::GoalRegion) is possible */
		PLAN_TO_GOAL_REGION = 4 | PLAN_TO_GOAL_SAMPLEABLE_REGION,
		
		/** \brief This bit is set if planning to generic goal regions (ompl::base::Goal) is possible */
		PLAN_TO_GOAL_ANY    = 32768 | PLAN_TO_GOAL_REGION
	    };
	
	ClassForward(Planner);
	ClassForward(PlannerData);
	
	/** \brief Datatype holding data a planner can expose for debug purposes. */
	class PlannerData
	{
	public:
	    PlannerData(void)
	    {
	    }
	    
	    virtual ~PlannerData(void)
	    {
	    }
	    
	    /** \brief The list of states in the current exploration datastructure */
	    std::vector<const State*> states;
	};
	
	/** \brief Base class for a planner */
	class Planner : private boost::noncopyable
	{
	    
	public:
	    
	    /** \brief Constructor */
	    Planner(const SpaceInformationPtr &si);
	    
	    /** \brief Destructor */
	    virtual ~Planner(void)
	    {
	    }

	    /** \brief Get the problem definition the planner is trying to solve */
	    const ProblemDefinitionPtr& getProblemDefinition(void) const;
	    
	    /** \brief Set the problem definition for the planner. The
		problem needs to be set before calling solve(). This
		also clears the internal datastructures of the planner. */
	    void setProblemDefinition(const ProblemDefinitionPtr &pdef);
	    
	    /** \brief Function that can solve the motion planning
		problem. This function can be called multiple times on
		the same problem, without calling clear() in
		between. This allows the planner to continue work more
		time on an unsolved problem, for example. If this
		option is used, it is assumed the problem definition
		is not changed (unpredictable results otherwise). The
		only change in the problem definition that is
		accounted for is the addition of starting or goal
		states (but not changing previously added start/goal states). */
	    virtual bool solve(double solveTime) = 0;
	    
	    /** \brief Clear all internal datastructures. Subsequent
		calls to solve() will ignore all previous work. */
	    virtual void clear(void) = 0;
	    
	    /** \brief Get information about the current run of the motion planner  */
	    virtual void getPlannerData(PlannerData &data) const = 0;
	    
	    /** \brief Return the type of the motion planner. This is useful if
		the planner wants to advertise what type of problems it
		can solve */
	    PlannerType getType(void) const;
	    
	    /** \brief Perform extra configuration steps, if needed. This must be called before solving */
	    virtual void setup(void);
	    
	protected:
	    
	    SpaceInformationPtr  m_si;
	    ProblemDefinitionPtr m_pdef;
	    PlannerType          m_type;	
	    bool                 m_setup;
	    msg::Interface       m_msg;
	};

	/** \brief Definition of a function that can allocate a planner */
	typedef boost::function<PlannerPtr(const SpaceInformationPtr&)> PlannerAllocator;
    }
}


#endif
