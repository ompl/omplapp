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

#ifndef OMPL_BASE_GOAL_
#define OMPL_BASE_GOAL_

#include "ompl/base/State.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/Path.h"
#include "ompl/util/ClassForward.h"
#include <iostream>
#include <limits>

namespace ompl
{
    namespace base
    {
	
	ClassForward(Goal);
	
	/** \brief Abstract definition of goals. Will contain solutions, if found */
	class Goal
	{
	public:
	    
	    /** \brief Constructor. The goal must always know the space information it is part of */
	    Goal(const SpaceInformationPtr &si) : m_si(si), m_maximumPathLength(std::numeric_limits<double>::max()),
						  m_difference(-1.0), m_approximate(false)
	    {
	    }
	    
	    /** \brief Destructor. Clears the solution as well */
	    virtual ~Goal(void)
	    {
	    }
	    
	    /** \brief Return true if the state statisfies the goal
	     *  constraints and compute the distance between the state
	     *  given as argument and the goal (even if the goal is
	     *  not satisfied). This distance can be an approximation.
	     *  Note: if this function returns true,
	     *  isStartGoalPairValid() need not be called. */
	    virtual bool isSatisfied(const State *st, double *distance) const = 0;

	    /** \brief Return true if the state statisfies the goal
	     *  constraints and the path length is less than the
	     *  desired maximum length.  This call aslo computes the
	     *  distance between the state given as argument and the
	     *  goal. */
	    bool isSatisfied(const State *st, double pathLength, double *distance) const
	    {
		if (pathLength > m_maximumPathLength)
		{
		    if (distance != NULL)
			isSatisfied(st, distance);
		    return false;
		}
		else
		    return isSatisfied(st, distance);
	    }
	    
	    /** \brief Since there can be multiple starting states
		(and multiple goal states) it is possible certain
		pairs are not to be allowed. By default we however
		assume all such pairs are allowed. Note: if this
		function returns true, isSatisfied() need not be
		called. */
	    virtual bool isStartGoalPairValid(const State * /* start */, const State * /* goal */) const 
	    {
		return true;
	    }
	    
	    /** \brief Returns true if a solution path has been found (could be approximate) */
	    bool isAchieved(void) const
	    {
		return m_path;
	    }

	    /** \brief Get the maximum length allowed for a solution path */
	    double getMaximumPathLength(void) const
	    {
		return m_maximumPathLength;
	    }

	    /** \brief Set the maximum length allowed for a solution
		path. This value is checked only in the version of
		isSatisfied() that takes the path length as
		argument */
	    void setMaximumPathLength(double maximumPathLength)
	    {
		m_maximumPathLength = maximumPathLength;
	    }
	    
	    /** \brief Return the found solution path. 

		This will need to be casted into the specialization
		computed by the planner */
	    const PathPtr& getSolutionPath(void) const
	    {
		return m_path;
	    }	    
	    
	    /** \brief Update the solution path. If a previous solution path exists, it is deleted. */
	    void setSolutionPath(const PathPtr &path, bool approximate = false)
	    {
		m_path = path;
		m_approximate = approximate;
	    }

	    /** \brief Forget the solution path. Memory is freed. */
	    void clearSolutionPath(void)
	    {
		m_path.reset();
	    }
	    
	    /** \brief If a difference between the desired solution and the
		solution found is computed by the planner, this functions
		returns it */
	    double getDifference(void) const
	    {
		return m_difference;
	    }
	    
	    /** \brief Set the difference between the found solution
		path and the desired solution path */
	    void setDifference(double difference)
	    {
		m_difference = difference;
	    }
	    
	    /** \brief Return true if the found solution is
		approximate (does not actually reach the desired goal,
		but hopefully is closer to it) */
	    bool isApproximate(void) const
	    {
		return m_approximate;
	    }
	    
	    /** \brief Print information about the goal */
	    virtual void print(std::ostream &out = std::cout) const
	    {
		out << "Goal memory address " << this << std::endl;
	    }
	    
	protected:
	    
	    /** \brief The space information for this goal */
	    SpaceInformationPtr      m_si;
	    
	    /** \brief The maximum length allowed for the solution path */
	    double                   m_maximumPathLength;
	    
	    /** \brief Solution path, if found */
	    PathPtr                  m_path;
	    	    
	    /** \brief The achieved difference between the found solution and the desired goal */
	    double                   m_difference;
	    
	    /** \brief True if goal was not achieved, but an approximate solution was found */
	    bool                     m_approximate;

	};
		
    }
}

#endif
