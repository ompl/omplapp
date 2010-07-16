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

#ifndef OMPL_BASE_SPACE_INFORMATION_
#define OMPL_BASE_SPACE_INFORMATION_

#include "ompl/base/State.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/StateManifold.h"
#include "ompl/base/StateSampler.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"

#include <boost/noncopyable.hpp>

#include <cstdlib>
#include <vector>
#include <iostream>

/** \brief Main namespace. Contains everything in this library */
namespace ompl
{
    
    /** \brief This namespace contains sampling based planning
	routines shared by both planning under geometric constraints
	(geometric) and planning under differential constraints
	(dynamic) */
    namespace base
    {

	ClassForward(SpaceInformation);
	
	/** \brief The base class for space information. This contains
	    all the information about the space planning is done in.
	    setup() needs to be called as well, before use */
	class SpaceInformation : private boost::noncopyable
	{
	public:
	    
	    /** \brief Constructor. Sets the instance of the manifold
		to plan on. */
	    SpaceInformation(const StateManifoldPtr &manifold);
	    
	    /** \brief Destructor */
	    virtual ~SpaceInformation(void)
	    {
	    }
	    	    
	    /** \brief Return the instance of the used manifold */
	    const StateManifoldPtr& getStateManifold(void) const
	    {
		return m_stateManifold;
	    }	
	    
	    /** \brief Set the instance of the validity checker to
		use. No memory freeing is performed. Parallel
		implementations of planners assume this validity
		checker is thread safe. */
	    void setStateValidityChecker(const StateValidityCheckerPtr &svc)
	    {
		m_stateValidityChecker = svc;
	    }
	    
	    /** \brief Return the instance of the used state validity checker */
	    const StateValidityCheckerPtr& getStateValidityChecker(void) const
	    {
		return m_stateValidityChecker;
	    }	
	    
	    /** \brief Set the resolution (maximum distance between states) at which state validity needs to be
		verified in order for a motion between two states to be considered valid */
	    void setStateValidityCheckingResolution(double resolution)
	    {
		m_resolution = resolution;
	    }
	    
	    /** \brief Get the resolution (maximum distance between states) at which state validity is verified */
	    double getStateValidityCheckingResolution(void) const
	    {
		return m_resolution;
	    }
	    
	    /** \brief Return the dimension of the state space */
	    unsigned int getStateDimension(void) const
	    {
		return m_stateManifold->getDimension();
	    }
	    
	    /** \brief Check if a given state is valid or not */
	    bool isValid(const State *state) const
	    {
		return m_stateValidityChecker->isValid(state);
	    }

	    /** \brief Allocate memory for a state */
	    State* allocState(void) const
	    {
		return m_stateManifold->allocState();
	    }
	    
	    /** \brief Free the memory of a state */
	    void freeState(State *state) const
	    {
		m_stateManifold->freeState(state);
	    }

	    /** \brief Print a state to a stream */
	    void printState(const State *state, std::ostream &out = std::cout) const
	    {
		m_stateManifold->printState(state, out);
	    }

	    /** \brief Copy a state to another */
	    void copyState(State *destination, const State *source) const
	    {
		m_stateManifold->copyState(destination, source);
	    }
	    
	    /** \brief Clone a state */
	    State* cloneState(const State *source) const
	    {
		State *copy = m_stateManifold->allocState();
		m_stateManifold->copyState(copy, source);
		return copy;
	    }
	    
	    /** \brief Check if two states are the same */
	    bool equalStates(const State *state1, const State *state2) const
	    {
		return m_stateManifold->equalStates(state1, state2);
	    }
	    
	    /** \brief Check if a state is inside the bounding box */
	    bool satisfiesBounds(const State *state) const
	    {
		return m_stateManifold->satisfiesBounds(state);
	    }
	    
	    /** \brief Compute the distance between two states */
	    double distance(const State *state1, const State *state2) const
	    {
		return m_stateManifold->distance(state1, state2);
	    }

	    /** \brief Bring the state within the bounds of the state space */
	    void enforceBounds(State *state) const
	    {
		m_stateManifold->enforceBounds(state);
	    }

	    /** \brief Allocate a state sampler */
	    StateSamplerPtr allocStateSampler(void) const
	    {
		return m_stateManifold->allocStateSampler();
	    }
	    
	    /** \brief Estimate the maximum (underapproximation)
		extent of the space we are planning in. This is done
		through random sampling. */
	    virtual double estimateExtent(unsigned int samples = 10);
	    
	    /** \brief Find a valid state near a given one. If the given state is valid, it will be returned itself.
	     *  The two passed state pointers must point to different states. Returns true on success.  */
	    virtual bool searchValidNearby(State *state, const State *near, double distance, unsigned int attempts) const;
	    
	    /** \brief Incrementally check if the path between two motions is valid. Also compute the last state that was
		valid and the time of that state. The time is used to parametrize the motion from s1 to s2, s1 being at t =
		0 and s2 being at t = 1. This function assumes s1 is valid. */
	    virtual bool checkMotion(const State *s1, const State *s2, State *lastValidState, double *lastValidTime) const;
	    
	    /** \brief Check if the path between two motions is valid using subdivision. This function assumes s1 is valid. */
	    virtual bool checkMotion(const State *s1, const State *s2) const;
	    
	    /** \brief Incrementally check if a sequence of states is valid. Given a vector of states, this routine only
		checks the first count elements and marks the index of the first invalid state */
	    virtual bool checkMotion(const std::vector<State*> &states, unsigned int count, unsigned int *firstInvalidStateIndex) const;
	    
	    /** \brief Check if a sequence of states is valid using subdivision. */
	    virtual bool checkMotion(const std::vector<State*> &states, unsigned int count) const;
	    
	    /** \brief Get the states that make up a motion. Returns the number of states that were added.

		The states are added at the distance specified by the collision checking resolution times the factor
		specified. A factor larger than 1 will result in fewer states per motion. The endpoints (s1 and s2) can 
		optionally be part of the computed set of states. */
	    virtual unsigned int getMotionStates(const State *s1, const State *s2, std::vector<State*> &states, double factor, bool endpoints, bool alloc) const;

	    /** \brief Print information about the current instance of the state space */
	    virtual void printSettings(std::ostream &out = std::cout) const;
	    
	    /** \brief Perform additional setup tasks (run once, before use) */
	    virtual void setup(void);
	    
	    /** \brief Return true if setup was called */
	    bool isSetup(void) const;
	    
	protected:
	    
	    StateValidityCheckerPtr m_stateValidityChecker;
	    StateManifoldPtr        m_stateManifold;
	    double                  m_resolution;
	    double                  m_maxExtent;
	    
	    bool                    m_setup;

	    msg::Interface          m_msg;
	};
	
    }
    
}
    
#endif
