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
#include "ompl/base/Manifold.h"
#include "ompl/base/StateSampler.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"

#include <cstdlib>
#include <vector>
#include <iostream>

/** \brief Main namespace */
namespace ompl
{
    
    /** \brief This namespace contains sampling based planning
	routines shared by both planning under geometric constraints
	(kinematic) and planning under differential constraints
	(dynamic) */
    namespace base
    {

	ClassForward(SpaceInformation);
	
	/** \brief The base class for space information. This contains
	    all the information about the space planning is done in.
	    setup() needs to be called as well, before use */
	class SpaceInformation
	{
	public:
	    
	    /** \brief Constructor. Sets the instance of the manifold
		to plan on. */
	    SpaceInformation(Manifold *manifold) : m_manifold(manifold)
	    {
		m_setup = false;
		m_stateValidityChecker = NULL;
	    }
	    
	    /** \brief Destructor */
	    virtual ~SpaceInformation(void)
	    {
	    }
	    	    
	    /** \brief Return the instance of the used manifold */
	    Manifold* getManifold(void)
	    {
		return m_manifold;
	    }	
	    
	    /** \brief Return the instance of the used manifold */
	    const Manifold* getManifold(void) const
	    {
		return m_manifold;
	    }	
	    
	    /** \brief Set the instance of the validity checker to
		use. No memory freeing is performed. Parallel
		implementations of planners assume this validity
		checker is thread safe. */
	    void setStateValidityChecker(StateValidityChecker *svc)
	    {
		m_stateValidityChecker = svc;
	    }
	    
	    /** \brief Return the instance of the used state validity checker */
	    StateValidityChecker* getStateValidityChecker(void)
	    {
		return m_stateValidityChecker;
	    }	
	    
	    /** \brief Return the instance of the used state validity checker */
	    const StateValidityChecker* getStateValidityChecker(void) const
	    {
		return m_stateValidityChecker;
	    }
	    
	    /** \brief Return the dimension of the state space */
	    unsigned int getStateDimension(void) const
	    {
		return m_manifold->getDimension();
	    }
	    
	    /** \brief Check if a given state is valid or not */
	    bool isValid(const State *state) const
	    {
		return (*m_stateValidityChecker)(state);
	    }

	    /** \brief Allocate memory for a state */
	    State* allocState(void) const
	    {
		return m_manifold->allocState();
	    }
	    
	    /** \brief Free the memory of a state */
	    void freeState(State *state) const
	    {
		m_manifold->freeState(state);
	    }
	    
	    /** \brief Copy a state to another */
	    void copyState(State *destination, const State *source) const
	    {
		m_manifold->copyState(destination, source);
	    }
	    
	    /** \brief Check if two states are the same */
	    bool equalStates(const State *state1, const State *state2) const
	    {
		return m_manifold->equalStates(state1, state2);
	    }
	    
	    /** \brief Check if a state is inside the bounding box */
	    bool satisfiesBounds(const State *state) const
	    {
		return m_manifold->satisfiesBounds(state);
	    }
	    
	    /** \brief Compute the distance between two states */
	    double distance(const State *state1, const State *state2) const
	    {
		return m_manifold->distance(state1, state2);
	    }

	    /** \brief Bring the state within the bounds of the state space */
	    void enforceBounds(State *state) const
	    {
		m_manifold->enforceBounds(state);
	    }

	    /** \brief Allocate a state sampler */
	    StateSamplerPtr allocStateSampler(void) const
	    {
		return m_manifold->allocStateSampler();
	    }
	    
	    /** \brief Find a valid state near a given one. If the given state is valid, it will be returned itself.
	     *  The two passed state pointers must point to different states. Returns true on success.  */
	    bool searchValidNearby(State *state, const State *near, double distance, unsigned int attempts) const;
	    
	    /** \brief Print a state to a stream */
	    void printState(const State *state, std::ostream &out = std::cout) const
	    {
		m_manifold->printState(state, out);
	    }
	    
	    /** \brief Print information about the current instance of the state space */
	    virtual void printSettings(std::ostream &out = std::cout) const;
	    
	    /** \brief Perform additional setup tasks (run once, before use) */
	    virtual void setup(void);
	    
	    /** \brief Return true if setup was called */
	    bool isSetup(void) const;
	    
	protected:
	    
	    StateValidityChecker        *m_stateValidityChecker;
	    Manifold                    *m_manifold;
	    
	    bool                         m_setup;

	    msg::Interface               m_msg;
	};
	
    }
    
}
    
#endif
