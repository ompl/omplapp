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

#ifndef OMPL_BASE_TOPOLOGY_
#define OMPL_BASE_TOPOLOGY_

#include "ompl/base/General.h"
#include "ompl/base/State.h"
#include <iostream>
#include <vector>
#include <cstdlib>

namespace ompl
{
    namespace base
    {
	class StateSampler;
	
	class Manifold
	{
	public:
	    
	    Manifold(void) : m_lowerBound(NULL), m_upperBound(NULL)
	    {
	    }
	    
	    virtual ~Manifold(void);

	    /** \brief Get the dimension of the space */
	    virtual unsigned int getDimension(void) const = 0;

	    /** \brief Bring the state within the bounds of the state space */
	    virtual void enforceBounds(State *state) const = 0;
	    
	    /** \brief Check if a state is inside the bounding box */
	    virtual bool satisfiesBounds(const State *state) const = 0;

	    /** \brief Copy a state to another */
	    virtual void copyState(State *destination, const State *source) const = 0;
	    
	    /** \brief Computes distance to between two states */
	    virtual double distance(const State *state1, const State *state2) const = 0;
	    
	    /** \brief Checks whether two states are equal */
	    virtual bool equalStates(const State *state1, const State *state2) const = 0;

	    /** \brief Computes the state that lies at time t \in [0, 1] on the
		segment that connects the current state to the
		destination state */
	    virtual void interpolate(const State *from, const State *to, const double t, State *state) const = 0;

	    /** \brief Allocate an instance of a state sampler for this space */
	    virtual StateSampler* allocStateSampler(void) const = 0;
	    
	    /** \brief Allocate a state that can store a point in the described space */
	    virtual State* allocState(void) const = 0;
	    
	    /** \brief Free the memory of the allocated state */
	    virtual void freeState(State *state) const = 0;
	    
	    /** \brief Set the bounds of this manifold. This defines
		the subset of the manifold in which sampling takes
		place and where valid states can exist. */
	    virtual void setBounds(const State *lower, const State *upper);

	    /** \brief Set the upper bound of this manifold. */
	    virtual void setUpperBound(const State *bound);

	    /** \brief Set the upper bound of this manifold. */
	    virtual void setLowerBound(const State *bound);
	    
	    /** \brief Print a state to screen */
	    virtual void printState(const State *state, std::ostream &out) const;
	    
	protected:

	    /** \brief Lower bound for the manifold */
	    State *m_lowerBound;

	    /** \brief Upper bound for the manifold */
	    State *m_upperBound;
	};
		
    	class CompoundManifold : public Manifold
	{
	public:
	    
	    CompoundManifold(void) : Manifold()
	    {
	    }
	    
	    virtual ~CompoundManifold(void);
	    
	    /** \brief Adds the topology of a space as part of the
		compound space. For computing distances within the
		compound space, the weight of the component also needs
		to be specified. Ownership of the passed topology
		instance is assumed and memory is freed upon
		destruction. */
	    virtual void addManifold(Manifold *component, double weight);

	    virtual unsigned int getDimension(void) const;

	    virtual void enforceBounds(State *state) const;
	    
	    virtual bool satisfiesBounds(const State *state) const;

	    virtual void copyState(State *destination, const State *source) const;
	    
	    virtual double distance(const State *state1, const State *state2) const;
	    
	    virtual bool equalStates(const State *state1, const State *state2) const;
	    
	    virtual void interpolate(const State *from, const State *to, const double t, State *state) const;
	    
	    virtual StateSampler* allocStateSampler(void) const;
	    
	    virtual State* allocState(void) const;
	    
	    virtual void freeState(State *state) const;	 

	    virtual void printState(const State *state, std::ostream &out) const;

	    virtual void setUpperBound(const State *bound);

	    virtual void setLowerBound(const State *bound);

	protected:
	    
	    std::vector<Manifold*> m_components;
	    std::vector<double>    m_weights;
	    
	};
    }
}

#endif
