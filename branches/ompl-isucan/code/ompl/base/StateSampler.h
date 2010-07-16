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

#ifndef OMPL_BASE_STATE_SAMPLER_
#define OMPL_BASE_STATE_SAMPLER_

#include "ompl/base/State.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/util/ClassForward.h"
#include <vector>
#include <boost/function.hpp>

namespace ompl
{
    namespace base
    {
	
	ClassForward(StateManifold);
	ClassForward(StateSampler);
	
	/** \brief Abstract definition of a state sampler. */
	class StateSampler
	{	    
	public:

	    /** \brief Constructor */
	    StateSampler(const StateManifold *manifold) : m_manifold(manifold)
	    {
	    }
	    
	    /** \brief Destructor */
	    virtual ~StateSampler(void)
	    {
	    }
	    
	    /** \brief Sample a state */
	    virtual void sample(State *state) = 0;
	    
	    /** \brief Sample a state near another, within specified distance */
	    virtual void sampleNear(State *state, const State *near, const double distance) = 0;
	    
	    /** \brief Return a reference to the random number generator used */
	    RNG& getRNG(void)
	    {
		return m_rng;
	    }
	    
	protected:
	    
	    const StateManifold *m_manifold;
	    RNG                  m_rng;
	};

	/** \brief Definition of a compound state sampler. This is useful to construct samplers for compound states. */
	class CompoundStateSampler : public StateSampler
	{	    
	public:

	    /** \brief Constructor */
	    CompoundStateSampler(const StateManifold* manifold) : StateSampler(manifold) 
	    {
	    }
	    
	    /** \brief Destructor. This frees the added samplers as well. */
	    virtual ~CompoundStateSampler(void)
	    {
	    }
	    
	    /** \brief Add a sampler as part of the new compound
		sampler. This sampler is used to sample part of the
		compound state. Ownership of the passed sampler
		instance is assumed and memory is freed upon
		destruction. */
	    virtual void addSampler(const StateSamplerPtr &sampler);
	    
	    /** \brief Sample a state. */
	    virtual void sample(State *state);
	    
	    /** \brief Sample a state near another, within specified distance. */
	    virtual void sampleNear(State *state, const State *near, const double distance);
	    
	protected:
	    
	    std::vector<StateSamplerPtr> m_samplers;
	    std::size_t                  m_samplerCount;
	    
	};

	/** \brief Definition of a function that can allocate a state sampler */
	typedef boost::function<StateSamplerPtr(const StateManifold*)> StateSamplerAllocator;
    }
}


#endif
