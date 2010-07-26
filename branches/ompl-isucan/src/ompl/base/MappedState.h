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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_MAPPED_STATE_
#define OMPL_BASE_MAPPED_STATE_

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <boost/concept_check.hpp>
#include <cstdlib>

namespace ompl
{
    namespace base
    {
	
	class MappedStateBase
	{
	public:
	    
	    virtual ~MappedStateBase(void)
	    {
	    }
	    
	    virtual const State* getState(void) const = 0;
	    
	};
	
	template <class T = StateManifold>
	class MappedState : public MappedStateBase,  public T::Mapper
	{
	    /** \brief Make sure the type we are casting to is indeed a state manifold */
	    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateManifold*>));
	    BOOST_CONCEPT_ASSERT((boost::Convertible<typename T::Mapper*, StateManifold::Mapper*>));
	    BOOST_CONCEPT_ASSERT((boost::Convertible<typename T::StateType*, State*>));

	public:
	    
	    MappedState(State *state) : T::Mapper(state), stateAlloc_(NULL)
	    {
	    }

	    MappedState(State &state) : T::Mapper(&state), stateAlloc_(NULL)
	    {
	    }

	    MappedState(const StateManifoldPtr &manifold) : T::Mapper(NULL)
	    {
		manifold_ = manifold;
		stateAlloc_ = manifold->allocState();
		T::Mapper::state_ = stateAlloc_;
	    }
	    
	    MappedState(const SpaceInformationPtr &si) : T::Mapper(NULL)
	    {
		manifold_ = si->getStateManifold();
		stateAlloc_ = manifold_->allocState();
		T::Mapper::state_ = stateAlloc_;
	    }
	    
	    MappedState(const StateManifoldPtr &manifold, State *state) : T::Mapper(state), stateAlloc_(NULL), manifold_(manifold)
	    {
	    }
	    
	    MappedState(const SpaceInformationPtr &si, State *state) : T::Mapper(state), stateAlloc_(NULL), manifold_(si->getStateManifold())
	    {
	    }
	    
	    MappedState(const StateManifoldPtr &manifold, State &state) : T::Mapper(&state), stateAlloc_(NULL), manifold_(manifold)
	    {
	    }

	    MappedState(const SpaceInformationPtr &si, State &state) : T::Mapper(&state), stateAlloc_(NULL), manifold_(si->getStateManifold())
	    {
	    }

	    MappedState(const MappedState<T> &other) : T::Mapper(NULL)
	    {
		manifold_ = other.manifold_;
		if (other.stateAlloc_)
		{
		    stateAlloc_ = manifold_->allocState();
		    manifold_->copyState(stateAlloc_, other.stateAlloc_);
		}
		else
		    stateAlloc_ = NULL;
		
		if (other.state_ == other.stateAlloc_)
		    T::Mapper::state_ = stateAlloc_;
		else
		    T::Mapper::state_ = other.state_;
	    }
	    
	    ~MappedState(void)
	    {
		if (stateAlloc_)
		    manifold_->freeState(stateAlloc_);
	    }
	    
	    void use(State *state)
	    {
		T::Mapper::state_ = state;
	    }
	    
	    void use(State &state)
	    {
		T::Mapper::state_ = &state;
	    }
	    
	    /** \brief Set this state to a random value */
	    void random(void)
	    {
		if (!manifold_)
		    throw Exception("No manifold defined for mapped state serving as facade");
		if (!sampler_)
		    sampler_ = manifold_->allocStateSampler();
		sampler_->sample(T::Mapper::state_);
	    }
	    
	    virtual const State* getState(void) const
	    {
		return T::Mapper::state_;
	    }
	    
	    /** \brief Assignment operator */
	    MappedState<T>& operator=(const MappedState<T> &other)
	    {
		if (other.stateAlloc_)
		{
		    if (stateAlloc_)
			manifold_->freeState(stateAlloc_);
		    manifold_ = other.manifold_;
		    stateAlloc_ = manifold_->allocState();
		    manifold_->copyState(stateAlloc_, other.stateAlloc_);
		}
		else
		{
		    if (stateAlloc_)
			manifold_->freeState(stateAlloc_);
		    manifold_ = other.manifold_;
		    stateAlloc_ = NULL;
		}
		
		if (other.state_ == other.stateAlloc_)
		    T::Mapper::state_ = stateAlloc_;
		else
		    T::Mapper::state_ = other.state_;
		return *this;
	    }

	    /** \brief Checks equality of two states */
	    bool operator==(const MappedState<T> &other) const
	    {
		if (manifold_)
		    return manifold_->equalStates(T::Mapper::state_, other.state_);
		if (other.manifold_)
		    return other.manifold_->equalStates( T::Mapper::state_, other.state_);
		throw Exception("Cannot compare two mapped states while both serve only as facade (no manifold defined)");		
	    }

	    /** \brief Checks equality of two states */	    
	    bool operator!=(const MappedState<T> &other) const
	    {
		return !(*this == other);
	    }
	    
	    /** \brief De-references to the contained state */
	    typename T::StateType& operator*(void) const
	    {
		return static_cast<typename T::StateType&>(*T::Mapper::state_);
	    }
	    
	    /** \brief Returns a pointer to the contained state */
	    typename T::StateType* operator->(void) const
	    {
		return static_cast<typename T::StateType*>(T::Mapper::state_);
	    }
	    
	    /** \brief Returns a pointer to the contained state */
	    typename T::StateType* get(void) const
	    {
		return static_cast<typename T::StateType*>(T::Mapper::state_);
	    }
	    
	    /** \brief Returns a reference to the contained state */
	    typename T::StateType& reference(void) const
	    {
		return static_cast<typename T::StateType&>(*T::Mapper::state_);
	    }
	    
	private:
	    
	    State            *stateAlloc_;
	    StateManifoldPtr  manifold_;
	    StateSamplerPtr   sampler_;
	};
    }
}

#endif
