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

#ifndef OMPL_BASE_SCOPED_STATE_
#define OMPL_BASE_SCOPED_STATE_

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <boost/concept_check.hpp>

namespace ompl
{
    namespace base
    {
	
	
	class ScopedState
	{
	public:
	    /** \brief Given the space that we are working with,
		allocate a state from the corresponding
		manifold. */
	    explicit
	    ScopedState(const SpaceInformationPtr &si) : manifold_(si->getStateManifold())
	    {	
		state_ = manifold_->allocState();
	    }
	    
	    /** \brief Given the manifold that we are working with,
		allocate a state. */
	    explicit
	    ScopedState(const StateManifoldPtr &manifold) : manifold_(manifold)
	    {
		state_ = manifold_->allocState();
	    }

	    /** \brief Copy constructor */
	    ScopedState(const ScopedState &other) : manifold_(other.getManifold())
	    {
		state_ = manifold_->allocState();
		manifold_->copyState(state_, other.get());
	    }

	    /** \brief Assignment operator. */
	    ScopedState& operator=(const ScopedState &other)
	    {
		if (&other != this)
		{
		    manifold_->freeState(state_);
		    manifold_ = other.getManifold();
		    
		    state_ = manifold_->allocState();
		    manifold_->copyState(state_, other.get());
		}
		return *this;
	    }

	    /** \brief Free the memory of the internally allocated state */	    
	    virtual ~ScopedState(void)
	    {
		if (state_)
		    manifold_->freeState(state_);
	    }

	    /** \brief Get the manifold that the state corresponds to */
	    const StateManifoldPtr& getManifold(void) const
	    {
		return manifold_;
	    }

	    /** \brief Checks equality of two states */
	    bool operator==(const ScopedState &other) const
	    {
		return manifold_->equalStates(state_, other.get());
	    }

	    /** \brief Checks equality of two states */	    
	    bool operator!=(const ScopedState &other) const
	    {
		return !(*this == other);
	    }
	    
	    /** \brief Set this state to a random value */
	    void random(void)
	    {
		if (!sampler_)
		    sampler_ = manifold_->allocStateSampler();
		sampler_->sample(state_);
	    }
	    
	    /** \brief De-references to the contained state */
	    State& operator*(void) const
	    {
		return *state_;
	    }
	    
	    /** \brief Returns a pointer to the contained state */
	    State* operator->(void) const
	    {
		return state_;
	    }
	    
	    /** \brief Returns a pointer to the contained state */
	    State* get(void) const
	    {
		return state_;
	    }
	    
	    /** \brief Returns a reference to the contained state */
	    State& reference(void) const
	    {
		return *state_;
	    }
	    
	protected:

	    StateManifoldPtr     manifold_;
	    State               *state_;
	    StateSamplerPtr      sampler_;
	    
	};
	    
	/** \brief Definition of a scoped state. 

	    This class allocates a state of a desired type using the
	    allocation mechanism of the manifold the state is part
	    of. The state is then freed when the instance goes out of
	    scope using the corresponding free mechanism. */
	template<class T = State>
	class ScopedStateTyped : public ScopedState
	{
	    
	    /** \brief Make sure the type we are allocating is indeed a state */
	    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));
	    
	public:
	    
	    /** \brief The state type this class instance assumes */
	    typedef T StateType;
	    
	    /** \brief Given the space that we are working with,
		allocate a state from the corresponding
		manifold. Throw an exception if the desired type to
		cast this state into does not match the type of states
		allocated. */
	    explicit
	    ScopedStateTyped(const SpaceInformationPtr &si) : ScopedState(si)
	    {	
		if (!dynamic_cast<T*>(state_))
		    throw Exception("StateManifold does not allocate states of desired type");
	    }
	    
	    /** \brief Given the manifold that we are working with,
		allocate a state. Throw an exception if the desired
		type to cast this state into does not match the type
		of states allocated. */
	    explicit
	    ScopedStateTyped(const StateManifoldPtr &manifold) : ScopedState(manifold)
	    {
		if (!dynamic_cast<T*>(state_))
		    throw Exception("StateManifold does not allocate states of desired type");
	    }
	    
	    /** \brief Copy constructor */
	    ScopedStateTyped(const ScopedStateTyped<T> &other) : ScopedState(other)
	    { 
	    }
	    
	    /** \brief Copy constructor that allows instantiation from states of other type */
	    template<class O>
	    ScopedStateTyped(const ScopedStateTyped<O> &other) : ScopedState(other.getManifold())
	    { 
		BOOST_CONCEPT_ASSERT((boost::Convertible<O*, State*>));
		
		if (!dynamic_cast<const T*>(other.get()))
		    throw Exception("Unable to copy state");
		
		static_cast<ScopedState&>(*this) = static_cast<const ScopedState&>(other);
	    }
	    
	    /** \brief Free the memory of the internally allocated state */
	    virtual ~ScopedStateTyped(void)
	    {	
	    }	    
	    
	    /** \brief Assignment operator */
	    ScopedStateTyped<T>& operator=(const ScopedState &other)
	    {
		static_cast<ScopedState&>(*this) = other;
		return *this;
	    }

	    /** \brief Assignment operator */
	    ScopedStateTyped<T>& operator=(const ScopedStateTyped<T> &other)
	    {
		static_cast<ScopedState&>(*this) = static_cast<const ScopedState&>(other);
		return *this;
	    }
	    
	    /** \brief Assignment operator that allows conversion of states */
	    template<class O>
	    ScopedStateTyped<T>& operator=(const ScopedStateTyped<O> &other)
	    {
		BOOST_CONCEPT_ASSERT((boost::Convertible<O*, State*>));

		if (!dynamic_cast<const T*>(other.get()))
		    throw Exception("Unable to copy state");

		static_cast<ScopedState&>(*this) = static_cast<const ScopedState&>(other);
		
		return *this;
	    }
	    /** \brief Checks equality of two states */
	    template<class O>
	    bool operator==(const ScopedStateTyped<O> &other) const
	    {
		BOOST_CONCEPT_ASSERT((boost::Convertible<O*, State*>));
		return static_cast<const ScopedState&>(*this) == static_cast<const ScopedState&>(other);
	    }

	    /** \brief Checks equality of two states */	    
	    template<class O>
	    bool operator!=(const ScopedStateTyped<O> &other) const
	    {
		return !(*this == other);
	    }	    

	    /** \brief De-references to the contained state */
	    T& operator*(void) const
	    {
		return static_cast<T&>(*state_);
	    }
	    
	    /** \brief Returns a pointer to the contained state */
	    T* operator->(void) const
	    {
		return static_cast<T*>(state_);
	    }
	    
	    /** \brief Returns a pointer to the contained state */
	    T* get(void) const
	    {
		return static_cast<T*>(state_);
	    }
	    
	    /** \brief Returns a reference to the contained state */
	    T& reference(void) const
	    {
		return static_cast<T&>(*state_);
	    }
	    
	};

	// do not use this yet; tentative syntax; do we need a ScopedStateTyped ?
	template<class T>
	class ScopedStateMapped : public ScopedState, public T
	{
	public:
	    /** \brief Given the space that we are working with,
		allocate a state from the corresponding
		manifold. */
	    explicit
	    ScopedStateMapped(const SpaceInformationPtr &si) : ScopedState(si), T(state_)
	    {	
	    }
	    
	    /** \brief Given the manifold that we are working with,
		allocate a state. */
	    explicit
	    ScopedStateMapped(const StateManifoldPtr &manifold) : ScopedState(manifold), T(state_)
	    {
	    }

	    /** \brief Copy constructor */
	    ScopedStateMapped(const ScopedState &other) : ScopedState(other), T(state_)
	    {
	    }
	    
	    // etc
	};
	
	    
    }
}

#endif
