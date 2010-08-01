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

#ifndef OMPL_BASE_STATE_
#define OMPL_BASE_STATE_

#include <boost/concept_check.hpp>

namespace ompl
{
    namespace base
    {
	
	/** \page stateAlloc Allocating memory for states

	    \li The simple version:\n
	    \code
	    ompl::base::StateManifoldPtr manifold(new T());
	    ompl::base::ScopedState<> state(manifold);
	    \endcode
	    or
	    \code
	    ompl::base::SpaceInformationPtr si(manifold);
	    ompl::base::ScopedState<T> state(si);
	    \endcode
	    The ompl::base::ScopedState class will do the necessary
	    memory operations to allocate a state from the correct
	    manifold. This is the recommended way of allocating states
	    for code other than ompl internals. Convenience operators such
	    as = and == are provided. If a type T is provided, where T is 
	    a manifold type, the maintained state is cast as T::StateType.

	    \li The expert version:\n
	    \code
	    ompl::base::SpaceInformationPtr si(manifold);
	    ompl::base::State* state = si->allocState();
	    ...
	    si->freeState(state);
	    \endcode
	    The structure of a state depends on a manifold
	    specification. The State type is just an abstract base for
	    the states of other manifolds.  For this reason, states
	    cannot be allocated directly, but through the use of a
	    manifold's allocation mechanism:
	    ompl::base::StateManifold::allocState(). States are to be
	    freed using ompl::base::StateManifold::freeState(). For
	    convenience, ompl::base::SpaceInformation::allocState()
	    and ompl::base::SpaceInformation::freeState() are defined
	    as well. Using the calls from the SpaceInformation class
	    is better since they certainly use the same manifold as
	    the one used for planning.  This is the lowest level of
	    operating on states and only recomended for expert users.

	    See \ref stateOps for how to fill the contents of the
	    allocated states.
	*/

	/** \page stateOps Operating with states

	    In order for states to be useful in setting start (or
	    goal) positions, accessing their content is needed. It is
	    assumed the reader is familiar with \ref stateAlloc.

	    \li Simple version:\n
	    After a state has been allocated from manifold T, no
	    matter what the state type is: State *, State& or
	    ScopedState&, an instance of T::Mapper can be defined to
	    allow access to the state's members.
	    \code
	    ompl::base::StateManifoldPtr manifold(new ompl::base::SE2StateManifold());
	    ompl::base::ScopedState<SE2StateManifold> state(manifold);
	    state->setX(...);
	    \endcode

	    \li Expert version:\n	    
	    \code
	    ompl::base::StateManifoldPtr manifold(new ompl::base::RealVectorStateManifold());
	    ompl::base::State *state = manifold->allocState();
	    state->as<ompl::base::RealVectorStateManifold::StateType>()->values[0] = ...;
	    manifold->freeState(state);
	    \endcode
	    For a manifold type of type T, the result of
	    ompl::base::StateManifold::allocState() can be casted to
	    T::StateType to gain access to the state's members. To
	    ease this functionality, the ompl::base::State::as()
	    functions have been defined.

	*/

	/** \brief Definition of an abstract state.
	 
	    See \ref stateAlloc and \ref stateOps. */
	class State
	{
	private:
	    
	    /** \brief Disable copy-constructor */
	    State(const State&);
	    
	    /** \brief Disable copy operator */
	    const State& operator=(const State&);
	    
	protected:
	    
	    State(void)
	    {
	    }
#ifdef __GCCXML__
	// This is needed for the Python bindings. The wrapping code generated by
	// Py++ needs, although it's not 100% clear why.
	public:
#endif
	    virtual ~State(void)
	    {
	    }
	    
	public:

	    /** \brief Cast this instance to a desired type. */
	    template<class T>
	    const T* as(void) const
	    {
		/** \brief Make sure the type we are allocating is indeed a state */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

		return static_cast<const T*>(this);
	    }

	    /** \brief Cast this instance to a desired type. */
	    template<class T>
	    T* as(void)
	    {	
		/** \brief Make sure the type we are allocating is indeed a state */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));
		
		return static_cast<T*>(this);
	    }
	    
	};

	/** \brief Definition of a compound state */
	class CompoundState : public State
	{
	public:
	    
	    CompoundState(void) : State()
	    {
	    }
	    
	    virtual ~CompoundState(void)
	    {
	    }

	    /** \brief Cast a component of this instance to a desired type. */
	    template<class T>
	    const T* as(const unsigned int index) const
	    {	    	
		/** \brief Make sure the type we are allocating is indeed a state */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

		return static_cast<const T*>(components[index]);
	    }

	    /** \brief Cast a component of this instance to a desired type. */
	    template<class T>
	    T* as(const unsigned int index)
	    {
		/** \brief Make sure the type we are allocating is indeed a state */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));
		
		return static_cast<T*>(components[index]);
	    }

	    /** \brief Access element i<sup>th</sup> component. This
		does not check whether the index is within bounds. */
	    State* operator[](unsigned int i) const
	    {
		return components[i];
	    }
	    
	    /** \brief The components that make up a compound state */
	    State **components;
	};

    }
}

#endif
