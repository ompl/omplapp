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
	
	/** \brief Definition of an abstract state 

	    @anchor stateOps
	    
	    \par Operating with states
	    
	    \li Simple case 1:\n
	    In the simplest scenario, after a manifold of type T is
	    defined, a state can be instantiated from that manifold
	    using ompl::base::ScopedStateTyped. This will
	    automatically allocate and free the state. The template
	    argument is optional and specifies the state type to be
	    maintained.

	    \code
	    ompl::base::StateManifold manifold(new T());
	    ompl::base::ScopedStateTyped<T::StateType> state(manifold);
	    state->value = ...;
	    \endcode

	    \li Simple case 2:\n	    
	    In some cases, for instance, the
	    ompl::base::SE2StateManifold, the layout of the state uses
	    a CompoundState, which makes accessing members a little
	    more complicated. To alleviate this, a manifold of type T
	    can provide a T::Mapper class that provides setters and
	    getters for the various members of the state. The
	    T::Mapper class does not create or even store a state. It
	    simply allows updating the state's content.

	    \code
	    ompl::base::StateManifold manifold(new T());
	    ompl::base::ScopedState state(manifold);
	    T::Mapper mapper(state);
	    mapper.setValue(...);
	    \endcode

	    \li Expert users:\n
	    The structure of a state depends on a manifold
	    specification. The State type is just an abstract base for
	    the states of other manifolds.  For this reason, states
	    cannot be allocated directly, but through the use of a
	    manifold's allocation mechanism:
	    ompl::base::StateManifold::allocState(). States are to be
	    freed using ompl::base::StateManifold::freeState(). For a
	    manifold type of type T, the result of
	    ompl::base::StateManifold::allocState() can be casted to
	    T::StateType to gain access to the state's members. For
	    convenience, it ompl::base::SpaceInformation::allocState()
	    and ompl::base::SpaceInformation::freeState() are defined
	    as well. Using these calls is better since they certainly
	    use the same manifold as the one used for planning.  This
	    is the lowest level of operating on states and only
	    recomended for expert users.
	*/
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
