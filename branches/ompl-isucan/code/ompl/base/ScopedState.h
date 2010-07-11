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

#ifndef OMPL_BASE_SCOPED_STATE_
#define OMPL_BASE_SCOPED_STATE_

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include "boost/concept_check.hpp"

namespace ompl
{
    namespace base
    {
	
	/** \brief Definition of a scoped state. 

	    This class allocates a state of a desired type using the
	    allocation mechanism of the manifold the state is part
	    of. The state is then freed when the instance goes out of
	    scope using the corresponding free mechanism. */
	template<class T = State>
	class ScopedState
	{
	    
	    /** \brief Make sure the type we are allocating is indeed a state */
	    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));
	    
	public:
	    
	    /** \brief Given the space that we are working with, allocate a state from the corresponding
		manifold. Throw an exception if the desired type to cast this state into does not match the type of states
		allocated. */
	    explicit
	    ScopedState(const SpaceInformationPtr &si) : m_si(si)
	    {
		State *s = m_si->allocState();
		m_state = dynamic_cast<T*>(s);
		if (!m_state)
		{
		    m_si->freeState(s);
		    throw Exception("Space information does not allocate states of desired type");
		}
	    }
	    
	    /** \brief Free the memory of the internally allocated state */
	    ~ScopedState(void)
	    {
		m_si->freeState(m_state);
	    }	    
	    
	    T& operator*(void) const
	    {
		return *m_state;
	    }
	    
	    T* operator->(void) const
	    {
		return m_state;
	    }
	    
	    T* get(void) const
	    {
		return m_state;
	    }
	    
	private:
	    
	    SpaceInformationPtr  m_si;	    
	    T                   *m_state;
	};
    }
}

#endif
