/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of Rice University nor the names of its
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

/* \author Ioan Sucan, Mark Moll */

#ifndef OMPL_BASE_STATESPACE_
#define OMPL_BASE_STATESPACE_

#include "State.h"

namespace ompl
{
	namespace base
	{
	
	/// Definition of a state space
	template<class State>
	class StateSpace
	{
		typedef StateSampler<State> StateSampler_t;
		typedef StateValidityChecker<State> StateValidityChecker_t;
		
		StateSpace(const StateSampler_t& sampler, 
			const StateValidityChecker_t& validityChecker)
			: m_sampler(sampler), m_validityChecker(validityChecker)
		{
			m_sampler.setStateSpace(this);
			m_validityChecker.setStateSpace(this);
		}
		
		unsigned int stateDimensionality()
		{
			return State::size();
		}

		/// Check if a given state is valid or not
	    virtual bool isValid(const State& state) const
		{
			m_validityChecker(state);
		}

		/// Set bounding box
		void setBoundingBox(const State& lo, const State& hi)
		{
			m_lowerBound = lo;
			m_upperBound = hi;
		}
		/// Get bounding box
		const getBoundingBox(State& lo, State& hi)
		{
			lo = m_lowerBound;
			hi = m_upperBound;
		}
		/// Check if a state is inside the bounding box
		bool satisfiesBounds(const State& s) const
		{
			return s.satisfiesBounds(m_lowerBound, m_upperBound);
		}
	    /// Bring the state within the bounds of the state space
	    void enforceBounds(State& state) const
		{
			s.enforceBounds(m_lowerBound, m_upperBound);
		}

		/// Compute the distance between two states
	    virtual double distance(const State& s1, const State& s2) const
	    {
			return s1.distance(s2);
	    }

		/// \brief Find a valid state near a given one. If the given state
		/// is valid, it will be returned itself. The two passed states 
		/// must refer to different states. Returns true on success.
		bool searchValidNearby(State& state, const State& near, 
			const std::vector<double> &rho, unsigned int attempts) const;

	protected:
		State                  m_lowerBound;
		State                  m_upperBound;
		StateSampler_t         m_sampler;
		StateValidityChecker_t m_validityChecker;
		msg::Interface         m_msg;
		
		friend class StateSampler_t;
		friend class StateValidityChecker_t;
	};
	
	}
}

#endif
