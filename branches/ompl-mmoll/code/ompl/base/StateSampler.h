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

#ifndef OMPL_BASE_STATE_SAMPLER_
#define OMPL_BASE_STATE_SAMPLER_

#include "ompl/util/RandomNumbers.h"
#include "ompl/base/State.h"
#include <vector>

namespace ompl
{
    namespace base
    {
	
	template<class State>
	class StateSampler
	{
	public:
		typedef StateSpace<State> StateSpace_t;
		typedef State State_t;
			
		virtual void setStateSpace(StateSpace_t* statespace) 
		{
			m_statespace = statespace;
		}
		/// Sample a state
		virtual void sample()(State& state) = 0;
		/// Sample a state near another, within given bounds
		virtual void sampleNear(State& state, const State& near, const double rho) = 0;
 		/// Sample a state near another, within bounds given for each dimension
		virtual void sampleNear(State& state, const State& near, const std::vector<double>& rho) = 0;
		/// Return a reference to the random number generator used
		RNG& getRNG(void)
		{
			return m_rng;
		}
	protected:
		StateSpace_t* m_statespace;
		RNG m_rng;
	};
	
	template<class State>
	class UniformStateSampler : public StateSampler<State>
	{
	public:
		virtual void sample(State& state);
		virtual void sampleNear(State& state, const State& near, const double rho);
		virtual void sampleNear(State& state, const State& near, const State& rho);
	};
	

	template<std::size_t N>
	class UniformStateSampler : public StateSampler<State<double,N> >
	{
		virtual void sample(State_t& s)
		{
			for (unsigned int i = 0 ; i < State_t::size() ; ++i)
				s[i] = m_rng.uniformReal(m_statespace->lowerBound[i], 
					m_statespace->upperBound[i]);
		}
		virtual void sample(State_t& s, const State_t& near, const double rho)
		{
			for (unsigned int i = 0 ; i < State_t::size() ; ++i)
				m_rng.uniformReal(std::max(m_statespace->lowerBound[i], near[i] - rho), 
					std::min(m_statespace->upperBound[i], near[i] + rho));
		}
		virtual void sample(State_t& s, const State_t& near, const State_t& rho)
		{
			for (unsigned int i = 0 ; i < State_t::size() ; ++i)
				m_rng.uniformReal(std::max(m_statespace->lowerBound[i], near[i] - rho[i]), 
					std::min(m_statespace->upperBound[i], near[i] + rho[i]));
		}
	};
	
	template<std::size_t N>
	class UniformStateSampler : public StateSampler<State<int,N> >
	{
		virtual void sample(State_t& s)
		{
			for (unsigned int i = 0 ; i < State_t::size() ; ++i)
				s[i] = m_rng.uniformInt(m_statespace->lowerBound[i], 
					m_statespace->upperBound[i]);
		}
		virtual void sample(State_t& s, const State_t& near, const double rho)
		{
			for (unsigned int i = 0 ; i < State_t::size() ; ++i)
				m_rng.uniformInt(std::max(m_statespace->lowerBound[i], near[i] - rho), 
					std::min(m_statespace->upperBound[i], near[i] + rho));
		}
		virtual void sample(State_t& s, const State_t& near, const State_t& rho)
		{
			for (unsigned int i = 0 ; i < State_t::size() ; ++i)
				m_rng.uniformInt(std::max(m_statespace->lowerBound[i], near[i] - rho[i]), 
					std::min(m_statespace->upperBound[i], near[i] + rho[i]));
		}
	};
	
	template<>
	class UniformStateSampler : public StateSampler<Rotation3D>
	{
		virtual void sample(State_t& s)
		{
			m_rng.quaternion(s.data());
		}
		virtual void sample(State_t& s, const State_t& near, const double rho)
		{
			sample(s);
		}
		virtual void sample(State_t& s, const State_t& near, const State_t& rho)
		{
			sample(s);
		}		
	};
	
	template<class T1, class T2>
	class UniformStateSampler : public StateSampler<std::pair<T1, T2> >
	{
		virtual void sample(State_t& s)
		{
			sample(s.first);
			sample(s.second);
		}
		virtual void sample(State_t& s, const State_t& near, const double rho)
		{
			sample(s.first, near.first, rho);
			sample(s.second, near.second, rho);
		}
		virtual void sample(State_t& s, const State_t& near, const State_t& rho)
		{
			sample(s.first, near.first, rho);
			sample(s.second, near.second, rho);
		}		
	};

	template<>
	class UniformStateSampler : public StateSampler<StateComposite>
	{
		virtual void sample(State_t& s)
		{
			for (unsigned int i=0; i<State_t::size(); ++i)
				sample(*s[i]);
		}
		virtual void sample(State_t& s, const State_t& near, const double rho)
		{
			for (unsigned int i=0; i<State_t::size(); ++i)
				sample(*s[i], *near[i], rho);
		}
		virtual void sample(State_t& s, const State_t& near, const State_t& rho)
		{
			for (unsigned int i=0; i<State_t::size(); ++i)
				sample(*s[i], *near[i], *rho[i]);
		}		
	};
	
	
	}

}


#endif
