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

#ifndef OMPL_BASE_STATEHYBRID_
#define OMPL_BASE_STATEHYBRID_

#include "ompl/base/AbstractState.h"

namespace ompl
{
	namespace base
	{
	/// Definition of a hybrid state
	template<class Discrete, class Continuous>
	class StateHybrid : public AbstractState, public std::pair<Discrete, Continuous>
	{
	public:
		StateHybrid()
		{
		}
		StateHybrid(const Discrete& mode, const Continuous& state)
			: std::pair<Discrete,Continuous>(mode, state)
		{
		}
		StateHybrid* clone()
		{
			return new StateHybrid(*this);
		}
		static std::size_t size(bool continousPartOnly=true)
		{
			return continousPartOnly ? Continuous::size() 
				: Discrete::size() + Continuous::size();
		}
		int& mode(unsigned int i)
		{
			return this->first[i];
		}
		double& operator[](unsigned int i)
		{
			return this->second[i];
		}
		static std::size_t size()
		{
			return Discrete::size() + Continuous::size();
		}
		virtual double distance(const AbstractState& as) const
		{
			const StateHybrid& s = static_cast<const StateHybrid&>(as);
			return this->first.distance(s.first) + this->second.distance(s.second);
		}
		virtual StateHybrid& invert()
		{
			this->second.invert();
			return *this;
		}
		virtual StateHybrid& operator+=(const AbstractState& as)
		{
			const StateHybrid& s = static_cast<const StateHybrid&>(as);
			this->second += s.second;
			assert(this->first == s.first);
			return *this;
		}
		virtual StateHybrid operator+(const StateHybrid& s) const
		{
			return StateHybrid(*this) += s;
		}
		virtual StateHybrid operator*(const StateHybrid& s) const
		{
			return StateHybrid(*this) += s;
		}
		virtual StateHybrid& operator-=(const AbstractState& as)
		{
			const StateHybrid& s = static_cast<const StateHybrid&>(as);
			return StateHybrid(*this) += StateHybrid(s).invert();
		}
		virtual StateHybrid operator-(const StateHybrid& s) const
		{
			return StateHybrid(*this) += StateHybrid(s).invert();
		}
		virtual StateHybrid operator*(const double t) const
		{
			return StateHybrid(this->first, this->second * t);
		}
		bool operator==(const AbstractState& as) const
		{
			const StateHybrid& s = static_cast<const StateHybrid&>(as);
			return this->first == s.first && this->second == s.second;
		}
		virtual void interpolate(const AbstractState& as, const double t, 
			AbstractState& sNew) const
		{
			const StateHybrid& s = static_cast<const StateHybrid&>(as);
			Continuous c;
			assert(this->first == s.first);
			this->second.interpolate(s.second, t, c);
			static_cast<StateHybrid&>(sNew) = StateHybrid(this->first, c);
		}
		static const StateHybrid min()
		{
			static const StateHybrid sMin(Discrete::min(),Continuous::min());
			return sMin;
		}
		static const StateHybrid max() 
		{
			static const StateHybrid sMax(Discrete::max(),Continuous::max());
			return sMax;
		}
		virtual bool satisfiesBounds(const AbstractState& alo, const AbstractState& ahi) const
		{
			const StateHybrid& lo = static_cast<const StateHybrid&>(alo);
			const StateHybrid& hi = static_cast<const StateHybrid&>(ahi);
			return this->first.satisfiesBounds(lo.first, hi.first)
				&& this->second.satisfiesBounds(lo.second, hi.second);
		}
		virtual StateHybrid& enforceBounds(const AbstractState& alo, const AbstractState& ahi)
		{
			const StateHybrid& lo = static_cast<const StateHybrid&>(alo);
			const StateHybrid& hi = static_cast<const StateHybrid&>(ahi);
			this->first.enforceBounds(lo.first, hi.first);
			this->second.enforceBounds(lo.second, hi.second);
			return *this;
		}
	};
	
	}
}
#endif
