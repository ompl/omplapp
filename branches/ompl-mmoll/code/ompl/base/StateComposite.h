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

#ifndef OMPL_BASE_COMPOSITE_
#define OMPL_BASE_COMPOSITE_

#include <boost/array.hpp>
#include "ompl/base/AbstractState.h"

namespace ompl
{
	namespace base
	{
	/// Definition of a composite state
	template<std::size_t N>
	class StateComposite : public AbstractState, public boost::array<AbstractState*,N>
	{
	public:
		StateComposite() : boost::array<AbstractState*,N>() 
		{
			this->assign(NULL);
		}
		StateComposite(const StateComposite& s)
		{
			for (std::size_t i=0; i<N; ++i)
				(*this)[i] = s[i]->clone();
		}
		virtual double distance(const AbstractState& as) const
		{
			const StateComposite& s = static_cast<const StateComposite&>(as);
			double d=0.;
			for (std::size_t i=0; i<N; ++i)
				d += (*this)[i]->distance(*s[i]);
			return d;
		}
		virtual StateComposite& invert()
		{
			for (std::size_t i=0; i<N; ++i)
				(*this)[i]->invert();
			return *this;
		}
		virtual StateComposite& operator+=(const AbstractState& as)
		{
			const StateComposite& s = static_cast<const StateComposite&>(as);
			for (std::size_t i=0; i<N; ++i)
				*(*this)[i] += *s[i];
			return *this;
		}
		virtual StateComposite operator+(const StateComposite& s) const
		{
			return StateComposite(*this) += s;
		}
		virtual StateComposite& operator-=(const AbstractState& as)
		{
			const StateComposite& s = static_cast<const StateComposite&>(as);
			for (std::size_t i=0; i<N; ++i)
				*(*this)[i] -= *s[i];
			return *this;
		}
		virtual StateComposite operator-(const StateComposite& s) const
		{
			return StateComposite(*this) -= s;
		}
		virtual bool operator==(const AbstractState& as) const
		{
			const StateComposite& s = static_cast<const StateComposite&>(as);
			for(std::size_t i=0; i<N; ++i)
				if (!((*this)[i] == *s[i]))
					return false;
			return true;
		}
		virtual void interpolate(const AbstractState& as, const double t, AbstractState& asNew) const
		{
			const StateComposite& s = static_cast<const StateComposite&>(as);
			StateComposite& sNew = static_cast<StateComposite&>(asNew);
			for(std::size_t i=0; i<N; ++i)
				(*this)[i]->interpolate(s[i], t, *sNew[i]);
		}
		virtual bool satisfiesBounds(const AbstractState& alo, const AbstractState& ahi) const
		{
			const StateComposite& lo = static_cast<const StateComposite&>(alo);
			const StateComposite& hi = static_cast<const StateComposite&>(ahi);
			for (std::size_t i=0; i<N; ++i)
				if (!(*this)[i]->satisfiesBounds(*lo[i], *hi[i]))
					return false;
			return true;
		}
		virtual StateComposite& enforceBounds(const AbstractState& alo, const AbstractState& ahi)
		{
			const StateComposite& lo = static_cast<const StateComposite&>(alo);
			const StateComposite& hi = static_cast<const StateComposite&>(ahi);
			for (std::size_t i=0; i<N; ++i)
				(*this)[i]->enforceBounds(*lo[i], *hi[i]);
			return *this;
		}
	};


	}
}
#endif
