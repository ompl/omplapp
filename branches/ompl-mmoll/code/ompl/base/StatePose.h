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

#ifndef OMPL_BASE_STATEROTATION_
#define OMPL_BASE_STATEROTATION_

#include "ompl/base/StateRotation.h"

namespace ompl
{
	namespace base
	{

	template <class Rotation, class Translation>
	class StatePose : public AbstractState, public std::pair<Rotation, Translation>
	{
	public:
		StatePose()
		{
		}
		StatePose(const Rotation& rot, const Translation& trans)
			: std::pair<Rotation,Translation>(rot, trans)
		{
		}
		StatePose* clone()
		{
			return new StatePose(*this);
		}
		static std::size_t size()
		{
			return Rotation::size() + Translation::size();
		}
		virtual double distance(const AbstractState& as) const
		{
			const StatePose& s = static_cast<const StatePose&>(as);
			return this->first.distance(s.first) + this->second.distance(s.second);
		}
		virtual StatePose& invert()
		{
			this->first.invert();
			this->second = this->first * this->second.invert();
			return *this;
		}
		virtual StatePose& operator+=(const AbstractState& as)
		{
			const StatePose& s = static_cast<const StatePose&>(as);
			this->second += this->first * s.second;
			this->first += s.first;
			return *this;
		}
		virtual StatePose operator+(const StatePose& s) const
		{
			return StatePose(*this) += s;
		}
		virtual StatePose operator*(const StatePose& s) const
		{
			return StatePose(*this) += s;
		}
		virtual StatePose& operator-=(const AbstractState& as)
		{
			const StatePose& s = static_cast<const StatePose&>(as);
			return StatePose(*this) += StatePose(s).invert();
		}
		virtual StatePose operator-(const StatePose& s) const
		{
			return StatePose(*this) += StatePose(s).invert();
		}
		virtual StatePose operator*(const double t) const
		{
			return StatePose(this->first, this->second * t);
		}
		virtual Translation operator*(const Translation& t) const
		{
			return this->second + this->first * t;
		}
		bool operator==(const AbstractState& as) const
		{
			const StatePose& s = static_cast<const StatePose&>(as);
			return this->first == s.first && this->second == s.second;
		}
		virtual void interpolate(const AbstractState& as, const double t, 
			AbstractState& sNew) const
		{
			const StatePose& s = static_cast<const StatePose&>(as);
			Rotation rot;
			Translation trans;
			this->first.interpolate(s.first, t, rot);
			this->second.interpolate(s.second, t, trans);
			static_cast<StatePose&>(sNew) = StatePose(rot, trans);
		}
		static const StatePose& min()
		{
			static const StatePose sMin(Rotation::min(),Translation::min());
			return sMin;
		}
		static const StatePose& max()
		{
			static const StatePose sMax(Rotation::max(),Translation::max());
			return sMax;
		}
		virtual bool satisfiesBounds(const AbstractState& alo, const AbstractState& ahi) const
		{
			const StatePose& lo = static_cast<const StatePose&>(alo);
			const StatePose& hi = static_cast<const StatePose&>(ahi);
			return this->first.satisfiesBounds(lo.first, hi.first)
				&& this->second.satisfiesBounds(lo.second, hi.second);
		}
		virtual StatePose& enforceBounds(const AbstractState& alo, const AbstractState& ahi)
		{
			const StatePose& lo = static_cast<const StatePose&>(alo);
			const StatePose& hi = static_cast<const StatePose&>(ahi);
			this->first.enforceBounds(lo.first, hi.first);
			this->second.enforceBounds(lo.second, hi.second);
			return *this;
		}
		
	};
	typedef StatePose<StateRotation2D, StatePosition2D> StatePose2D;
	typedef StatePose<StateRotation3D, StatePosition3D> StatePose3D;

	}
}

#endif
