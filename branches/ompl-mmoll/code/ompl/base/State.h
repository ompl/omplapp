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

#ifndef OMPL_BASE_STATE_
#define OMPL_BASE_STATE_

#include <iostream>
#include <iterator>
#include <limits>
#include <functional>
#include <numeric>
#include <cstdlib>
#include <cmath>
#include <boost/array.hpp>
#include "ompl/base/AbstractState.h"

namespace ompl
{
	namespace base
	{
	
	/// Definition of a state
	template<class T, std::size_t N>
	class State : public AbstractState, public boost::array<T,N>
	{
	public:
		State() : boost::array<T,N>() {}
		State(const T t)
		{
			assign(t);
		}
		State(const T x, const T y)
		{
			assert(N>1);
			(*this)[0] = x;
			(*this)[1] = y;
		}
		State(const T x, const T y, const T z)
		{
			assert(N>2);
			(*this)[0] = x;
			(*this)[1] = y;
			(*this)[2] = z;
		}
		State* clone()
		{
			return new State(*this);
		}
		virtual double norm_sq() const
		{
			double sum=0, x;
			for (std::size_t i=0; i<N; ++i)
			{
				x = (*this)[i];
				sum += x*x;
			}
			return sum;
		}
		virtual double norm() const
		{
			return std::sqrt(norm_sq());
		
		}
		virtual double distance(const AbstractState& s) const
		{
			return (*this - static_cast<const State&>(s)).norm();
		}
		virtual State& invert()
		{
			std::transform(this->begin(), this->end(), this->begin(),
				std::negate<T>());
			return *this;
		}
		virtual State& operator+=(const AbstractState& s)
		{
			std::transform(this->begin(), this->end(), static_cast<const State&>(s).begin(),
				this->begin(), std::plus<T>());
			return *this;
		}
		virtual State operator+(const State& s) const
		{
			return State(*this) += s;
		}
		virtual State& operator-=(const AbstractState& s)
		{
			std::transform(this->begin(), this->end(), static_cast<const State&>(s).begin(),
				this->begin(), std::minus<T>());
			return *this;
		}
		virtual State operator-(const State& s) const
		{
			return State(*this) -= s;
		}
		virtual State operator*(const T t) const
		{
			State result;
			for (std::size_t i=0; i<N; ++i) 
				result[i] = t * (*this)[i];
			return result;
		}
		virtual bool operator==(const AbstractState& as) const
		{
			const State& s = static_cast<const State&>(as);
			for(std::size_t i=0; i<N; ++i)
				if (std::abs((*this)[i]-s[i]) > epsilon<T>::value)
					return false;
			return true;
		}
		virtual void interpolate(const AbstractState& s, const double t, AbstractState& sNew) const
		{
			static_cast<State&>(sNew) = *this + (static_cast<const State&>(s)-*this)*t;
		}
		virtual T dot(const State& s) const
		{
			return std::inner_product(this->begin(),this->end(),s.begin(),0);
		}
		static const State& min()
		{
			static const State sMin(-std::numeric_limits<T>::max());
			return sMin;
		}
		static const State& max()
		{
			static const State sMax(std::numeric_limits<T>::max());
			return sMax;
		}
		virtual bool satisfiesBounds(const AbstractState& alo = min(), 
			const AbstractState& ahi = max()) const
		{
			const State& lo = static_cast<const State&>(alo);
			const State& hi = static_cast<const State&>(ahi);
			for (std::size_t i=0; i<N; ++i)
				if ((*this)[i] < lo[i] || (*this)[i] > hi[i])
					return false;
			return true;
		}
		virtual State& enforceBounds(const AbstractState& alo = min(), 
			const AbstractState& ahi = max())
		{
			const State& lo = static_cast<const State&>(alo);
			const State& hi = static_cast<const State&>(ahi);
			for (std::size_t i=0; i<N; ++i)
				if ((*this)[i] < lo[i])
					(*this)[i] = lo[i];
				else if ((*this)[i] > hi[i])
					(*this)[i] = hi[i];
			return *this;
		}
	};
	typedef State<double, 1> StatePosition1D;
	typedef State<double, 2> StatePosition2D;
	typedef State<double, 3> StatePosition3D;
	
	template<class T, std::size_t N>
	std::ostream& operator<<(std::ostream& out, const State<T,N>& s)
	{
		std::ostream_iterator<double> out_it (out," ");
		std::copy(s.begin(), s.end(), out_it );
		return out;
	}
	
	template<class T1, class T2>
	std::ostream& operator<<(std::ostream& out, const std::pair<T1, T2>& s)
	{
		return out << s.first << "| " << s.second;
	}
	
	}

}

#endif
